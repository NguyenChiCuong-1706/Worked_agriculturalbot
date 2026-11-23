import os
import re
import signal
import shlex
import subprocess
import time
import json
from pathlib import Path

from flask import Flask, request, jsonify, render_template


app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = int(os.environ.get("VOICE_NAV_MAX_UPLOAD_BYTES", 10 * 1024 * 1024))

# =========================
# Path configuration
# =========================

HOME = Path.home()
APP_DIR = Path(__file__).resolve().parent

WHISPER_DIR = HOME / "whisper.cpp"
WHISPER_BIN = WHISPER_DIR / "build/bin/whisper-cli"
MODEL_PATH = WHISPER_DIR / "models/ggml-small.en.bin"

UPLOAD_DIR = Path("/tmp/voice_nav")
UPLOAD_DIR.mkdir(parents=True, exist_ok=True)

ROS_SETUP = "/opt/ros/kilted/setup.bash"
ROS_WS_SETUP = os.environ.get("ROS_WS_SETUP", str(HOME / "carbot_ws/install/setup.bash"))
VOICE_TOPIC = "/voice_command_text"
CMD_VEL_TOPIC = "/cmd_vel_teleop"
NAVIGATE_TO_POSE_ACTION = os.environ.get("NAVIGATE_TO_POSE_ACTION", "/navigate_to_pose")
NAV_GOAL_HELPER = APP_DIR / "send_nav_goal.py"
NAV_RUNTIME_HELPER = APP_DIR / "nav_runtime.py"
ROUTE_SERVER_NODE = os.environ.get("ROUTE_SERVER_NODE", "/route_server")
NAV2_PARAMS_FILE = Path(
    os.environ.get(
        "NAV2_PARAMS_FILE",
        HOME / "carbot_ws/src/Carbot/fl_nav2_helper/config/nav2_params.yaml",
    )
).expanduser()
ROBOT_TASK_CONFIG = Path(
    os.environ.get(
        "ROBOT_TASK_CONFIG",
        APP_DIR / "navigation_config.json"
    )
).expanduser()
ROUTE_GRAPH_OVERRIDE = os.environ.get("ROUTE_GRAPH_FILE")

CERT_FILE = Path(os.environ.get("VOICE_NAV_CERT", "cert.pem")).expanduser()
if not CERT_FILE.is_absolute():
    CERT_FILE = APP_DIR / CERT_FILE

KEY_FILE = Path(os.environ.get("VOICE_NAV_KEY", "key.pem")).expanduser()
if not KEY_FILE.is_absolute():
    KEY_FILE = APP_DIR / KEY_FILE

PROMPT = (
    "Robot navigation commands: go to home station, go to row number one, "
    "go to row number two, go to charging station, go to lab, go to the stair, "
    "stop robot, cancel navigation."
)

robot_task_processes = {}
ROBOT_TASK_STATE_DIR = UPLOAD_DIR / "task_state"
ROBOT_TASK_STATE_DIR.mkdir(parents=True, exist_ok=True)


def resolve_config_path(value, base_dir: Path) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return (base_dir / path).resolve()


def load_robot_task_config():
    """
    Load robot task launch settings from JSON so deployment-specific paths can
    change without editing this Flask server.
    """
    default_workdir = (HOME / "carbot_ws").resolve()
    config = {
        "ros_setup": ROS_SETUP,
        "workspace_setup": "install/setup.bash",
        "tasks": {
            "navigation": {
                "label": "Navigation",
                "script": str(default_workdir / "navigation.sh"),
                "workdir": str(default_workdir),
                "log": "navigation.log",
            },
        },
    }

    if ROBOT_TASK_CONFIG.exists():
        with ROBOT_TASK_CONFIG.open("r", encoding="utf-8") as config_file:
            user_config = json.load(config_file)
        config.update(user_config)

    config_dir = ROBOT_TASK_CONFIG.parent
    tasks = {}
    for task_id, task_config in config.get("tasks", {}).items():
        script = resolve_config_path(task_config["script"], config_dir)
        workdir = resolve_config_path(task_config.get("workdir", default_workdir), config_dir)
        workspace_setup = Path(
            task_config.get("workspace_setup", config.get("workspace_setup", workdir / "install/setup.bash"))
        ).expanduser()
        if not workspace_setup.is_absolute():
            workspace_setup = (workdir / workspace_setup).resolve()

        log_name = task_config.get("log", f"{task_id}.log")
        log_path = Path(log_name).expanduser()
        if not log_path.is_absolute():
            log_path = UPLOAD_DIR / log_path

        tasks[task_id] = {
            "id": task_id,
            "label": task_config.get("label", task_id.title()),
            "ros_setup": str(resolve_config_path(task_config.get("ros_setup", config.get("ros_setup", ROS_SETUP)), config_dir)),
            "script": script,
            "workdir": workdir,
            "workspace_setup": workspace_setup,
            "log": log_path,
        }

    return tasks


ROBOT_TASKS = load_robot_task_config()


# =========================
# Utility functions
# =========================

def check_required_files():
    missing = []

    if not WHISPER_BIN.exists():
        missing.append(str(WHISPER_BIN))

    if not MODEL_PATH.exists():
        missing.append(str(MODEL_PATH))

    if not Path(ROS_SETUP).exists():
        missing.append(ROS_SETUP)

    if not Path(ROS_WS_SETUP).exists():
        missing.append(ROS_WS_SETUP)

    if not NAV_GOAL_HELPER.exists():
        missing.append(str(NAV_GOAL_HELPER))

    if not NAV_RUNTIME_HELPER.exists():
        missing.append(str(NAV_RUNTIME_HELPER))

    if not NAV2_PARAMS_FILE.exists():
        missing.append(str(NAV2_PARAMS_FILE))

    if not CERT_FILE.exists():
        missing.append(str(CERT_FILE))

    if not KEY_FILE.exists():
        missing.append(str(KEY_FILE))

    if missing:
        print("\nERROR: Missing required files:")
        for item in missing:
            print(f"  - {item}")
        print("Fix the paths in server.py before running.\n")
        return False

    return True


def clean_whisper_text(text: str) -> str:
    """
    Clean whisper.cpp output before publishing to ROS 2.
    """
    text = text.replace("\n", " ")
    text = text.replace("\r", " ")
    text = text.strip()

    # Remove common empty/no-speech outputs
    text = text.replace("[BLANK_AUDIO]", "")
    text = text.replace("[ Silence ]", "")
    text = text.replace("[silence]", "")

    # Keep words, numbers, spaces, and basic punctuation.
    # The route_mission_client will do final normalization.
    text = re.sub(r"\s+", " ", text)
    text = text.strip(" .,:;!?\"'")

    return text


def route_graph_from_nav2_params():
    """Read route_server.graph_filepath without requiring PyYAML in the web venv."""
    if not NAV2_PARAMS_FILE.exists():
        raise FileNotFoundError(f"Nav2 parameter file not found: {NAV2_PARAMS_FILE}")

    in_route_server = False
    with NAV2_PARAMS_FILE.open("r", encoding="utf-8") as params_file:
        for line in params_file:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue

            if not line[0].isspace():
                in_route_server = stripped == "route_server:"
                continue

            if not in_route_server:
                continue

            match = re.match(r"^\s*graph_filepath\s*:\s*(.*?)\s*(?:#.*)?$", line)
            if not match:
                continue

            value = match.group(1).strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                value = value[1:-1]
            if not value:
                break

            graph_file = Path(value).expanduser()
            if not graph_file.is_absolute():
                graph_file = (NAV2_PARAMS_FILE.parent / graph_file).resolve()
            return graph_file

    raise ValueError(
        f"route_server.ros__parameters.graph_filepath was not found in {NAV2_PARAMS_FILE}"
    )


def active_route_graph_from_ros():
    """Return the graph loaded by the running Nav2 route server, if available."""
    cmd = (
        f"source {shlex.quote(ROS_SETUP)} && "
        f"source {shlex.quote(ROS_WS_SETUP)} && "
        f"ros2 param get {shlex.quote(ROUTE_SERVER_NODE)} graph_filepath"
    )
    try:
        result = subprocess.run(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None

    if result.returncode != 0:
        return None

    match = re.search(r"(?:String value is:|value is:)\s*(.+?)\s*$", result.stdout)
    if not match:
        return None

    value = match.group(1).strip().strip("\"'")
    if not value:
        return None
    return Path(value).expanduser()


def resolve_route_graph_file():
    active_graph = active_route_graph_from_ros()
    if active_graph is not None:
        return active_graph, "active_nav2_parameter"

    if ROUTE_GRAPH_OVERRIDE:
        return Path(ROUTE_GRAPH_OVERRIDE).expanduser(), "environment_override"

    return route_graph_from_nav2_params(), "nav2_params_file"


def load_route_graph_nodes(graph_file):
    if not graph_file.exists():
        raise FileNotFoundError(f"Route graph file not found: {graph_file}")

    with graph_file.open("r", encoding="utf-8") as route_graph:
        graph = json.load(route_graph)

    nodes = []
    for feature in graph.get("features", []):
        geometry = feature.get("geometry") or {}
        properties = feature.get("properties") or {}
        coordinates = geometry.get("coordinates") or []

        if geometry.get("type") != "Point" or len(coordinates) < 2:
            continue

        node_id = properties.get("id")
        if node_id is None:
            continue

        try:
            node_id = int(node_id)
            x = float(coordinates[0])
            y = float(coordinates[1])
            yaw = float(properties.get("yaw", 0.0))
        except (TypeError, ValueError):
            continue

        nodes.append({
            "id": node_id,
            "x": x,
            "y": y,
            "yaw": yaw,
            "label": str(properties.get("label") or properties.get("name") or f"Node {node_id}"),
            "frame": properties.get("frame", "map"),
        })

    nodes.sort(key=lambda item: item["id"])
    return nodes


def publish_voice_text_to_ros(text: str) -> bool:
    """
    Publish recognized text to ROS 2 topic:
      /voice_command_text std_msgs/msg/String

    This uses 'ros2 topic pub --once' because the Flask server runs inside
    a Python virtual environment and may not directly import rclpy.
    """
    clean_text = clean_whisper_text(text)

    if not clean_text:
        print("No valid text to publish to ROS 2.")
        return False

    # Escape single quotes for ROS YAML message
    ros_text = clean_text.replace("\\", "\\\\").replace("'", "''")
    msg_arg = "{data: '" + ros_text + "'}"

    cmd = (
        f"source {shlex.quote(ROS_SETUP)} && "
        f"source {shlex.quote(ROS_WS_SETUP)} && "
        f"ros2 topic pub --once {shlex.quote(VOICE_TOPIC)} "
        f"std_msgs/msg/String {shlex.quote(msg_arg)}"
    )

    try:
        result = subprocess.run(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=8,
            check=False,
        )

        if result.returncode != 0:
            print("\nERROR: Failed to publish to ROS 2.")
            print("STDOUT:", result.stdout)
            print("STDERR:", result.stderr)
            return False

        print(f"Published to {VOICE_TOPIC}: {clean_text}")
        return True

    except subprocess.TimeoutExpired:
        print("ERROR: ROS 2 topic publish timed out.")
        return False

    except Exception as exc:
        print(f"ERROR: Exception while publishing to ROS 2: {exc}")
        return False


def send_navigation_goal_to_ros(node):
    """Send a NavigateToPose goal and wait only for its acceptance response."""
    cmd = (
        f"source {shlex.quote(ROS_SETUP)} && "
        f"source {shlex.quote(ROS_WS_SETUP)} && "
        f"exec /usr/bin/python3 {shlex.quote(str(NAV_GOAL_HELPER))} "
        f"--action {shlex.quote(NAVIGATE_TO_POSE_ACTION)} "
        f"--frame {shlex.quote(str(node.get('frame') or 'map'))} "
        f"--x {shlex.quote(str(float(node['x'])))} "
        f"--y {shlex.quote(str(float(node['y'])))} "
        f"--yaw {shlex.quote(str(float(node.get('yaw', 0.0))))}"
    )

    try:
        result = subprocess.run(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=12,
            check=False,
        )

        response = None
        for line in reversed(result.stdout.splitlines()):
            try:
                response = json.loads(line)
                break
            except json.JSONDecodeError:
                continue

        if not isinstance(response, dict):
            response = {
                "accepted": False,
                "state": "helper_error",
                "error": result.stderr.strip() or result.stdout.strip() or "Goal helper returned no response.",
            }

        if result.returncode != 0 or not response.get("accepted", False):
            print("\nERROR: Nav2 did not accept the route node goal.")
            print("Response:", response)
            print("STDERR:", result.stderr)
            return False, response

        print(
            f"Nav2 accepted route node {node['id']} as goal "
            f"{response.get('goal_id', '<unknown>')}"
        )
        return True, response
    except subprocess.TimeoutExpired:
        response = {
            "accepted": False,
            "state": "timeout",
            "error": "Timed out waiting for Nav2 to respond to the goal request.",
        }
        print("ERROR:", response["error"])
        return False, response

    except Exception as exc:
        response = {
            "accepted": False,
            "state": "helper_error",
            "error": f"Exception while sending the Nav2 goal: {exc}",
        }
        print("ERROR:", response["error"])
        return False, response


def convert_audio_to_wav(input_path: Path, output_path: Path) -> bool:
    """
    Convert browser audio to 16 kHz mono WAV for whisper.cpp.
    """
    cmd = [
        "ffmpeg",
        "-y",
        "-i", str(input_path),
        "-ar", "16000",
        "-ac", "1",
        str(output_path),
    ]

    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    if result.returncode != 0:
        print("\nERROR: ffmpeg conversion failed.")
        print(result.stderr)
        return False

    return True


def run_whisper(wav_path: Path) -> str:
    """
    Run whisper.cpp on one recorded WAV command.
    """
    cmd = [
        str(WHISPER_BIN),
        "-m", str(MODEL_PATH),
        "-f", str(wav_path),
        "-nt",
        "-l", "en",
        "-t", "8",
        "--prompt", PROMPT,
    ]

    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    if result.returncode != 0:
        print("\nERROR: whisper.cpp failed.")
        print("STDOUT:", result.stdout)
        print("STDERR:", result.stderr)
        return ""

    return clean_whisper_text(result.stdout)


def run_nav_runtime_helper(arguments, timeout=10):
    cmd = (
        f"source {shlex.quote(ROS_SETUP)} && "
        f"source {shlex.quote(ROS_WS_SETUP)} && "
        f"exec /usr/bin/python3 {shlex.quote(str(NAV_RUNTIME_HELPER))} "
        + " ".join(shlex.quote(str(argument)) for argument in arguments)
    )
    try:
        result = subprocess.run(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "state": "timeout", "error": "ROS runtime check timed out."}

    for line in reversed(result.stdout.splitlines()):
        try:
            payload = json.loads(line)
            if isinstance(payload, dict):
                return payload
        except json.JSONDecodeError:
            continue
    return {
        "ok": False,
        "state": "helper_error",
        "error": result.stderr.strip() or result.stdout.strip() or "ROS helper returned no response.",
    }


def task_state_file(task_id):
    return ROBOT_TASK_STATE_DIR / f"{task_id}.json"


def process_start_ticks(pid):
    stat_text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    fields_after_name = stat_text[stat_text.rfind(")") + 2:].split()
    return int(fields_after_name[19])


def write_task_state(task_id, pid, task):
    state = {
        "pid": pid,
        "process_start_ticks": process_start_ticks(pid),
        "script": str(task["script"]),
        "started_at": time.time(),
    }
    task_state_file(task_id).write_text(json.dumps(state), encoding="utf-8")


def read_validated_task_pid(task_id):
    state_path = task_state_file(task_id)
    if not state_path.exists():
        return None
    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
        pid = int(state["pid"])
        expected_script = str(get_robot_task(task_id)["script"])
        if state.get("script") != expected_script:
            raise ValueError("PID state does not match the configured task script")
        if int(state["process_start_ticks"]) != process_start_ticks(pid):
            raise ValueError("PID was reused by another process")
        os.kill(pid, 0)
        return pid
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        state_path.unlink(missing_ok=True)
        return None


def get_robot_task(task_id):
    return ROBOT_TASKS.get(task_id)


def is_robot_task_running(task_id):
    process = robot_task_processes.get(task_id)
    if process is not None and process.poll() is None:
        return True
    return read_validated_task_pid(task_id) is not None


def robot_task_status(task_id):
    task = get_robot_task(task_id)
    process = robot_task_processes.get(task_id)
    managed_pid = read_validated_task_pid(task_id) if task else None
    runtime = {}
    if task_id == "navigation" and task is not None:
        runtime = run_nav_runtime_helper(["status"], timeout=5)

    ros_running = bool(runtime.get("running", False))
    managed_running = managed_pid is not None
    process_failed = process is not None and process.poll() not in (None, 0)
    if runtime.get("ready"):
        state = "ready"
    elif runtime.get("state") == "failed":
        state = "failed"
    elif ros_running or managed_running:
        state = "starting"
    elif process_failed:
        state = "failed"
    else:
        state = "stopped"

    return {
        "ok": task is not None,
        "task": task_id,
        "label": task["label"] if task else task_id,
        "running": ros_running or managed_running,
        "managed": managed_running,
        "pid": managed_pid,
        "state": state,
        "ready": bool(runtime.get("ready", False)),
        "localization_ready": bool(runtime.get("localization_ready", False)),
        "action_ready": bool(runtime.get("action_ready", False)),
        "lifecycle": runtime.get("lifecycle", {}),
        "runtime_error": runtime.get("error"),
        "script": str(task["script"]) if task else None,
        "workdir": str(task["workdir"]) if task else None,
        "config": str(ROBOT_TASK_CONFIG),
        "log": str(task["log"]) if task else None,
    }


def launch_robot_task(task_id):
    """
    Start one ROS 2 robot task script in the background.
    """
    task = get_robot_task(task_id)
    if task is None:
        return False, f"Unknown robot task: {task_id}", None

    if is_robot_task_running(task_id):
        pid = read_validated_task_pid(task_id)
        return False, f"{task['label']} is already running with PID {pid}.", pid

    if task_id == "navigation":
        runtime = run_nav_runtime_helper(["status"], timeout=5)
        if runtime.get("running"):
            return False, f"{task['label']} is already running outside this web process.", None

    if not task["script"].exists():
        return False, f"Missing script: {task['script']}", None

    if not Path(task["ros_setup"]).exists():
        return False, f"Missing ROS setup: {task['ros_setup']}", None

    if not task["workspace_setup"].exists():
        return False, f"Missing workspace setup: {task['workspace_setup']}", None

    cmd = (
        f"source {shlex.quote(task['ros_setup'])} && "
        f"source {shlex.quote(str(task['workspace_setup']))} && "
        f"bash {shlex.quote(str(task['script']))}"
    )

    log_file = task["log"].open("a", encoding="utf-8")
    log_file.write(f"\n\n===== Starting {task['label']} at {time.ctime()} =====\n")
    log_file.flush()

    process = subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=str(task["workdir"]),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    robot_task_processes[task_id] = process
    write_task_state(task_id, process.pid, task)
    log_file.close()

    return True, f"{task['label']} launch started.", process.pid


def get_rosbridge_url():
    configured_url = os.environ.get("ROSBRIDGE_URL")
    if configured_url:
        return configured_url

    scheme = os.environ.get(
        "ROSBRIDGE_SCHEME",
        "wss" if request.is_secure else "ws",
    )
    host = os.environ.get("ROSBRIDGE_HOST", request.host.split(":")[0])
    port = os.environ.get("ROSBRIDGE_PORT", "9090")
    return f"{scheme}://{host}:{port}"


def stop_robot_task(task_id):
    """
    Stop a running ROS 2 robot task with SIGINT, like pressing Ctrl+C.
    """
    task = get_robot_task(task_id)
    if task is None:
        return False, f"Unknown robot task: {task_id}", None

    pid = read_validated_task_pid(task_id)
    if pid is None:
        runtime = run_nav_runtime_helper(["status"], timeout=5) if task_id == "navigation" else {}
        if runtime.get("running"):
            return False, f"{task['label']} is running, but it was not started by this web controller.", None
        return False, f"{task['label']} is not running.", None

    process = robot_task_processes.get(task_id)

    with task["log"].open("a", encoding="utf-8") as log_file:
        log_file.write(f"\n===== Stopping {task['label']} at {time.ctime()} =====\n")

    try:
        os.killpg(pid, signal.SIGINT)
        if process is not None:
            process.wait(timeout=12)
        else:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and Path(f"/proc/{pid}").exists():
                time.sleep(0.2)
            if Path(f"/proc/{pid}").exists():
                raise subprocess.TimeoutExpired(str(task["script"]), 12)
    except subprocess.TimeoutExpired:
        os.killpg(pid, signal.SIGTERM)
        task_state_file(task_id).unlink(missing_ok=True)
        return True, f"{task['label']} did not stop after Ctrl+C, sent terminate signal.", pid
    except ProcessLookupError:
        pass

    task_state_file(task_id).unlink(missing_ok=True)
    return True, f"{task['label']} stopped.", pid


# =========================
# Flask routes
# =========================

@app.route("/")
def index():
    return render_template("index.html", rosbridge_url=get_rosbridge_url())


@app.route("/health", methods=["GET"])
def health():
    try:
        route_graph_file, route_graph_source = resolve_route_graph_file()
    except Exception as exc:
        route_graph_file = None
        route_graph_source = f"error: {exc}"

    return jsonify({
        "status": "ok",
        "whisper_bin": str(WHISPER_BIN),
        "model_path": str(MODEL_PATH),
        "voice_topic": VOICE_TOPIC,
        "cmd_vel_topic": CMD_VEL_TOPIC,
        "navigate_to_pose_action": NAVIGATE_TO_POSE_ACTION,
        "route_server_node": ROUTE_SERVER_NODE,
        "nav2_params_file": str(NAV2_PARAMS_FILE),
        "route_graph_file": str(route_graph_file) if route_graph_file else None,
        "route_graph_source": route_graph_source,
    })


@app.route("/route-graph", methods=["GET"])
def route_graph():
    try:
        graph_file, graph_source = resolve_route_graph_file()
        nodes = load_route_graph_nodes(graph_file)
    except Exception as exc:
        return jsonify({
            "ok": False,
            "error": f"Failed to load route graph: {exc}",
            "nav2_params_file": str(NAV2_PARAMS_FILE),
        }), 500

    return jsonify({
        "ok": True,
        "route_graph_file": str(graph_file),
        "route_graph_source": graph_source,
        "nodes": nodes,
    })


@app.route("/route-node-command", methods=["POST"])
def route_node_command():
    payload = request.get_json(silent=True) or {}
    node_id = payload.get("node_id")

    try:
        node_id = int(node_id)
    except (TypeError, ValueError):
        return jsonify({
            "ok": False,
            "error": "Invalid route node ID.",
        }), 400

    try:
        graph_file, graph_source = resolve_route_graph_file()
        nodes_by_id = {node["id"]: node for node in load_route_graph_nodes(graph_file)}
    except Exception as exc:
        return jsonify({
            "ok": False,
            "error": f"Failed to load the active Nav2 route graph: {exc}",
        }), 500

    node = nodes_by_id.get(node_id)
    if node is None:
        return jsonify({
            "ok": False,
            "error": f"Route node {node_id} was not found in the graph.",
            "route_graph_file": str(graph_file),
            "route_graph_source": graph_source,
        }), 404

    accepted, action_response = send_navigation_goal_to_ros(node)

    if accepted:
        status = 200
    elif action_response.get("state") == "server_unavailable":
        status = 503
    elif action_response.get("state") == "rejected":
        status = 409
    elif action_response.get("state") == "timeout":
        status = 504
    else:
        status = 500

    return jsonify({
        "ok": accepted,
        "text": f"go to node {node_id}",
        "node": node,
        "accepted": accepted,
        "action": NAVIGATE_TO_POSE_ACTION,
        "route_graph_file": str(graph_file),
        "route_graph_source": graph_source,
        "goal_id": action_response.get("goal_id"),
        "state": action_response.get("state"),
        "error": action_response.get("error"),
    }), status


@app.route("/transcribe", methods=["POST"])
def transcribe():
    audio = request.files.get("audio")

    if audio is None:
        return jsonify({
            "ok": False,
            "error": "No audio received."
        }), 400

    timestamp = int(time.time() * 1000)

    webm_path = UPLOAD_DIR / f"cmd_{timestamp}.webm"
    wav_path = UPLOAD_DIR / f"cmd_{timestamp}.wav"

    try:
        audio.save(webm_path)

        print("\n==============================")
        print(f"Received audio: {webm_path}")

        if not convert_audio_to_wav(webm_path, wav_path):
            return jsonify({
                "ok": False,
                "error": "Audio conversion failed."
            }), 500

        print(f"Converted WAV: {wav_path}")
        text = run_whisper(wav_path)

        print("\n===== WHISPER TEXT =====")
        print(text)
        print("========================")

        if not text:
            return jsonify({
                "ok": False,
                "text": "",
                "error": "Whisper returned empty text."
            }), 500

        published = publish_voice_text_to_ros(text)
        return jsonify({
            "ok": published,
            "text": text,
            "published_to_ros": published,
            "topic": VOICE_TOPIC,
        })
    finally:
        webm_path.unlink(missing_ok=True)
        wav_path.unlink(missing_ok=True)


@app.errorhandler(413)
def upload_too_large(_error):
    limit_mb = app.config["MAX_CONTENT_LENGTH"] / (1024 * 1024)
    return jsonify({
        "ok": False,
        "error": f"Audio upload exceeds the {limit_mb:g} MiB limit.",
    }), 413


@app.route("/cancel", methods=["POST"])
def cancel_navigation():
    """Cancel Nav2 directly and hold zero velocity until cancellation is confirmed."""
    response = run_nav_runtime_helper(
        [
            "cancel",
            "--action", NAVIGATE_TO_POSE_ACTION,
            "--cmd-vel-topic", CMD_VEL_TOPIC,
            "--timeout", "8",
        ],
        timeout=11,
    )
    status = 200 if response.get("ok") and response.get("confirmed") else 503
    response.update({
        "text": "stop robot",
        "cmd_vel_topic": CMD_VEL_TOPIC,
        "action": NAVIGATE_TO_POSE_ACTION,
    })
    return jsonify(response), status


def robot_task_response(task_id, ok, message, pid, status):
    payload = robot_task_status(task_id)
    payload.update({
        "ok": ok,
        "message": message,
        "pid": pid,
    })
    return jsonify(payload), status


def run_robot_task_route(task_id):
    ok, message, pid = launch_robot_task(task_id)

    status = 200 if ok else 409
    if not ok and message.startswith("Missing"):
        status = 500
    if not ok and message.startswith("Unknown"):
        status = 404

    return robot_task_response(task_id, ok, message, pid, status)


def stop_robot_task_route(task_id):
    ok, message, pid = stop_robot_task(task_id)
    status = 200 if ok else 409
    if not ok and message.startswith("Unknown"):
        status = 404

    return robot_task_response(task_id, ok, message, pid, status)


def robot_task_status_route(task_id):
    status = robot_task_status(task_id)
    return jsonify(status), 200 if status["ok"] else 404


@app.route("/run-navigation", methods=["POST"])
def run_navigation():
    return run_robot_task_route("navigation")


@app.route("/stop-navigation", methods=["POST"])
def stop_navigation():
    return stop_robot_task_route("navigation")


@app.route("/navigation-status", methods=["GET"])
def navigation_status():
    return robot_task_status_route("navigation")


@app.route("/run-sim", methods=["POST"])
def run_sim_legacy():
    return run_navigation()


@app.route("/stop-sim", methods=["POST"])
def stop_sim_legacy():
    return stop_navigation()


@app.route("/sim-status", methods=["GET"])
def sim_status_legacy():
    return navigation_status()


# =========================
# Main
# =========================

if __name__ == "__main__":
    check_required_files()

    app.run(
        host="0.0.0.0",
        port=8443,
        ssl_context=(str(CERT_FILE), str(KEY_FILE)),
    )
