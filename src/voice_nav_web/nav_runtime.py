#!/usr/bin/env python3
"""Inspect Nav2 readiness or cancel navigation while holding a zero-velocity lock."""

import argparse
import json
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_action_status_default


ACTIVE_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def emit(payload, code=0):
    print(json.dumps(payload, separators=(",", ":")), flush=True)
    return code


def parse_args():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    status = subparsers.add_parser("status")
    status.add_argument("--action", default="/navigate_to_pose")
    status.add_argument("--localization-topic", default="/pose_stamped")
    status.add_argument("--timeout", type=float, default=0.35)

    cancel = subparsers.add_parser("cancel")
    cancel.add_argument("--action", default="/navigate_to_pose")
    cancel.add_argument("--cmd-vel-topic", default="/cmd_vel_teleop")
    cancel.add_argument("--timeout", type=float, default=8.0)
    cancel.add_argument("--lock-rate", type=float, default=20.0)
    return parser.parse_args()


def lifecycle_state(node, node_name, timeout):
    client = node.create_client(GetState, f"/{node_name}/get_state")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return "unavailable"
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            return "unavailable"
        return future.result().current_state.label
    finally:
        node.destroy_client(client)


def runtime_status(node, args):
    action_client = ActionClient(node, NavigateToPose, args.action)
    action_ready = action_client.wait_for_server(timeout_sec=args.timeout)
    node_names = {name for name, _ in node.get_node_names_and_namespaces()}
    lifecycle = {
        name: lifecycle_state(node, name, args.timeout)
        for name in ("bt_navigator", "controller_server", "planner_server", "route_server")
    }
    localization_node = "fast_lio_localization_sc_qn_node" in node_names
    received_localization = False

    def localization_callback(_message):
        nonlocal received_localization
        received_localization = True

    localization_subscription = node.create_subscription(
        PoseStamped, args.localization_topic, localization_callback, 10)
    localization_deadline = time.monotonic() + args.timeout
    while localization_node and not received_localization and time.monotonic() < localization_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(localization_subscription)
    localization_ready = localization_node and received_localization
    nav_lifecycle_ready = all(state == "active" for state in lifecycle.values())
    failed = any(state in ("finalized", "errorprocessing") for state in lifecycle.values())
    ready = action_ready and nav_lifecycle_ready and localization_ready
    early_stack_nodes = {"livox_lidar_publisher", "laser_mapping"}
    detected = action_ready or localization_node or bool(node_names & early_stack_nodes) or any(
        state != "unavailable" for state in lifecycle.values()
    )

    return {
        "running": detected,
        "ready": ready,
        "localization_ready": localization_ready,
        "action_ready": action_ready,
        "lifecycle": lifecycle,
        "state": "ready" if ready else ("failed" if failed else ("starting" if detected else "stopped")),
        "localization_topic": args.localization_topic,
    }


def zero_twist(node, publisher):
    message = TwistStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(message)


def cancel_navigation(node, args):
    cancel_service = f"{args.action.rstrip('/')}/_action/cancel_goal"
    status_topic = f"{args.action.rstrip('/')}/_action/status"
    cancel_client = node.create_client(CancelGoal, cancel_service)
    publisher = node.create_publisher(TwistStamped, args.cmd_vel_topic, 10)
    latest_active_ids = set()
    received_status = False

    def status_callback(message):
        nonlocal received_status, latest_active_ids
        received_status = True
        latest_active_ids = {
            bytes(entry.goal_info.goal_id.uuid).hex()
            for entry in message.status_list
            if entry.status in ACTIVE_STATUSES
        }

    subscription = node.create_subscription(
        GoalStatusArray,
        status_topic,
        status_callback,
        qos_profile_action_status_default,
    )
    started = time.monotonic()
    deadline = started + args.timeout

    try:
        while time.monotonic() < deadline and not cancel_client.wait_for_service(timeout_sec=0.05):
            zero_twist(node, publisher)
            rclpy.spin_once(node, timeout_sec=0.01)

        if not cancel_client.service_is_ready():
            return {
                "ok": False,
                "confirmed": False,
                "state": "action_unavailable",
                "error": f"Cancellation service {cancel_service} is unavailable.",
            }

        request = CancelGoal.Request()
        future = cancel_client.call_async(request)
        period = 1.0 / max(args.lock_rate, 1.0)
        while time.monotonic() < deadline and not future.done():
            zero_twist(node, publisher)
            rclpy.spin_once(node, timeout_sec=period)

        if not future.done() or future.result() is None:
            return {
                "ok": False,
                "confirmed": False,
                "state": "cancel_timeout",
                "error": "Nav2 did not answer the cancellation request.",
            }

        response = future.result()
        if response.return_code not in (
            CancelGoal.Response.ERROR_NONE,
            CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
        ):
            return {
                "ok": False,
                "confirmed": False,
                "state": "cancel_rejected",
                "return_code": int(response.return_code),
                "error": "Nav2 rejected the cancellation request.",
            }

        target_ids = {bytes(info.goal_id.uuid).hex() for info in response.goals_canceling}
        if not target_ids:
            zero_twist(node, publisher)
            return {
                "ok": True,
                "confirmed": True,
                "state": "no_active_goal",
                "canceled_goal_ids": [],
            }

        while time.monotonic() < deadline:
            zero_twist(node, publisher)
            rclpy.spin_once(node, timeout_sec=period)
            if received_status and target_ids.isdisjoint(latest_active_ids):
                return {
                    "ok": True,
                    "confirmed": True,
                    "state": "canceled",
                    "canceled_goal_ids": sorted(target_ids),
                }

        return {
            "ok": False,
            "confirmed": False,
            "state": "confirmation_timeout",
            "canceled_goal_ids": sorted(target_ids),
            "error": "Velocity lock held, but Nav2 cancellation was not confirmed before timeout.",
        }
    finally:
        zero_twist(node, publisher)
        node.destroy_subscription(subscription)
        node.destroy_publisher(publisher)
        node.destroy_client(cancel_client)


def main():
    args = parse_args()
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node(f"voice_nav_runtime_{args.command}_{os.getpid()}")
        if args.command == "status":
            return emit(runtime_status(node, args))
        result = cancel_navigation(node, args)
        return emit(result, 0 if result["ok"] else 2)
    except Exception as error:
        return emit({"ok": False, "state": "helper_error", "error": str(error)}, 3)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
