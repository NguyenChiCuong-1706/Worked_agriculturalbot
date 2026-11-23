#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <functional>
#include <chrono>
#include <fstream>
#include <future>

#include <nlohmann/json.hpp>

#include <sys/select.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"

class RouteMissionClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  RouteMissionClient()
  : Node("route_mission_client")
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose"
    );

    this->declare_parameter<std::string>("behavior_tree", "");
    behavior_tree_ = this->get_parameter("behavior_tree").as_string();

    this->declare_parameter<std::string>("route_server_node", "/route_server");
    route_server_node_ = this->get_parameter("route_server_node").as_string();
    route_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, route_server_node_);

    this->declare_parameter<int>("home_node_id", 1);
    this->declare_parameter<int>("charging_node_id", 4);
    home_node_id_ = this->get_parameter("home_node_id").as_int();
    charging_node_id_ = this->get_parameter("charging_node_id").as_int();

    start_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RouteMissionClient::wait_for_nav2, this)
    );

    input_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RouteMissionClient::check_terminal_input, this)
    );

    voice_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/voice_command_text",
      10,
      std::bind(&RouteMissionClient::voice_command_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Route mission client started.");
    RCLCPP_INFO(this->get_logger(), "Terminal input examples: 1 3 5, node one then node three, list IDs, list aliases, cancel, q");
    RCLCPP_INFO(this->get_logger(), "Voice input topic: /voice_command_text [std_msgs/msg/String]");
    RCLCPP_INFO(this->get_logger(), "Voice examples: go to the stair, go to lab, go to row number one, stop robot");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> route_params_client_;

  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr input_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_command_sub_;

  std::map<int, geometry_msgs::msg::PoseStamped> node_goals_;
  std::unordered_map<std::string, int> node_aliases_;

  std::vector<int> mission_ids_;
  std::size_t current_mission_index_ = 0;

  bool nav2_ready_ = false;
  bool route_graph_loaded_ = false;
  bool graph_request_pending_ = false;
  bool navigating_ = false;
  bool mission_active_ = false;

  int current_active_node_id_ = -1;
  int home_node_id_ = 1;
  int charging_node_id_ = 4;

  std::string behavior_tree_;
  std::string route_server_node_;
  std::string route_graph_file_;

  geometry_msgs::msg::PoseStamped make_goal(
    double x, double y, double yaw, const std::string & frame)
  {
    geometry_msgs::msg::PoseStamped pose;

    pose.header.frame_id = frame;
    pose.header.stamp = this->now();

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    return pose;
  }

  bool load_route_graph(const std::string & graph_filepath)
  {
    try {
      std::ifstream graph_stream(graph_filepath);
      if (!graph_stream.is_open()) {
        RCLCPP_ERROR(
          this->get_logger(), "Could not open active route graph: %s", graph_filepath.c_str());
        return false;
      }

      nlohmann::json graph;
      graph_stream >> graph;

      std::map<int, geometry_msgs::msg::PoseStamped> loaded_goals;
      for (const auto & feature : graph.value("features", nlohmann::json::array())) {
        const auto geometry = feature.value("geometry", nlohmann::json::object());
        if (geometry.value("type", "") != "Point") {
          continue;
        }

        const auto coordinates = geometry.value("coordinates", nlohmann::json::array());
        const auto properties = feature.value("properties", nlohmann::json::object());
        if (coordinates.size() < 2 || !properties.contains("id")) {
          continue;
        }

        const int id = properties.at("id").get<int>();
        const double x = coordinates.at(0).get<double>();
        const double y = coordinates.at(1).get<double>();
        const double yaw = properties.value("yaw", 0.0);
        const std::string frame = properties.value("frame", "map");

        if (!loaded_goals.emplace(id, make_goal(x, y, yaw, frame)).second) {
          RCLCPP_ERROR(this->get_logger(), "Duplicate node ID %d in %s", id, graph_filepath.c_str());
          return false;
        }
      }

      if (loaded_goals.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No Point nodes found in %s", graph_filepath.c_str());
        return false;
      }

      node_goals_ = std::move(loaded_goals);
      route_graph_file_ = graph_filepath;
      RCLCPP_INFO(
        this->get_logger(), "Loaded %zu destinations from active route graph: %s",
        node_goals_.size(), route_graph_file_.c_str());
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to load route graph '%s': %s",
        graph_filepath.c_str(), error.what());
      return false;
    }
  }

  std::string trim(const std::string & text)
  {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }

    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
  }

  std::string to_lowercase(std::string text)
  {
    std::transform(
      text.begin(),
      text.end(),
      text.begin(),
      [](unsigned char c) {
        return std::tolower(c);
      }
    );

    return text;
  }

  std::string normalize_command_text(const std::string & text)
  {
    std::string lower = to_lowercase(text);
    std::string cleaned;
    cleaned.reserve(lower.size());

    for (const unsigned char c : lower) {
      if (std::isalnum(c)) {
        cleaned.push_back(static_cast<char>(c));
      } else {
        cleaned.push_back(' ');
      }
    }

    std::stringstream ss(cleaned);
    std::string token;
    std::string normalized;

    while (ss >> token) {
      if (!normalized.empty()) {
        normalized.push_back(' ');
      }
      normalized += token;
    }

    return normalized;
  }

  bool contains_word(const std::string & text, const std::string & word)
  {
    std::stringstream ss(text);
    std::string token;

    while (ss >> token) {
      if (token == word) {
        return true;
      }
    }

    return false;
  }

  bool contains_phrase_with_word_boundary(
    const std::string & normalized_text,
    const std::string & normalized_phrase,
    std::size_t & position) const
  {
    const std::string padded_text = " " + normalized_text + " ";
    const std::string padded_phrase = " " + normalized_phrase + " ";

    const auto found = padded_text.find(padded_phrase);
    if (found == std::string::npos) {
      return false;
    }

    // Convert from padded-text position to original normalized text position.
    position = found;
    return true;
  }

  bool token_to_node_id(const std::string & token, int & id)
  {
    static const std::map<std::string, int> word_to_id = {
      {"zero", 0},
      {"one", 1}, {"first", 1}, {"won", 1},
      {"two", 2}, {"second", 2}, {"to", 2}, {"too", 2},
      {"three", 3}, {"third", 3}, {"tree", 3},
      {"four", 4}, {"fourth", 4}, {"for", 4},
      {"five", 5}, {"fifth", 5},
      {"six", 6}, {"sixth", 6},
      {"seven", 7}, {"seventh", 7},
      {"eight", 8}, {"eighth", 8},
      {"nine", 9}, {"ninth", 9},
      {"ten", 10}, {"tenth", 10}
    };

    const auto word_it = word_to_id.find(token);
    if (word_it != word_to_id.end()) {
      id = word_it->second;
      return true;
    }

    // Accept pure numbers: "1", "23".
    if (!token.empty() && std::all_of(token.begin(), token.end(), [](unsigned char c) {
      return std::isdigit(c);
    })) {
      try {
        id = std::stoi(token);
        return true;
      } catch (...) {
        return false;
      }
    }

    // Accept mixed tokens from STT or typed input: "node3", "row4".
    std::string digits;
    for (const unsigned char c : token) {
      if (std::isdigit(c)) {
        digits.push_back(static_cast<char>(c));
      }
    }

    if (!digits.empty()) {
      try {
        id = std::stoi(digits);
        return true;
      } catch (...) {
        return false;
      }
    }

    return false;
  }

  void load_node_aliases()
  {
    // Format: "alias phrase:node_id"
    // You should normally override this from config/voice_node_aliases.yaml.
    this->declare_parameter<std::vector<std::string>>(
      "node_aliases",
      std::vector<std::string>{
        "node one:1",
        "row one:1",
        "row number one:1",
        "stair:1",
        "stairs:1",
        "the stair:1",

        "node two:2",
        "row two:2",
        "row number two:2",
        "lab:2",
        "laboratory:2",
        "the lab:2",

        "node three:3",
        "office:3",
        "main office:3",

        "node four:4",
        "charging station:4",
        "charger:4",
        "charge station:4"
      }
    );

    const auto alias_entries = this->get_parameter("node_aliases").as_string_array();
    node_aliases_.clear();

    for (const auto & entry : alias_entries) {
      const auto colon_pos = entry.rfind(":");
      if (colon_pos == std::string::npos) {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid node alias entry: '%s'. Expected format: alias:id",
          entry.c_str()
        );
        continue;
      }

      std::string alias = trim(entry.substr(0, colon_pos));
      std::string id_text = trim(entry.substr(colon_pos + 1));
      alias = normalize_command_text(alias);

      if (alias.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty alias in entry: '%s'", entry.c_str());
        continue;
      }

      int node_id = -1;
      try {
        node_id = std::stoi(id_text);
      } catch (...) {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid node ID in alias entry: '%s'",
          entry.c_str()
        );
        continue;
      }

      if (node_goals_.find(node_id) == node_goals_.end()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Alias '%s' points to node ID %d, but this node does not exist in node_goals_.",
          alias.c_str(),
          node_id
        );
        continue;
      }

      node_aliases_[alias] = node_id;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu voice node aliases.", node_aliases_.size());
  }

  bool parse_mission_ids_from_text(
    const std::string & text,
    std::vector<int> & ids)
  {
    ids.clear();

    const std::string normalized = normalize_command_text(text);
    if (normalized.empty()) {
      return false;
    }

    // 1. Configurable alias matching from YAML.
    // Examples:
    //   "go to the stair" -> alias "stair" -> node ID 1
    //   "go to lab"       -> alias "lab"   -> node ID 2
    struct AliasMatch
    {
      std::size_t position;
      std::size_t length;
      int node_id;
      std::string alias;
    };

    std::vector<AliasMatch> matches;

    for (const auto & item : node_aliases_) {
      const std::string & alias = item.first;
      const int node_id = item.second;

      std::size_t pos = std::string::npos;
      if (contains_phrase_with_word_boundary(normalized, alias, pos)) {
        matches.push_back(AliasMatch{pos, alias.size(), node_id, alias});
      }
    }

    if (!matches.empty()) {
      std::sort(
        matches.begin(),
        matches.end(),
        [](const AliasMatch & a, const AliasMatch & b) {
          if (a.position != b.position) {
            return a.position < b.position;
          }
          return a.length > b.length;
        }
      );

      std::size_t last_accepted_end = 0;
      bool has_accepted_match = false;

      for (const auto & match : matches) {
        const std::size_t match_end = match.position + match.length;

        // If aliases overlap at the same text location, keep the longest one only.
        if (has_accepted_match && match.position < last_accepted_end) {
          continue;
        }

        if (ids.empty() || ids.back() != match.node_id) {
          ids.push_back(match.node_id);
          RCLCPP_INFO(
            this->get_logger(),
            "Voice alias matched: '%s' -> node ID %d",
            match.alias.c_str(),
            match.node_id
          );
        }

        last_accepted_end = match_end;
        has_accepted_match = true;
      }

      return !ids.empty();
    }

    // 2. Semantic fallback for common fixed concepts.
    if (contains_word(normalized, "home")) {
      ids.push_back(home_node_id_);
      return true;
    }

    if (contains_word(normalized, "charging") || contains_word(normalized, "charger")) {
      ids.push_back(charging_node_id_);
      return true;
    }

    // 3. Numeric fallback.
    // Examples:
    //   "1 3 5"
    //   "go to node one"
    //   "node1 then node3"
    std::stringstream ss(normalized);
    std::string token;

    while (ss >> token) {
      int id = -1;
      if (token_to_node_id(token, id)) {
        if (id > 0) {
          if (ids.empty() || ids.back() != id) {
            ids.push_back(id);
          }
        }
      }
    }

    return !ids.empty();
  }

  bool is_cancel_command(const std::string & text)
  {
    const std::string normalized = normalize_command_text(text);

    return normalized == "cancel" ||
           normalized == "stop" ||
           normalized == "stop robot" ||
           normalized == "cancel navigation" ||
           normalized == "cancel mission" ||
           (contains_word(normalized, "stop") && contains_word(normalized, "robot")) ||
           (contains_word(normalized, "cancel") && contains_word(normalized, "navigation"));
  }

  bool is_list_command(const std::string & text)
  {
    const std::string normalized = normalize_command_text(text);

    return normalized == "list" ||
           normalized == "list ids" ||
           normalized == "list id" ||
           normalized == "ids" ||
           normalized == "list nodes" ||
           normalized == "show nodes";
  }

  bool is_list_alias_command(const std::string & text)
  {
    const std::string normalized = normalize_command_text(text);

    return normalized == "list aliases" ||
           normalized == "show aliases" ||
           normalized == "list voice aliases" ||
           normalized == "show voice aliases";
  }

  bool is_quit_command(const std::string & text)
  {
    const std::string normalized = normalize_command_text(text);

    return normalized == "q" ||
           normalized == "quit" ||
           normalized == "exit";
  }

  bool terminal_has_input()
  {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int result = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
    return result > 0 && FD_ISSET(STDIN_FILENO, &readfds);
  }

  void check_terminal_input()
  {
    if (!terminal_has_input()) {
      return;
    }

    std::string line;
    std::getline(std::cin, line);
    line = trim(line);

    if (line.empty()) {
      return;
    }

    if (is_quit_command(line)) {
      RCLCPP_INFO(this->get_logger(), "Shutdown requested.");
      rclcpp::shutdown();
      return;
    }

    if (is_list_command(line)) {
      print_available_nodes();
      return;
    }

    if (is_list_alias_command(line)) {
      print_loaded_aliases();
      return;
    }

    if (is_cancel_command(line)) {
      cancel_current_goal();
      return;
    }

    std::vector<int> requested_ids;

    if (!parse_mission_ids_from_text(line, requested_ids)) {
      RCLCPP_WARN(this->get_logger(), "Invalid input. Examples: 1 3 5, node one, go to lab, go to the stair");
      RCLCPP_WARN(this->get_logger(), "Commands: list IDs, list aliases, cancel, q");
      return;
    }

    start_mission_from_ids(requested_ids, "terminal");
  }

  void voice_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string raw_text = trim(msg->data);

    if (raw_text.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty voice command.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Voice text received: '%s'", raw_text.c_str());

    if (is_list_command(raw_text)) {
      print_available_nodes();
      return;
    }

    if (is_list_alias_command(raw_text)) {
      print_loaded_aliases();
      return;
    }

    if (is_cancel_command(raw_text)) {
      RCLCPP_WARN(this->get_logger(), "Voice command requested mission cancel.");
      cancel_current_goal();
      return;
    }

    std::vector<int> requested_ids;

    if (!parse_mission_ids_from_text(raw_text, requested_ids)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not extract a valid node ID from voice command: '%s'. Try: go to lab, go to the stair, go to node one.",
        raw_text.c_str()
      );
      return;
    }

    start_mission_from_ids(requested_ids, "voice");
  }

  void wait_for_nav2()
  {
    if (!nav2_ready_) {
      if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /navigate_to_pose action server...");
      } else {
        nav2_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Nav2 /navigate_to_pose action server is ready.");
      }
    }

    if (!route_graph_loaded_ && !graph_request_pending_) {
      if (!route_params_client_->service_is_ready()) {
        RCLCPP_INFO(
          this->get_logger(), "Waiting for %s parameter service...", route_server_node_.c_str());
      } else {
        graph_request_pending_ = true;
        route_params_client_->get_parameters(
          {"graph_filepath"},
          [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
            graph_request_pending_ = false;
            try {
              const auto parameters = future.get();
              if (parameters.empty() || parameters.front().get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                RCLCPP_ERROR(
                  this->get_logger(), "%s did not return a string graph_filepath parameter.",
                  route_server_node_.c_str());
                return;
              }

              const auto graph_filepath = parameters.front().as_string();
              if (graph_filepath.empty() || !load_route_graph(graph_filepath)) {
                return;
              }

              route_graph_loaded_ = true;
              load_node_aliases();
              finish_initialization_if_ready();
            } catch (const std::exception & error) {
              RCLCPP_ERROR(
                this->get_logger(), "Failed to read active route graph parameter: %s", error.what());
            }
          });
      }
    }

    finish_initialization_if_ready();
  }

  void finish_initialization_if_ready()
  {
    if (!nav2_ready_ || !route_graph_loaded_) {
      return;
    }

    start_timer_->cancel();
    RCLCPP_INFO(
      this->get_logger(), "Mission client is ready using route graph: %s", route_graph_file_.c_str());
    print_available_nodes();
    print_loaded_aliases();
  }

  void start_mission_from_ids(
    const std::vector<int> & requested_ids,
    const std::string & source)
  {
    if (!route_graph_loaded_) {
      RCLCPP_WARN(this->get_logger(), "The active Nav2 route graph is not loaded yet.");
      return;
    }

    if (!validate_node_ids(requested_ids)) {
      print_available_nodes();
      return;
    }

    if (!nav2_ready_) {
      RCLCPP_WARN(this->get_logger(), "Nav2 is not ready yet.");
      return;
    }

    if (navigating_ || mission_active_) {
      RCLCPP_WARN(
        this->get_logger(),
        "A mission is already running. Send 'cancel' or 'stop robot' first if you want to stop it."
      );
      return;
    }

    mission_ids_ = requested_ids;
    current_mission_index_ = 0;
    mission_active_ = true;

    RCLCPP_INFO(this->get_logger(), "New mission received from %s.", source.c_str());
    print_current_mission();

    send_next_goal();
  }

  bool validate_node_ids(const std::vector<int> & ids)
  {
    for (const auto id : ids) {
      if (node_goals_.find(id) == node_goals_.end()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Node ID %d does not exist in the active route graph.",
          id
        );
        return false;
      }
    }

    return true;
  }

  void print_available_nodes()
  {
    RCLCPP_INFO(this->get_logger(), "Available node IDs from %s:", route_graph_file_.c_str());

    for (const auto & item : node_goals_) {
      int id = item.first;
      const auto & pose = item.second.pose;

      RCLCPP_INFO(
        this->get_logger(),
        "  ID %d -> x=%.2f, y=%.2f, quaternion z=%.3f, w=%.3f",
        id,
        pose.position.x,
        pose.position.y,
        pose.orientation.z,
        pose.orientation.w
      );
    }
  }

  void print_loaded_aliases()
  {
    RCLCPP_INFO(this->get_logger(), "Loaded voice aliases:");

    std::vector<std::pair<std::string, int>> sorted_aliases(
      node_aliases_.begin(),
      node_aliases_.end()
    );

    std::sort(
      sorted_aliases.begin(),
      sorted_aliases.end(),
      [](const std::pair<std::string, int> & a, const std::pair<std::string, int> & b) {
        if (a.second != b.second) {
          return a.second < b.second;
        }
        return a.first < b.first;
      }
    );

    for (const auto & item : sorted_aliases) {
      RCLCPP_INFO(
        this->get_logger(),
        "  '%s' -> node ID %d",
        item.first.c_str(),
        item.second
      );
    }
  }

  void print_current_mission()
  {
    std::stringstream ss;
    ss << "Mission IDs: ";

    for (std::size_t i = 0; i < mission_ids_.size(); i++) {
      ss << mission_ids_[i];

      if (i + 1 < mission_ids_.size()) {
        ss << " -> ";
      }
    }

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  void send_next_goal()
  {
    if (!mission_active_) {
      return;
    }

    if (current_mission_index_ >= mission_ids_.size()) {
      RCLCPP_INFO(this->get_logger(), "All mission goals completed.");
      mission_active_ = false;
      navigating_ = false;
      current_active_node_id_ = -1;
      return;
    }

    int node_id = mission_ids_[current_mission_index_];

    if (node_goals_.find(node_id) == node_goals_.end()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Node ID %d disappeared from the active route graph. Mission stopped.",
        node_id
      );
      mission_active_ = false;
      navigating_ = false;
      current_active_node_id_ = -1;
      return;
    }

    auto goal_msg = NavigateToPose::Goal();

    auto pose = node_goals_.at(node_id);
    pose.header.stamp = this->now();

    goal_msg.pose = pose;
    goal_msg.behavior_tree = behavior_tree_;

    navigating_ = true;
    current_active_node_id_ = node_id;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending mission goal %zu / %zu: node ID %d, x=%.2f, y=%.2f",
      current_mission_index_ + 1,
      mission_ids_.size(),
      node_id,
      goal_msg.pose.pose.position.x,
      goal_msg.pose.pose.position.y
    );

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(
        &RouteMissionClient::goal_response_callback,
        this,
        std::placeholders::_1
      );

    send_goal_options.feedback_callback =
      std::bind(
        &RouteMissionClient::feedback_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      );

    send_goal_options.result_callback =
      std::bind(
        &RouteMissionClient::result_callback,
        this,
        std::placeholders::_1
      );

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2.");
      navigating_ = false;
      mission_active_ = false;
      current_active_node_id_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2.");
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Distance remaining: %.2f m",
      feedback->distance_remaining
    );
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    int node_id = current_active_node_id_;
    navigating_ = false;

    auto nav_result = result.result;

    if (!nav_result) {
      RCLCPP_ERROR(this->get_logger(), "Navigation result is null.");
      mission_active_ = false;
      current_active_node_id_ = -1;
      return;
    }

    if (!mission_active_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received result for node ID %d, but mission is no longer active.",
        node_id
      );
      current_active_node_id_ = -1;
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        nav_result->error_code == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Node ID %d reached successfully.",
        node_id
      );

      current_mission_index_++;
      current_active_node_id_ = -1;

      send_next_goal();
      return;
    }

    if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Navigation to node ID %d aborted. error_code=%u, error_msg=%s",
        node_id,
        nav_result->error_code,
        nav_result->error_msg.c_str()
      );
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(
        this->get_logger(),
        "Navigation to node ID %d canceled.",
        node_id
      );
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Navigation to node ID %d failed. error_code=%u, error_msg=%s",
        node_id,
        nav_result->error_code,
        nav_result->error_msg.c_str()
      );
    }

    mission_active_ = false;
    navigating_ = false;
    current_active_node_id_ = -1;
  }

  void cancel_current_goal()
  {
    if (!navigating_ && !mission_active_) {
      RCLCPP_WARN(this->get_logger(), "No active mission to cancel.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Canceling current mission...");

    action_client_->async_cancel_all_goals();

    navigating_ = false;
    mission_active_ = false;
    mission_ids_.clear();
    current_mission_index_ = 0;
    current_active_node_id_ = -1;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RouteMissionClient>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
