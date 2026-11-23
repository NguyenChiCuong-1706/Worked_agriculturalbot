#include <memory>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <functional>
#include <chrono>

#include <sys/select.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class RouteMissionClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

  RouteMissionClient()
  : Node("route_mission_client")
  {
    action_client_ =
      rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose"
      );

    this->declare_parameter<std::string>("behavior_tree", "");
    behavior_tree_ = this->get_parameter("behavior_tree").as_string();

    init_predefined_nodes();

    start_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RouteMissionClient::wait_for_nav2, this)
    );

    input_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RouteMissionClient::check_terminal_input, this)
    );

    RCLCPP_INFO(this->get_logger(), "Route mission client started.");
    RCLCPP_INFO(this->get_logger(), "Type node IDs, for example: 1 3 5");
    RCLCPP_INFO(this->get_logger(), "Other commands: list IDs, cancel, q");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr input_timer_;

  std::map<int, geometry_msgs::msg::PoseStamped> node_goals_;

  std::vector<int> mission_ids_;
  std::size_t current_mission_index_ = 0;

  bool nav2_ready_ = false;
  bool navigating_ = false;
  bool mission_active_ = false;

  int current_active_node_id_ = -1;

  std::string behavior_tree_;

  geometry_msgs::msg::PoseStamped make_goal(
    double x,
    double y,
    double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;

    pose.header.frame_id = "map";
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

  void init_predefined_nodes()
  {
    // Temporary example node IDs.
    // You can adjust these x, y, yaw values later according to your real route graph.
    node_goals_[1] = make_goal(0.0, 0.0, 0.0);
    node_goals_[2] = make_goal(0.0, 2.5, 0.0);
    node_goals_[3] = make_goal(3.0, 2.5, 0.0);
    node_goals_[4] = make_goal(3.01001, 3.57769, 0);
    // node_goals_[5] = make_goal(5.0, 0.5, 3.14);
    // node_goals_[6] = make_goal(6.0, 0.0, 3.14);
  }

  void wait_for_nav2()
  {
    if (nav2_ready_) {
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /navigate_to_pose action server...");
      return;
    }

    nav2_ready_ = true;
    start_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Nav2 /navigate_to_pose action server is ready.");
    print_available_nodes();
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

    std::string command = to_lowercase(line);

    if (command == "q" ||
        command == "quit" ||
        command == "exit")
    {
      RCLCPP_INFO(this->get_logger(), "Shutdown requested.");
      rclcpp::shutdown();
      return;
    }

    if (command == "list" ||
        command == "list ids" ||
        command == "list id" ||
        command == "ids")
    {
      print_available_nodes();
      return;
    }

    if (command == "cancel") {
      cancel_current_goal();
      return;
    }

    std::vector<int> requested_ids;

    if (!parse_node_ids(line, requested_ids)) {
      RCLCPP_WARN(this->get_logger(), "Invalid input. Example: 1 3 5");
      RCLCPP_WARN(this->get_logger(), "Commands: list IDs, cancel, q");
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
        "A mission is already running. Type 'cancel' first if you want to stop it."
      );
      return;
    }

    mission_ids_ = requested_ids;
    current_mission_index_ = 0;
    mission_active_ = true;

    RCLCPP_INFO(this->get_logger(), "New mission received.");
    print_current_mission();

    send_next_goal();
  }

  bool parse_node_ids(
    const std::string & line,
    std::vector<int> & ids)
  {
    std::stringstream ss(line);
    std::string token;

    while (ss >> token) {
      try {
        std::size_t processed_chars = 0;
        int id = std::stoi(token, &processed_chars);

        if (processed_chars != token.size()) {
          return false;
        }

        ids.push_back(id);
      } catch (...) {
        return false;
      }
    }

    return !ids.empty();
  }

  bool validate_node_ids(const std::vector<int> & ids)
  {
    for (const auto id : ids) {
      if (node_goals_.find(id) == node_goals_.end()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Node ID %d does not exist in predefined node list.",
          id
        );
        return false;
      }
    }

    return true;
  }

  void print_available_nodes()
  {
    RCLCPP_INFO(this->get_logger(), "Available predefined node IDs:");

    for (const auto & item : node_goals_) {
      int id = item.first;
      const auto & pose = item.second.pose;

      RCLCPP_INFO(
        this->get_logger(),
        "  ID %d -> x=%.2f, y=%.2f, yaw is stored in quaternion z=%.3f, w=%.3f",
        id,
        pose.position.x,
        pose.position.y,
        pose.orientation.z,
        pose.orientation.w
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
        "Node ID %d disappeared from predefined list. Mission stopped.",
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

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

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

  void goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
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

  void result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result)
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
