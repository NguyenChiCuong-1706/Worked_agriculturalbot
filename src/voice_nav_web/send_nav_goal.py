#!/usr/bin/env python3
"""Send one Nav2 NavigateToPose goal and report whether it was accepted."""

import argparse
import json
import math
import os
import sys

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def emit(payload, exit_code):
    print(json.dumps(payload, separators=(",", ":")), flush=True)
    return exit_code


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--action", default="/navigate_to_pose")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--server-timeout", type=float, default=5.0)
    parser.add_argument("--response-timeout", type=float, default=5.0)
    return parser.parse_args()


def main():
    args = parse_args()
    node = None
    action_client = None

    try:
        rclpy.init()
        node = rclpy.create_node(f"voice_nav_goal_sender_{os.getpid()}")
        action_client = ActionClient(node, NavigateToPose, args.action)

        if not action_client.wait_for_server(timeout_sec=args.server_timeout):
            return emit({
                "accepted": False,
                "state": "server_unavailable",
                "error": f"Nav2 action server {args.action} is not available.",
            }, 3)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = args.frame
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(args.yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(args.yaw * 0.5)

        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=args.response_timeout,
        )

        if not future.done():
            return emit({
                "accepted": False,
                "state": "timeout",
                "error": f"Nav2 did not answer the goal request within {args.response_timeout:g} seconds.",
            }, 4)

        goal_handle = future.result()
        if goal_handle is None:
            error = future.exception()
            return emit({
                "accepted": False,
                "state": "request_error",
                "error": f"Nav2 goal request failed: {error!r}",
            }, 5)

        if not goal_handle.accepted:
            return emit({
                "accepted": False,
                "state": "rejected",
                "error": "Nav2 rejected the navigation goal.",
            }, 2)

        goal_id = bytes(goal_handle.goal_id.uuid).hex()
        return emit({
            "accepted": True,
            "state": "accepted",
            "goal_id": goal_id,
            "action": args.action,
            "frame": args.frame,
            "x": args.x,
            "y": args.y,
            "yaw": args.yaw,
        }, 0)

    except Exception as exc:
        return emit({
            "accepted": False,
            "state": "helper_error",
            "error": f"Goal helper failed: {exc}",
        }, 6)
    finally:
        if action_client is not None:
            action_client.destroy()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
