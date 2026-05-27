import math
import sys
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped


FILE = "/home/ubuntu/saved_places.yaml"
WAIT_BETWEEN_PLACES = 3.0


class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__("multi_goal_navigator")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.current_pose = None

        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            amcl_qos
        )

    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def load_places(self):
        try:
            with open(FILE, "r") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Error reading file: {e}")
            return None

        if data is None:
            self.get_logger().error("saved_places.yaml is empty")
            return None

        return data

    def wait_for_current_pose(self, timeout_sec=15.0):
        self.get_logger().info("Waiting for current robot pose from /amcl_pose...")

        start_time = time.monotonic()

        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.monotonic() - start_time > timeout_sec:
                self.get_logger().warn(
                    "Could not get /amcl_pose. Optimized route cannot be calculated from current position."
                )
                return False

        self.get_logger().info(
            f"Current pose received: x={self.current_pose.position.x:.3f}, "
            f"y={self.current_pose.position.y:.3f}"
        )
        return True

    def build_goal(self, place_name, place_data):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = place_data.get("frame_id", "map")
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(place_data["x"])
        goal_msg.pose.pose.position.y = float(place_data["y"])
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = float(place_data["z"])
        goal_msg.pose.pose.orientation.w = float(place_data["w"])

        return goal_msg

    def go_to_place(self, place_name, place_data):
        self.get_logger().info("Waiting for Nav2 action server...")

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server /navigate_to_pose is not available")
            return False

        goal_msg = self.build_goal(place_name, place_data)

        self.get_logger().info(f"Sending robot to '{place_name}'...")
        send_future = self.client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"Failed to send goal for '{place_name}'")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for '{place_name}'")
            return False

        self.get_logger().info(f"Goal accepted for '{place_name}'. Waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error(f"No result returned for '{place_name}'")
            return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Successfully reached '{place_name}'")
            return True

        self.get_logger().error(
            f"Failed to reach '{place_name}'. Goal status={result.status}"
        )
        return False

    def distance_xy(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def optimize_route(self, requested_places, data):
        if len(requested_places) <= 1:
            return requested_places

        if not self.wait_for_current_pose(timeout_sec=15.0):
            self.get_logger().warn("Using original route order instead.")
            return requested_places

        remaining = requested_places[:]
        optimized = []

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        while remaining:
            nearest_place = None
            nearest_distance = None

            for place_name in remaining:
                place_data = data[place_name]
                place_x = float(place_data["x"])
                place_y = float(place_data["y"])

                distance = self.distance_xy(current_x, current_y, place_x, place_y)

                if nearest_distance is None or distance < nearest_distance:
                    nearest_distance = distance
                    nearest_place = place_name

            optimized.append(nearest_place)
            remaining.remove(nearest_place)

            current_x = float(data[nearest_place]["x"])
            current_y = float(data[nearest_place]["y"])

        self.get_logger().info(
            "Optimized route: " + " -> ".join(optimized)
        )

        return optimized


def parse_args(argv):
    optimize = False
    raw_places = []

    for arg in argv:
        if arg in ["--optimize", "-o"]:
            optimize = True
        else:
            raw_places.append(arg)

    places = []

    for arg in raw_places:
        parts = arg.split(",")
        for part in parts:
            place = part.strip()
            if place:
                places.append(place)

    return optimize, places


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 goto_place_adv.py <place_name>")
        print("  python3 goto_place_adv.py <place1> <place2> <place3>")
        print("  python3 goto_place_adv.py <place1>,<place2>,<place3>")
        print("  python3 goto_place_adv.py --optimize <place1> <place2> <place3>")
        print("  python3 goto_place_adv.py -o <place1>,<place2>,<place3>")
        sys.exit(1)

    optimize, requested_places = parse_args(sys.argv[1:])

    if len(requested_places) < 1:
        print("No destination was provided")
        sys.exit(1)

    rclpy.init()
    node = MultiGoalNavigator()

    try:
        data = node.load_places()
        if data is None:
            sys.exit(1)

        for place_name in requested_places:
            if place_name not in data:
                node.get_logger().error(f"Place '{place_name}' not found")
                sys.exit(1)

        if optimize:
            requested_places = node.optimize_route(requested_places, data)
        else:
            node.get_logger().info(
                "Using requested route order: " + " -> ".join(requested_places)
            )

        for index, place_name in enumerate(requested_places):
            place_data = data[place_name]

            success = node.go_to_place(place_name, place_data)

            if not success:
                node.get_logger().error(f"Stopping route because '{place_name}' failed")
                sys.exit(1)

            is_last_place = index == len(requested_places) - 1

            if not is_last_place:
                node.get_logger().info(
                    f"Waiting {WAIT_BETWEEN_PLACES:.1f} seconds before next destination..."
                )
                time.sleep(WAIT_BETWEEN_PLACES)

        node.get_logger().info("Finished all requested destinations")
        sys.exit(0)

    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user")
        sys.exit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()