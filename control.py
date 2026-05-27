import math
import subprocess
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.playback_state_pub = self.create_publisher(String, '/voice_playback_state', 10)

        self.cmd_sub = self.create_subscription(
            String,
            '/voice_motion_cmd',
            self.command_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.current_yaw = None
        self.turning = False
        self.target_yaw = None
        self.turn_direction = 0.0

        self.turn_speed = 0.45
        self.yaw_tolerance = 0.035

        self.current_motion = 'stop'

        self.front_stop_distance = 0.30
        self.back_stop_distance = 0.30

        self.front_angle_width = math.radians(30.0)
        self.back_angle_width = math.radians(30.0)

        self.wall_in_front = False
        self.wall_behind = False

        self.last_front_distance = None
        self.last_back_distance = None

        self.turn_timer = self.create_timer(0.05, self.turn_control_loop)
        self.safety_timer = self.create_timer(0.05, self.safety_control_loop)

        self.get_logger().info('Control node started. Waiting for /voice_motion_cmd...')

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def scan_callback(self, msg: LaserScan):
        min_front_distance = None
        min_back_distance = None

        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:

                if -self.front_angle_width <= angle <= self.front_angle_width:
                    if min_front_distance is None or r < min_front_distance:
                        min_front_distance = r

                # Behind the robot is around +pi or -pi.
                if abs(abs(angle) - math.pi) <= self.back_angle_width:
                    if min_back_distance is None or r < min_back_distance:
                        min_back_distance = r

            angle += msg.angle_increment

        self.last_front_distance = min_front_distance
        self.last_back_distance = min_back_distance

        if min_front_distance is None:
            self.wall_in_front = False
        else:
            self.wall_in_front = min_front_distance <= self.front_stop_distance

        if min_back_distance is None:
            self.wall_behind = False
        else:
            self.wall_behind = min_back_distance <= self.back_stop_distance

    def safety_control_loop(self):
        if self.current_motion == 'forward' and self.wall_in_front:
            self.get_logger().warn(
                f'Wall detected in front at {self.last_front_distance:.2f} m. Stopping.'
            )
            self.current_motion = 'stop'
            self.publish_twist(0.0, 0.0)
            self.play_obstacle_sound()

        elif self.current_motion == 'backward' and self.wall_behind:
            self.get_logger().warn(
                f'Wall detected behind at {self.last_back_distance:.2f} m. Stopping.'
            )
            self.current_motion = 'stop'
            self.publish_twist(0.0, 0.0)
            self.play_obstacle_sound()

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received motion command: "{command}"')

        if command == 'forward':
            self.cancel_turn_if_needed()

            if self.wall_in_front:
                distance_text = 'unknown'
                if self.last_front_distance is not None:
                    distance_text = f'{self.last_front_distance:.2f} m'

                self.get_logger().warn(
                    f'Forward command ignored because wall is in front: {distance_text}'
                )
                self.current_motion = 'stop'
                self.publish_twist(0.0, 0.0)
                self.play_obstacle_sound()
                return

            self.current_motion = 'forward'
            self.publish_twist(0.08, 0.0)
            self.play_sound('/home/ubuntu/forward.wav')            

        elif command == 'backward':
            self.cancel_turn_if_needed()

            if self.wall_behind:
                distance_text = 'unknown'
                if self.last_back_distance is not None:
                    distance_text = f'{self.last_back_distance:.2f} m'

                self.get_logger().warn(
                    f'Backward command ignored because wall is behind: {distance_text}'
                )
                self.current_motion = 'stop'
                self.publish_twist(0.0, 0.0)
                self.play_obstacle_sound()
                return

            self.current_motion = 'backward'
            self.publish_twist(-0.08, 0.0)
            self.play_sound('/home/ubuntu/backward.wav')

        elif command == 'left':
            self.current_motion = 'turning'
            self.start_90_degree_turn(direction='left')
            self.play_sound('/home/ubuntu/tleft.wav')

        elif command == 'right':
            self.current_motion = 'turning'
            self.start_90_degree_turn(direction='right')
            self.play_sound('/home/ubuntu/tright.wav')

        elif command == 'stop':
            self.current_motion = 'stop'
            self.turning = False
            self.target_yaw = None
            self.turn_direction = 0.0
            self.publish_twist(0.0, 0.0)
            self.play_sound('/home/ubuntu/stop.wav')

        else:
            self.get_logger().warn(f'Unknown motion command: "{command}"')

    def start_90_degree_turn(self, direction: str):
        if self.current_yaw is None:
            self.get_logger().warn('Cannot turn 90 degrees yet because /odom yaw is not available')
            self.current_motion = 'stop'
            return

        if direction == 'left':
            delta = math.pi / 2.0
            self.turn_direction = 1.0
        elif direction == 'right':
            delta = -math.pi / 2.0
            self.turn_direction = -1.0
        else:
            return

        self.target_yaw = self.normalize_angle(self.current_yaw + delta)
        self.turning = True

        self.get_logger().info(
            f'Starting 90 degree {direction} turn. '
            f'Current yaw={self.current_yaw:.3f}, target yaw={self.target_yaw:.3f}'
        )

    def turn_control_loop(self):
        if not self.turning:
            return

        if self.current_yaw is None or self.target_yaw is None:
            return

        error = self.angle_difference(self.target_yaw, self.current_yaw)

        if abs(error) <= self.yaw_tolerance:
            self.publish_twist(0.0, 0.0)
            self.turning = False
            self.target_yaw = None
            self.turn_direction = 0.0
            self.current_motion = 'stop'
            self.get_logger().info('90 degree turn complete')
            return

        msg = Twist()
        msg.angular.z = self.turn_speed if error > 0.0 else -self.turn_speed
        self.cmd_pub.publish(msg)

    def cancel_turn_if_needed(self):
        if self.turning:
            self.turning = False
            self.target_yaw = None
            self.turn_direction = 0.0
            self.current_motion = 'stop'
            self.publish_twist(0.0, 0.0)

    def publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f'Published /cmd_vel: linear.x={linear_x}, angular.z={angular_z}'
        )

    def play_obstacle_sound(self):
        self.play_sound('/home/ubuntu/obstacle.wav')

    def publish_playback_state(self, state: str):
        msg = String()
        msg.data = state
        self.playback_state_pub.publish(msg)

    def play_sound(self, wav_path: str):
        def worker():
            self.publish_playback_state('start')

            try:
                process = subprocess.Popen(
                    ['aplay', '-D', 'plughw:3,0', wav_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                process.wait()
            except Exception as e:
                self.get_logger().error(f'Failed to play sound {wav_path}: {e}')
            finally:
                self.publish_playback_state('stop')

        threading.Thread(target=worker, daemon=True).start()       

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def angle_difference(self, target: float, current: float):
        return self.normalize_angle(target - current)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()