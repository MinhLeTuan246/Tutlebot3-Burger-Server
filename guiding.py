import os
import signal
import subprocess
import time
import math
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal

SAVED_PLACES_FILE = '/home/ubuntu/saved_places.yaml'
WAIT_BETWEEN_ROUTE_PLACES = 3.0
PAUSE_WAIT_SECONDS = 10.0
RETURN_AFTER_GUIDING_SECONDS = 10.0

MAX_NAV_FAILURES_BEFORE_RETURN = 2

class GuidingNode(Node):
    READY = 'READY'
    GUIDING = 'GUIDING'
    PAUSING = 'PAUSING'
    RETURNING = 'RETURNING'

    def __init__(self):
        super().__init__('guiding')

        self.state = self.READY

        self.command_sub = self.create_subscription(
            String,
            '/voice_guiding_cmd',
            self.command_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/guiding_state',
            10
        )        

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cancel_nav_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )        

        self.current_process = None
        self.current_target = None
        self.previous_target = None
        self.process_stop_reason = None
        self.resume_from_pause = False

        self.return_deadline = None
        self.pause_deadline = None
        self.retry_deadline = None

        self.route_targets = []
        self.route_index = 0
        self.route_wait_deadline = None
        self.current_pose = None  

        self.force_return_active = False

        self.returning_sound_played = False

        self.nav_failure_count = 0
        self.block_path_sound_played = False

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )              

        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info('Guiding node started. State = READY')

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def handle_force_return_to_dock(self):
        if self.state == self.RETURNING:
            self.get_logger().warn('Force return ignored because robot is already RETURNING')
            return

        if self.force_return_active:
            self.get_logger().warn('Force return ignored because force return is already active')
            return

        if self.state == self.READY:
            self.get_logger().warn('Force return requested while READY. No active guiding route.')
            return

        self.force_return_active = True

        self.get_logger().warn('Force return to dock requested')

        self.return_deadline = None
        self.pause_deadline = None
        self.retry_deadline = None
        self.route_wait_deadline = None
        self.route_targets = []
        self.route_index = 0

        if self.is_process_running():
            self.stop_current_process('force_return_to_dock')

        self.cancel_navigation_goal()
        self.stop_robot_motion()

        self.previous_target = 'dock'
        self.returning_sound_played = False
        self.start_target('dock', self.RETURNING)

    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose


    def load_saved_places(self):
        try:
            with open(SAVED_PLACES_FILE, 'r') as f:
                data = yaml.safe_load(f)

            if not isinstance(data, dict):
                return {}

            return data
        except Exception as e:
            self.get_logger().warn(f'Could not load saved places: {e}')
            return {}


    def distance_xy(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


    def optimize_route(self, targets):
        data = self.load_saved_places()

        valid_targets = [t for t in targets if t in data]
        missing_targets = [t for t in targets if t not in data]

        for t in missing_targets:
            self.get_logger().warn(f'Location "{t}" does not exist in saved_places.yaml')

        if len(valid_targets) <= 1:
            return valid_targets

        if self.current_pose is None:
            self.get_logger().warn('No /amcl_pose yet. Using requested route order.')
            return valid_targets

        remaining = valid_targets[:]
        optimized = []

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        while remaining:
            nearest = None
            nearest_distance = None

            for target in remaining:
                place = data[target]
                d = self.distance_xy(
                    current_x,
                    current_y,
                    float(place['x']),
                    float(place['y'])
                )

                if nearest_distance is None or d < nearest_distance:
                    nearest_distance = d
                    nearest = target

            optimized.append(nearest)
            remaining.remove(nearest)

            current_x = float(data[nearest]['x'])
            current_y = float(data[nearest]['y'])

        self.get_logger().info('Optimized guiding route: ' + ' -> '.join(optimized))
        return optimized

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received guiding command: "{command}"')

        if command == 'force return to dock':
            self.handle_force_return_to_dock()
            return

        if command in ['go to position one', 'go to the first position']:
            self.handle_route_command(['pos1'], optimize=False)

        elif command in ['go to position two', 'go to the second position']:
            self.handle_route_command(['pos2'], optimize=False)

        elif command == 'pause guiding':
            self.handle_pause_guiding()

        elif command == 'return to the dock':
            self.handle_return_to_dock()

        elif command.startswith('route '):
            parts = command.split()
            optimize = False
            targets = []

            for p in parts[1:]:
                if p in ['--optimize', '-o', 'optimize', 'optimized']:
                    optimize = True
                else:
                    targets.append(p.strip())

            targets = [t.replace(' ', '_') for t in targets if t]

            if not targets:
                self.get_logger().warn('Ignoring route command because no targets were given')
                self.play_sound('/home/ubuntu/guiding_invalid.wav')
                return

            self.handle_route_command(targets, optimize=optimize)

        elif command.startswith('go to '):
            target = command[len('go to '):].strip()

            if not target:
                self.get_logger().warn('Ignoring go-to command because target is empty')
                self.play_sound('/home/ubuntu/guiding_invalid.wav')
                return

            if target in ['dock', 'the dock']:
                self.get_logger().warn('Use "return to the dock" for dock return logic')
                return

            target = target.replace(' ', '_')
            self.handle_route_command([target], optimize=False)

        else:
            self.get_logger().warn(f'Unknown guiding command: "{command}"')

    def handle_location_command(self, target: str):
        if self.state == self.RETURNING:
            self.get_logger().warn('Ignoring location command because state is RETURNING')
            return

        self.return_deadline = None
        self.pause_deadline = None
        self.retry_deadline = None

        if self.is_process_running():
            self.stop_current_process('replace')

        self.previous_target = target
        self.start_target(target, self.GUIDING)

    def handle_invalid_location_command(self, missing_targets):
        if isinstance(missing_targets, str):
            missing_targets = [missing_targets]

        missing_text = ", ".join(missing_targets)

        self.get_logger().warn(
            f'Invalid guiding command. Location does not exist: {missing_text}'
        )

        self.play_sound('/home/ubuntu/guiding_invalid.wav')

        now = time.monotonic()

        # If robot is paused, reset the pause countdown.
        if self.state == self.PAUSING and self.pause_deadline is not None:
            self.pause_deadline = now + PAUSE_WAIT_SECONDS
            self.get_logger().warn(
                f'Invalid command received while PAUSING. Pause timer reset to {PAUSE_WAIT_SECONDS:.1f} seconds.'
            )

        # If robot has reached the final destination and is waiting before auto-return,
        # reset the return-to-dock countdown.
        if (
            self.state == self.GUIDING
            and self.current_process is None
            and self.return_deadline is not None
        ):
            self.return_deadline = now + RETURN_AFTER_GUIDING_SECONDS
            self.get_logger().warn(
                f'Invalid command received during final wait. Return timer reset to {RETURN_AFTER_GUIDING_SECONDS:.1f} seconds.'
            )

    def handle_route_command(self, targets, optimize=False):
        if self.state == self.GUIDING and self.is_process_running():
            self.get_logger().warn(
                'Ignoring new destination command while actively GUIDING. Pause guiding first before changing destination.'
            )
            self.play_sound('/home/ubuntu/guiding_in_progress.wav')
            return

        data = self.load_saved_places()

        clean_targets = []
        for target in targets:
            target = str(target).strip().lower().replace(' ', '_')
            if target:
                clean_targets.append(target)

        if not clean_targets:
            self.get_logger().warn('Ignoring route command because no targets were given')
            self.play_sound('/home/ubuntu/guiding_invalid.wav')
            return

        missing_targets = [target for target in clean_targets if target not in data]

        if missing_targets:
            self.handle_invalid_location_command(missing_targets)
            return

        if self.state == self.RETURNING:
            self.get_logger().warn('Ignoring route command because state is RETURNING')
            return

        valid_targets = clean_targets

        if optimize:
            valid_targets = self.optimize_route(valid_targets)

        if not valid_targets:
            self.handle_invalid_location_command(clean_targets)
            return

        self.return_deadline = None
        self.pause_deadline = None
        self.retry_deadline = None
        self.route_wait_deadline = None

        if self.is_process_running():
            self.stop_current_process('replace')

        self.route_targets = valid_targets
        self.route_index = 0

        self.get_logger().info('Starting guiding route: ' + ' -> '.join(self.route_targets))
        self.start_next_route_target()

    def start_next_route_target(self):
        if self.route_index >= len(self.route_targets):
            self.get_logger().info('Route complete')
            self.route_targets = []
            self.route_index = 0
            self.current_target = None
            self.return_deadline = time.monotonic() + RETURN_AFTER_GUIDING_SECONDS
            return

        target = self.route_targets[self.route_index]
        self.start_target(target, self.GUIDING)

    def handle_pause_guiding(self):
        if self.state != self.GUIDING:
            self.get_logger().warn(f'Ignoring pause guiding because state is {self.state}')
            return

        if self.current_target is not None:
            self.previous_target = self.current_target

        if self.is_process_running():
            self.stop_current_process('pause')

        self.cancel_navigation_goal()
        self.stop_robot_motion()

        self.state = self.PAUSING
        self.return_deadline = None
        self.retry_deadline = None
        self.pause_deadline = time.monotonic() + PAUSE_WAIT_SECONDS

        self.play_sound('/home/ubuntu/pausing.wav')
        self.get_logger().info('State changed to PAUSING')

    def handle_return_to_dock(self):
        if self.state == self.RETURNING:
            self.get_logger().warn('Return-to-dock ignored because robot is already RETURNING')
            return

        if self.state != self.PAUSING:
            self.get_logger().warn(f'Ignoring return to dock because state is {self.state}')
            return

        self.return_deadline = None
        self.pause_deadline = None
        self.retry_deadline = None

        if self.is_process_running():
            self.stop_current_process('replace')

        self.previous_target = 'dock'
        self.start_target('dock', self.RETURNING)

    def start_target(self, target: str, state: str):
        self.state = state
        self.current_target = target
        self.return_deadline = None
        self.pause_deadline = None
        self.retry_deadline = None
        self.process_stop_reason = None

        cmd = ['python3', '/home/ubuntu/goto_place.py', target]
        self.get_logger().info(f'Starting target "{target}" with state {state}: {" ".join(cmd)}')


        try:
            self.current_process = subprocess.Popen(
                cmd,
                env=os.environ.copy(),
                preexec_fn=os.setsid,
            )

            if state == self.RETURNING:
                self.nav_failure_count = 0
                self.block_path_sound_played = False
                if not self.returning_sound_played:
                    self.play_sound('/home/ubuntu/returning.wav')
                    self.returning_sound_played = True
            elif state == self.GUIDING:
                if self.resume_from_pause:
                    self.resume_from_pause = False
                else:
                    self.play_sound('/home/ubuntu/guiding_on_my_way.wav')              

        except Exception as e:
            self.current_process = None
            self.get_logger().error(f'Failed to start target "{target}": {e}')
            self.retry_deadline = time.monotonic() + 1.0

    def tick(self):
        self.publish_state()
        self.check_process_result()

        now = time.monotonic()

        if self.state == self.GUIDING and self.route_wait_deadline is not None:
            if now >= self.route_wait_deadline:
                self.route_wait_deadline = None
                self.start_next_route_target()
            return        

        if self.retry_deadline is not None and now >= self.retry_deadline:
            target = self.current_target
            state = self.state
            self.retry_deadline = None

            if target is not None:
                self.get_logger().warn(f'Retrying target "{target}"')
                self.start_target(target, state)
            return

        if self.state == self.GUIDING and self.current_process is None and self.return_deadline is not None:
            if now >= self.return_deadline:
                self.get_logger().info('No new guiding command received. Returning to dock.')
                self.previous_target = 'dock'
                self.returning_sound_played = False
                self.start_target('dock', self.RETURNING)
            return

        if self.state == self.PAUSING and self.pause_deadline is not None:
            if now >= self.pause_deadline:
                self.pause_deadline = None

                self.play_sound('/home/ubuntu/pausing_resume.wav')
                self.resume_from_pause = True

                if self.route_targets and self.route_index < len(self.route_targets):
                    self.get_logger().info(
                        f'Resuming route at "{self.route_targets[self.route_index]}"'
                    )
                    self.state = self.GUIDING
                    self.start_next_route_target()
                    return

                if self.previous_target is None:
                    self.state = self.READY
                    self.get_logger().info('No previous target to resume. State changed to READY')
                    return

                if self.previous_target == 'dock':
                    self.get_logger().info('Resuming dock return')
                    self.returning_sound_played = False
                    self.start_target('dock', self.RETURNING)
                else:
                    self.get_logger().info(f'Resuming previous target "{self.previous_target}"')
                    self.start_target(self.previous_target, self.GUIDING)

    def check_process_result(self):
        if self.current_process is None:
            return

        returncode = self.current_process.poll()
        if returncode is None:
            return

        finished_target = self.current_target
        self.current_process = None

        if self.process_stop_reason is not None:
            self.get_logger().info(f'Process stopped intentionally: {self.process_stop_reason}')
            self.process_stop_reason = None
            return

        if returncode == 0:
            if self.state == self.GUIDING:
                self.get_logger().info(f'Successfully reached "{finished_target}"')
                self.play_sound('/home/ubuntu/guiding_reached.wav')

                if self.route_targets:
                    self.route_index += 1

                    if self.route_index < len(self.route_targets):
                        next_target = self.route_targets[self.route_index]
                        self.get_logger().info(
                            f'Waiting {WAIT_BETWEEN_ROUTE_PLACES:.1f} seconds before next target "{next_target}"'
                        )
                        self.route_wait_deadline = time.monotonic() + WAIT_BETWEEN_ROUTE_PLACES
                        return

                    self.get_logger().info('Successfully reached final route destination')
                    self.route_targets = []
                    self.route_index = 0

                self.return_deadline = time.monotonic() + RETURN_AFTER_GUIDING_SECONDS

            elif self.state == self.RETURNING:
                self.get_logger().info('Successfully reached dock')
                self.state = self.READY
                self.current_target = None
                self.previous_target = None
                self.return_deadline = None
                self.pause_deadline = None
                self.retry_deadline = None
                self.force_return_active = False
                self.returning_sound_played = False
                self.play_sound('/home/ubuntu/ready.wav')
                self.get_logger().info('State changed to READY')

        else:
            self.get_logger().warn(
                f'Target "{finished_target}" failed with return code {returncode}.'
            )

            if self.state == self.GUIDING:
                self.nav_failure_count += 1

                if self.nav_failure_count >= MAX_NAV_FAILURES_BEFORE_RETURN:
                    self.get_logger().error(
                        f'Target "{finished_target}" appears unreachable. Path may be blocked. Returning to dock.'
                    )

                    if not self.block_path_sound_played:
                        self.play_sound('/home/ubuntu/block_path.wav')
                        self.block_path_sound_played = True

                    self.cancel_navigation_goal()
                    self.stop_robot_motion()

                    self.route_targets = []
                    self.route_index = 0
                    self.route_wait_deadline = None
                    self.return_deadline = None
                    self.pause_deadline = None
                    self.retry_deadline = None

                    self.previous_target = 'dock'
                    self.returning_sound_played = False
                    self.start_target('dock', self.RETURNING)
                    return

                self.get_logger().warn(
                    f'Retrying target "{finished_target}" after failure {self.nav_failure_count}/{MAX_NAV_FAILURES_BEFORE_RETURN}.'
                )
                self.retry_deadline = time.monotonic() + 1.0
                return

            if self.state == self.RETURNING:
                self.get_logger().error('Dock return failed. Will retry dock return.')
                self.retry_deadline = time.monotonic() + 1.0
                return

    def stop_current_process(self, reason: str):
        if not self.is_process_running():
            self.current_process = None
            return

        self.process_stop_reason = reason
        self.get_logger().info(f'Stopping current process due to: {reason}')

        try:
            os.killpg(os.getpgid(self.current_process.pid), signal.SIGTERM)
            self.current_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Current process did not stop in time, forcing kill...')
            try:
                os.killpg(os.getpgid(self.current_process.pid), signal.SIGKILL)
                self.current_process.wait(timeout=2)
            except Exception as e:
                self.get_logger().error(f'Failed to force kill current process: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to stop current process: {e}')
        finally:
            self.current_process = None

    def is_process_running(self):
        return self.current_process is not None and self.current_process.poll() is None

    def play_sound(self, wav_path: str):
        try:
            subprocess.Popen(
                ['aplay', '-D', 'plughw:3,0', wav_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().error(f'Failed to play sound {wav_path}: {e}')

    def cancel_navigation_goal(self):
        if not self.cancel_nav_client.service_is_ready():
            self.get_logger().warn('Nav2 cancel service is not ready yet')
            return

        req = CancelGoal.Request()
        future = self.cancel_nav_client.call_async(req)
        future.add_done_callback(self.cancel_navigation_done)

        self.get_logger().info('Requested Nav2 goal cancel')


    def cancel_navigation_done(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Nav2 cancel response received. goals_canceling={len(response.goals_canceling)}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to cancel Nav2 goal: {e}')


    def stop_robot_motion(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(5):
            self.cmd_pub.publish(msg)

        self.get_logger().info('Published stop command to /cmd_vel')            

    def destroy_node(self):
        try:
            self.stop_current_process('shutdown')
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GuidingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()