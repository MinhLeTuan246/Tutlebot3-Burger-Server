import json
import os
import queue
import re
import signal
import subprocess
import threading
import yaml
import time

import pyaudio
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vosk import KaldiRecognizer, Model

SAVED_PLACES_FILE = "/home/ubuntu/saved_places.yaml"


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        self.model_path = '/home/ubuntu/vosk_models/vosk-model-small-en-us-0.15'
        self.sample_rate = 44100
        self.usb_mic_index = 4

        self.motion_commands = [
            'forward',
            'backward',
            'left',
            'right',
            'stop',
        ]

        self.motion_pub = self.create_publisher(String, '/voice_motion_cmd', 10)
        self.guiding_pub = self.create_publisher(String, '/voice_guiding_cmd', 10)
        self.playback_state_sub = self.create_subscription(String, '/voice_playback_state', self.playback_state_callback, 10)        

        self.typed_cmd_sub = self.create_subscription(
            String,
            '/typed_voice_cmd',
            self.typed_command_callback,
            10
        )

        self.get_logger().info('Loading Vosk model...')
        self.model = Model(self.model_path)

        # IMPORTANT:
        # Do not restrict Vosk to a fixed grammar list anymore.
        # This allows natural phrases like:
        # "please go to kitchen"
        # "go to pos1 then pos2"
        # "go to pos1 and pos2 in minimal time"
        self.vosk_grammar = self.build_vosk_grammar()
        self.recognizer = KaldiRecognizer(
            self.model,
            self.sample_rate,
            json.dumps(self.vosk_grammar)
        )

        self.audio_queue = queue.Queue(maxsize=10)
        self.audio_running = True

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.usb_mic_index,
            frames_per_buffer=4000,
        )

        self.stream.start_stream()

        self.audio_thread = threading.Thread(
            target=self.audio_read_loop
        )
        self.audio_thread.start()

        self.bringup_process = None
        self.mapping_process = None
        self.navigation_process = None
        self.control_process = None
        self.guiding_process = None

        self.last_command_key = None

        self.listening_suspended = False
        self.suspend_lock = threading.Lock()
        self.suspend_count = 0     

        self.timer = self.create_timer(0.1, self.listen_loop)
        self.get_logger().info('Voice command node started. Speak now.')
        self.play_sound('/home/ubuntu/voice_control_on.wav')

    # ------------------------------------------------------------------
    # Text cleanup and interpretation
    # ------------------------------------------------------------------

    def normalize_text(self, text: str) -> str:
        text = ' '.join(text.strip().lower().split())

        replacements = {
            'turn on bringup': 'turn on bring up',
            'on bringup': 'turn on bring up',
            'on bring up': 'turn on bring up',
            'turn off bringup': 'turn off bring up',
            'turn on bring up up': 'turn on bring up',
            'turn off bring up up': 'turn off bring up',

            'turn on map ping': 'turn on mapping',
            'turn off map ping': 'turn off mapping',
            'turn on mappingg': 'turn on mapping',
            'turn off mappingg': 'turn off mapping',
            'turn on cartographer': 'turn on mapping',
            'turn off cartographer': 'turn off mapping',
            'start cartographer': 'turn on mapping',
            'stop cartographer': 'turn off mapping',

            'save maps': 'save map',
            'safe map': 'save map',
            'save nap': 'save map',

            'turn on nav': 'turn on navigation',
            'turn off nav': 'turn off navigation',
            'turn on navigationg': 'turn on navigation',
            'turn off navigationg': 'turn off navigation',
            'start nav': 'turn on navigation',
            'stop nav': 'turn off navigation',

            'turn on controller': 'turn on control',
            'turn off controller': 'turn off control',
            'turn on controls': 'turn on control',
            'turn off controls': 'turn off control',

            'turn on guide ding': 'turn on guiding',
            'turn off guide ding': 'turn off guiding',
            'turn on guidingg': 'turn on guiding',
            'turn off guidingg': 'turn off guiding',

            'pause': 'pause guiding',
            'return to dock': 'return to the dock',
            'return dock': 'return to the dock',
            'return to the doc': 'return to the dock',

            'forwardd': 'forward',
            'go forward': 'forward',
            'move forward': 'forward',

            'back word': 'backward',
            'go backward': 'backward',
            'move backward': 'backward',
            'reverse': 'backward',

            'leftward': 'left',
            'turn left': 'left',

            'rightward': 'right',
            'turn right': 'right',

            'stopward': 'stop',
            'halt': 'stop',
            'freeze': 'stop',
        }

        return replacements.get(text, text)

    def load_saved_location_names(self):
        try:
            with open(SAVED_PLACES_FILE, "r") as f:
                data = yaml.safe_load(f)

            if not isinstance(data, dict):
                return []

            return list(data.keys())

        except Exception as e:
            self.get_logger().warn(f"Could not load saved places for voice grammar: {e}")
            return []


    def build_vosk_grammar(self):
        locations = self.load_saved_location_names()

        grammar = [
            "[unk]",

            # Bringup
            "turn on bring up",
            "turn off bring up",
            "start bring up",
            "stop bring up",
            "turn on robot",
            "turn off robot",
            "start robot",
            "stop robot",

            # Mapping / cartographer
            "turn on mapping",
            "turn off mapping",
            "start mapping",
            "stop mapping",
            "turn on cartographer",
            "turn off cartographer",
            "start cartographer",
            "stop cartographer",
            "save map",

            # Navigation
            "turn on navigation",
            "turn off navigation",
            "start navigation",
            "stop navigation",
            "turn on nav",
            "turn off nav",

            # Control
            "turn on control",
            "turn off control",
            "start control",
            "stop control",
            "turn on controller",
            "turn off controller",

            # Guiding
            "turn on guiding",
            "turn off guiding",
            "start guiding",
            "stop guiding",
            "pause guiding",
            "pause",
            "return to the dock",
            "return to dock",
            "go to dock",
            "go home",
            "return home",

            # Motion
            "forward",
            "move forward",
            "go forward",
            "backward",
            "move backward",
            "go backward",
            "reverse",
            "left",
            "turn left",
            "right",
            "turn right",
            "stop",
            "halt",
            "freeze",
        ]

        # Add dynamic single-location and route phrases.
        for loc in locations:
            spoken = loc.replace("_", " ")

            if loc == "dock":
                continue

            grammar.extend([
                f"go to {spoken}",
                f"move to {spoken}",
                f"please go to {spoken}",
                f"please move to {spoken}",
                f"navigate to {spoken}",
                f"guide to {spoken}",
                f"send robot to {spoken}",
            ])

        # Add common two-location route phrases.
        for a in locations:
            for b in locations:
                if a == b:
                    continue
                if a == "dock" or b == "dock":
                    continue

                aa = a.replace("_", " ")
                bb = b.replace("_", " ")

                grammar.extend([
                    f"go to {aa} then {bb}",
                    f"please go to {aa} then {bb}",
                    f"move to {aa} then {bb}",
                    f"go to {aa} and {bb}",
                    f"please go to {aa} and {bb}",
                    f"go to {aa} and {bb} in minimal time",
                    f"go to {aa} then {bb} in minimal time",
                    f"go to {aa} and {bb} in shortest route",
                    f"go to {aa} then {bb} in shortest route",
                ])

        # Remove duplicates while preserving order.
        unique = []
        seen = set()
        for phrase in grammar:
            phrase = phrase.strip()
            if phrase and phrase not in seen:
                unique.append(phrase)
                seen.add(phrase)

        self.get_logger().info(f"Loaded {len(unique)} Vosk grammar phrases")
        return unique

    def clean_location_target(self, raw_target: str) -> str:
        target = raw_target.lower().strip()
        target = re.sub(r'[.,!?]', '', target)
        target = re.sub(r'\b(the|a|an)\b', '', target)
        target = re.sub(r'\s+', ' ', target).strip()
        target = target.replace(' ', '_')

        aliases = {
            'position_one': 'pos1',
            'first_position': 'pos1',
            'pos_one': 'pos1',
            'pos_1': 'pos1',
            'point_one': 'pos1',
            'one': 'pos1',

            'position_two': 'pos2',
            'second_position': 'pos2',
            'pos_two': 'pos2',
            'pos_2': 'pos2',
            'point_two': 'pos2',
            'two': 'pos2',

            'charging_place': 'dock',
            'charging_station': 'dock',
            'home': 'dock',
            'base': 'dock',
            'doc': 'dock',
        }

        return aliases.get(target, target)

    def extract_guiding_route(self, text: str):
        optimize = re.search(
            r'\b(optimize|optimized|optimal|shortest|fastest|minimal|minimise|minimize|minimum|best route|least time)\b',
            text
        ) is not None

        match = re.search(
            r'\b(?:please\s+)?(?:move|go|navigate|guide|take|send|bring)\s+(?:the\s+robot\s+)?to\s+(.+)$',
            text
        )

        if not match:
            return None

        body = match.group(1).lower()
        body = re.sub(r'[.,!?]', ' ', body)

        # Remove route optimization words from the destination section.
        body = re.sub(
            r'\b(in|with|using)\s+(minimal|minimum|shortest|fastest|optimized|optimal|best)\s+(time|route|path|way)?\b',
            ' ',
            body
        )
        body = re.sub(r'\bminimal\s+time\b', ' ', body)
        body = re.sub(r'\bminimum\s+time\b', ' ', body)
        body = re.sub(r'\bshortest\s+(route|path|way)\b', ' ', body)
        body = re.sub(r'\boptimized\s+(route|path|way)\b', ' ', body)
        body = re.sub(r'\bbest\s+(route|path|way)\b', ' ', body)

        parts = re.split(r'\s+(?:then|and|after that|next)\s+|,', body)

        targets = []
        for part in parts:
            target = self.clean_location_target(part)
            if not target:
                continue
            if target == 'dock':
                continue
            if target not in targets:
                targets.append(target)

        if len(targets) < 2:
            return None

        return {
            'targets': targets,
            'optimize': optimize,
        }

    def extract_guiding_target(self, text: str):
        patterns = [
            r'\b(?:please\s+)?(?:move|go|navigate|guide|take|send|bring)\s+(?:the\s+robot\s+)?to\s+(.+)$',
            r'\b(?:please\s+)?(?:move|go|navigate|guide|take|send|bring)\s+(?:the\s+robot\s+)?(?:towards|into|inside)\s+(.+)$',
        ]

        for pattern in patterns:
            match = re.search(pattern, text)
            if not match:
                continue

            target = self.clean_location_target(match.group(1))
            if not target:
                return None

            if target == 'dock':
                return None

            return target

        return None

    def interpret_command(self, raw_text: str):
        text = self.normalize_text(raw_text)

        # System commands
        if re.search(r'(turn on|start|enable|run|activate).*(bring ?up|robot)', text) or \
           re.search(r'(bring ?up|robot).*(turn on|start|enable|run|activate)', text):
            return {'type': 'system', 'command': 'turn on bring up'}

        if re.search(r'(turn off|stop|disable|kill|shutdown).*(bring ?up|robot)', text) or \
           re.search(r'(bring ?up|robot).*(turn off|stop|disable|kill|shutdown)', text):
            return {'type': 'system', 'command': 'turn off bring up'}

        if re.search(r'(turn on|start|enable|run|activate).*(mapping|slam|cartographer)', text) or \
           re.search(r'(mapping|slam|cartographer).*(turn on|start|enable|run|activate)', text):
            return {'type': 'system', 'command': 'turn on mapping'}

        if re.search(r'(turn off|stop|disable|kill|shutdown).*(mapping|slam|cartographer)', text) or \
           re.search(r'(mapping|slam|cartographer).*(turn off|stop|disable|kill|shutdown)', text):
            return {'type': 'system', 'command': 'turn off mapping'}

        if text in ['save map', 'export map']:
            return {'type': 'system', 'command': 'save map'}

        if re.search(r'(turn on|start|enable|run|activate).*(navigation|nav2|navigate)', text) or \
           re.search(r'(navigation|nav2|navigate).*(turn on|start|enable|run|activate)', text):
            return {'type': 'system', 'command': 'turn on navigation'}

        if re.search(r'(turn off|stop|disable|kill|shutdown).*(navigation|nav2|navigate)', text) or \
           re.search(r'(navigation|nav2|navigate).*(turn off|stop|disable|kill|shutdown)', text):
            return {'type': 'system', 'command': 'turn off navigation'}

        if re.search(r'(turn on|start|enable|run|activate).*(control|controller)', text) or \
           re.search(r'(control|controller).*(turn on|start|enable|run|activate)', text):
            return {'type': 'system', 'command': 'turn on control'}

        if re.search(r'(turn off|stop|disable|kill|shutdown).*(control|controller)', text) or \
           re.search(r'(control|controller).*(turn off|stop|disable|kill|shutdown)', text):
            return {'type': 'system', 'command': 'turn off control'}

        if re.search(r'(turn on|start|enable|run|activate).*(guiding|guide)', text) or \
           re.search(r'(guiding|guide).*(turn on|start|enable|run|activate)', text):
            return {'type': 'system', 'command': 'turn on guiding'}

        if re.search(r'(turn off|stop|disable|kill|shutdown).*(guiding|guide)', text) or \
           re.search(r'(guiding|guide).*(turn off|stop|disable|kill|shutdown)', text):
            return {'type': 'system', 'command': 'turn off guiding'}

        # Guiding control commands
        if text in ['pause guiding', 'pause']:
            return {'type': 'guiding', 'command': 'pause guiding'}

        if text in ['return to the dock', 'return to dock', 'go to dock', 'go home', 'return home']:
            return {'type': 'guiding', 'command': 'return to the dock'}

        # Multi-location route commands
        route = self.extract_guiding_route(text)
        if route is not None:
            prefix = 'route --optimize' if route['optimize'] else 'route'
            command = prefix + ' ' + ' '.join(route['targets'])
            return {'type': 'guiding', 'command': command}

        # Single dynamic location command
        target = self.extract_guiding_target(text)
        if target is not None:
            return {'type': 'guiding', 'command': f'go to {target}'}

        # Legacy position commands
        if text in ['go to position one', 'go to the first position', 'go to pos one', 'go to pos 1']:
            return {'type': 'guiding', 'command': 'go to position one'}

        if text in ['go to position two', 'go to the second position', 'go to pos two', 'go to pos 2']:
            return {'type': 'guiding', 'command': 'go to position two'}

        # Motion commands
        if text in ['forward', 'ahead']:
            return {'type': 'motion', 'command': 'forward'}

        if text in ['backward', 'reverse', 'back']:
            return {'type': 'motion', 'command': 'backward'}

        if text == 'left':
            return {'type': 'motion', 'command': 'left'}

        if text == 'right':
            return {'type': 'motion', 'command': 'right'}

        if text == 'stop':
            return {'type': 'motion', 'command': 'stop'}

        return None

    # ------------------------------------------------------------------
    # Input handling
    # ------------------------------------------------------------------

    def typed_command_callback(self, msg: String):
        raw_text = msg.data.strip()

        if not raw_text:
            return

        parsed = self.interpret_command(raw_text)

        self.get_logger().info(
            f'TYPED RAW="{raw_text}" PARSED="{parsed}"'
        )

        if parsed is None:
            self.get_logger().warn('Ignored typed command: could not interpret')
            return

        self.execute_parsed_command(parsed)

    def audio_read_loop(self):
        while self.audio_running:
            try:
                data = self.stream.read(4000, exception_on_overflow=False)

                if not self.audio_running:
                    break

                if self.audio_queue.full():
                    try:
                        self.audio_queue.get_nowait()
                    except queue.Empty:
                        pass

                self.audio_queue.put_nowait(data)

            except Exception as e:
                if self.audio_running:
                    try:
                        self.get_logger().error(f'Audio read thread error: {e}')
                    except Exception:
                        pass
                break

    def listen_loop(self):
        if self.listening_suspended:
            self.clear_audio_queue()
            return

        try:
            data = self.audio_queue.get_nowait()
        except queue.Empty:
            return

        if self.recognizer.AcceptWaveform(data):
            result = json.loads(self.recognizer.Result())
            raw_text = result.get('text', '').strip()

            if not raw_text:
                return

            parsed = self.interpret_command(raw_text)

            self.get_logger().info(
                f'RAW="{raw_text}" PARSED="{parsed}"'
            )

            if parsed is None:
                self.get_logger().warn('Ignored: could not interpret command')
                return

            command_key = f"{parsed['type']}:{parsed['command']}"

            # Avoid repeated system/guiding commands from duplicated recognition.
            # Motion commands are allowed to repeat.
            if command_key == self.last_command_key and parsed['type'] != 'motion':
                return

            self.last_command_key = command_key
            self.execute_parsed_command(parsed)

    # ------------------------------------------------------------------
    # Command execution
    # ------------------------------------------------------------------

    def execute_parsed_command(self, parsed):
        command_type = parsed['type']
        command = parsed['command']

        if command_type == 'system':
            self.execute_system_command(command)

        elif command_type == 'motion':
            self.forward_motion_command(command)

        elif command_type == 'guiding':
            self.forward_guiding_command(command)

    def execute_system_command(self, command: str):
        if command == 'turn on bring up':
            self.start_bringup()

        elif command == 'turn off bring up':
            self.stop_bringup()

        elif command == 'turn on mapping':
            self.start_mapping()

        elif command == 'save map':
            self.save_map()

        elif command == 'turn off mapping':
            self.stop_mapping()

        elif command == 'turn on navigation':
            self.start_navigation()

        elif command == 'turn off navigation':
            self.stop_navigation()

        elif command == 'turn on control':
            self.start_control()

        elif command == 'turn off control':
            self.stop_control()

        elif command == 'turn on guiding':
            self.start_guiding()

        elif command == 'turn off guiding':
            self.stop_guiding()

    def forward_motion_command(self, command: str):
        if not self.is_process_running(self.control_process):
            self.get_logger().warn(
                f'Ignored motion command "{command}" because control is not running'
            )
            return

        msg = String()
        msg.data = command
        self.motion_pub.publish(msg)
        self.get_logger().info(f'Forwarded motion command to /voice_motion_cmd: "{command}"')
        self.suspend_listening_for(1.2)

    def forward_guiding_command(self, command: str):
        if not self.is_process_running(self.guiding_process):
            self.get_logger().warn(
                f'Ignored guiding command "{command}" because guiding is not running'
            )
            return

        msg = String()
        msg.data = command
        self.guiding_pub.publish(msg)
        self.get_logger().info(f'Forwarded guiding command to /voice_guiding_cmd: "{command}"')
        self.suspend_listening_for(1.5)

    # ------------------------------------------------------------------
    # Sound
    # ------------------------------------------------------------------

    def begin_listening_suspend(self):
        with self.suspend_lock:
            self.suspend_count += 1
            self.listening_suspended = True
            self.clear_audio_queue()

    def end_listening_suspend(self):
        with self.suspend_lock:
            if self.suspend_count > 0:
                self.suspend_count -= 1

            self.clear_audio_queue()

            if self.suspend_count == 0:
                self.listening_suspended = False

    def playback_state_callback(self, msg: String):
        state = msg.data.strip().lower()

        if state == 'start':
            self.get_logger().info('External playback started. Suspending microphone listening.')
            self.begin_listening_suspend()

        elif state == 'stop':
            self.get_logger().info('External playback stopped. Resuming microphone listening.')
            self.end_listening_suspend()

    def play_sound(self, wav_path: str):
        def worker():
            self.begin_listening_suspend()

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
                self.end_listening_suspend()

        threading.Thread(target=worker, daemon=True).start()

    def clear_audio_queue(self):
        try:
            while True:
                self.audio_queue.get_nowait()
        except queue.Empty:
            pass


    def suspend_listening_for(self, seconds: float):
        def worker():
            self.begin_listening_suspend()
            time.sleep(seconds)
            self.end_listening_suspend()

        threading.Thread(target=worker, daemon=True).start()           

    # ------------------------------------------------------------------
    # Process control
    # ------------------------------------------------------------------

    def start_bringup(self):
        if self.is_process_running(self.bringup_process):
            self.get_logger().warn('Bringup already running')
            return

        self.get_logger().info(
            'Starting bringup: ros2 launch turtlebot3_bringup robot_ld19.launch.py'
        )

        try:
            self.bringup_process = self.launch_process(
                ['ros2', 'launch', 'turtlebot3_bringup', 'robot_ld19.launch.py']
            )
            self.get_logger().info('Bringup process started')
            self.play_sound('/home/ubuntu/bringup_on.wav')
        except Exception as e:
            self.bringup_process = None
            self.get_logger().error(f'Failed to start bringup: {e}')

    def stop_bringup(self):
        if not self.is_process_running(self.bringup_process):
            self.get_logger().warn('Bringup is not running')
            self.bringup_process = None
            return

        self.get_logger().info('Stopping bringup process...')
        self.stop_process(self.bringup_process, 'Bringup')
        self.bringup_process = None
        self.play_sound('/home/ubuntu/bringup_off.wav')

    def start_mapping(self):
        if self.is_process_running(self.mapping_process):
            self.get_logger().warn('Mapping already running')
            return

        self.get_logger().info(
            'Starting mapping: ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false rviz:=false'
        )

        try:
            self.mapping_process = self.launch_process(
                [
                    'ros2',
                    'launch',
                    'turtlebot3_cartographer',
                    'cartographer.launch.py',
                    'use_sim_time:=false',
                    'rviz:=false',
                ]
            )
            self.get_logger().info('Mapping process started')
            self.play_sound('/home/ubuntu/carto_on.wav')
        except Exception as e:
            self.mapping_process = None
            self.get_logger().error(f'Failed to start mapping: {e}')

    def save_map(self):
        self.get_logger().info('Saving map: ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/map')

        try:
            subprocess.Popen(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', '/home/ubuntu/map'],
                env=os.environ.copy(),
                preexec_fn=os.setsid,
            )
            self.get_logger().info('Map save command started')
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')

    def stop_mapping(self):
        if not self.is_process_running(self.mapping_process):
            self.get_logger().warn('Mapping is not running')
            self.mapping_process = None
            return

        self.get_logger().info('Stopping mapping process...')
        self.stop_process(self.mapping_process, 'Mapping')
        self.mapping_process = None
        self.play_sound('/home/ubuntu/carto_off.wav')

    def start_navigation(self):
        if self.is_process_running(self.navigation_process):
            self.get_logger().warn('Navigation already running')
            return

        self.get_logger().info(
            'Starting navigation: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=false map:=/home/ubuntu/map.yaml rviz:=false'
        )

        try:
            self.navigation_process = self.launch_process(
                [
                    'ros2',
                    'launch',
                    'turtlebot3_navigation2',
                    'navigation2.launch.py',
                    'use_sim_time:=false',
                    'map:=/home/ubuntu/map.yaml',
                    'rviz:=false',
                ]
            )
            self.get_logger().info('Navigation process started')
            self.play_sound('/home/ubuntu/navi_on.wav')
        except Exception as e:
            self.navigation_process = None
            self.get_logger().error(f'Failed to start navigation: {e}')

    def stop_navigation(self):
        if not self.is_process_running(self.navigation_process):
            self.get_logger().warn('Navigation is not running')
            self.navigation_process = None
            return

        self.get_logger().info('Stopping navigation process...')
        self.stop_process(self.navigation_process, 'Navigation')
        self.navigation_process = None
        self.play_sound('/home/ubuntu/navi_off.wav')

    def start_control(self):
        if self.is_process_running(self.control_process):
            self.get_logger().warn('Control already running')
            return

        self.get_logger().info(
            'Starting control: ros2 run robot_launcher control'
        )

        try:
            self.control_process = self.launch_process(
                ['ros2', 'run', 'robot_launcher', 'control']
            )
            self.get_logger().info('Control process started')
            self.play_sound('/home/ubuntu/control_on.wav')

        except Exception as e:
            self.control_process = None
            self.get_logger().error(f'Failed to start control: {e}')

    def stop_control(self):
        if not self.is_process_running(self.control_process):
            self.get_logger().warn('Control is not running')
            self.control_process = None
            return

        self.get_logger().info('Stopping control process...')
        self.stop_process(self.control_process, 'Control')
        self.control_process = None
        self.play_sound('/home/ubuntu/control_off.wav')

    def start_guiding(self):
        if self.is_process_running(self.guiding_process):
            self.get_logger().warn('Guiding already running')
            return

        self.get_logger().info(
            'Starting guiding: ros2 run robot_launcher guiding'
        )

        try:
            self.guiding_process = self.launch_process(
                ['ros2', 'run', 'robot_launcher', 'guiding']
            )
            self.get_logger().info('Guiding process started')
            self.play_sound('/home/ubuntu/guiding_on.wav')

        except Exception as e:
            self.guiding_process = None
            self.get_logger().error(f'Failed to start guiding: {e}')

    def stop_guiding(self):
        if not self.is_process_running(self.guiding_process):
            self.get_logger().warn('Guiding is not running')
            self.guiding_process = None
            return

        self.get_logger().info('Stopping guiding process...')
        self.stop_process(self.guiding_process, 'Guiding')
        self.guiding_process = None
        self.play_sound('/home/ubuntu/guiding_off.wav')

    def launch_process(self, cmd):
        env = os.environ.copy()

        log_dir = '/home/ubuntu/voice_node_logs'
        os.makedirs(log_dir, exist_ok=True)

        process_name = cmd[2] if len(cmd) > 2 else 'process'
        log_path = os.path.join(log_dir, f'{process_name}.log')

        log_file = open(log_path, 'a')

        return subprocess.Popen(
            cmd,
            env=env,
            preexec_fn=os.setsid,
            stdout=log_file,
            stderr=log_file,
        )

    def stop_process(self, process, name: str):
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            self.get_logger().info(f'{name} process stopped')
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{name} did not stop in time, forcing kill...')
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=2)
                self.get_logger().info(f'{name} process killed')
            except Exception as e:
                self.get_logger().error(f'Failed to force kill {name}: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to stop {name}: {e}')

    def is_process_running(self, process):
        return process is not None and process.poll() is None

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        try:
            self.play_sound('/home/ubuntu/voice_control_off.wav')
        except Exception:
            pass

        try:
            if self.is_process_running(self.guiding_process):
                self.stop_process(self.guiding_process, 'Guiding')
            self.guiding_process = None
        except Exception:
            pass

        try:
            if self.is_process_running(self.control_process):
                self.stop_process(self.control_process, 'Control')
            self.control_process = None
        except Exception:
            pass

        try:
            if self.is_process_running(self.navigation_process):
                self.stop_process(self.navigation_process, 'Navigation')
            self.navigation_process = None
        except Exception:
            pass

        try:
            if self.is_process_running(self.mapping_process):
                self.stop_process(self.mapping_process, 'Mapping')
            self.mapping_process = None
        except Exception:
            pass

        try:
            if self.is_process_running(self.bringup_process):
                self.stop_process(self.bringup_process, 'Bringup')
            self.bringup_process = None
        except Exception:
            pass

        try:
            self.audio_running = False

            try:
                self.audio_thread.join(timeout=1.0)
            except Exception:
                pass

            try:
                if self.stream.is_active():
                    self.stream.stop_stream()
            except Exception:
                pass

            try:
                self.stream.close()
            except Exception:
                pass

            try:
                self.audio.terminate()
            except Exception:
                pass

        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        try:
            node.get_logger().error(f'Voice command node stopped with error: {e}')
        except Exception:
            pass

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()