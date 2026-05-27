# TurtleBot3 Burger Robot Server

This folder contains the robot-side files used to control a TurtleBot3 Burger robot with ROS 2 Humble.  
The system supports web-based control, SLAM, Navigation2, saved-location guiding, manual voice movement, offline voice control, and audio feedback.

## File Overview

| File | Purpose |
|---|---|
| `robot_with_bridge.launch.py` | Starts `rosbridge_websocket` and the custom `robot_launcher` node. This allows the web app to communicate with ROS 2. |
| `robot_launcher.py` | Main ROS 2 service manager. It starts/stops Bringup, SLAM, Navigation, Control, and Guiding modules. It also handles map saving and saved-location management. |
| `control.py` | Manual movement control node. It receives motion commands and publishes `/cmd_vel`. It also checks LiDAR obstacles before moving forward or backward. |
| `guiding.py` | Main guiding state machine. It receives guiding commands, calls `goto_place.py`, handles pause/resume, route execution, force return to dock, and automatic return. |
| `goto_place.py` | Navigation helper script. It reads target coordinates from `saved_places.yaml` and sends goals to Nav2 through `/navigate_to_pose`. |
| `voice_command_node.py` | Offline voice command node using Vosk. It listens to the USB microphone and sends movement or guiding commands to ROS topics. |
| `save_place.py` | Terminal helper script for saving the current `/amcl_pose` into `saved_places.yaml`. |
| `delete_place.py` | Terminal helper script for deleting saved locations from `saved_places.yaml`. |
| `saved_places.yaml` | Local database of saved robot destinations such as `dock`, `table`, `kitchen`, and `door`. |
| `burger.yaml` | Navigation2 parameter configuration for the TurtleBot3 Burger. |
| `README.md` | Project documentation. |

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot3 Burger
- LD19 LiDAR
- Navigation2
- Cartographer
- rosbridge_server
- Python 3
- Vosk offline speech recognition model
- USB microphone
- Speaker connected to ALSA device `plughw:3,0`

## Expected File Locations

On the robot, the files are usually placed inside the `robot_launcher` ROS 2 package:

```bash
~/turtlebot3_ws/src/robot_launcher/
```

Python nodes should be placed inside:

```bash
~/turtlebot3_ws/src/robot_launcher/robot_launcher/
```

Example:

```bash
~/turtlebot3_ws/src/robot_launcher/robot_launcher/robot_launcher.py
~/turtlebot3_ws/src/robot_launcher/robot_launcher/control.py
~/turtlebot3_ws/src/robot_launcher/robot_launcher/guiding.py
~/turtlebot3_ws/src/robot_launcher/robot_launcher/voice_command_node.py
```

The saved-location file is expected at:

```bash
/home/ubuntu/saved_places.yaml
```

The Vosk model is expected at:

```bash
/home/ubuntu/vosk_models/vosk-model-small-en-us-0.15
```

## Build

After copying or editing the files, rebuild the ROS 2 package:

```bash
cd ~/turtlebot3_ws
colcon build --packages-select robot_launcher
source install/setup.bash
```

## Main Launch Command

Start the web bridge and robot launcher:

```bash
ros2 launch robot_launcher robot_with_bridge.launch.py
```

This starts:

- `rosbridge_websocket`
- `robot_launcher`

The web app connects to rosbridge using:

```text
ws://<robot-ip-address>:9090
```

Example:

```text
ws://172.20.10.2:9090
```

## Robot Launcher Services

`robot_launcher.py` provides services that can be called by the web app through rosbridge.

| Service | Function |
|---|---|
| `/start_robot` | Start TurtleBot3 Bringup |
| `/stop_robot` | Stop TurtleBot3 Bringup |
| `/start_slam` | Start Cartographer SLAM |
| `/stop_slam` | Stop Cartographer SLAM |
| `/start_navigation` | Start Navigation2 |
| `/stop_navigation` | Stop Navigation2 |
| `/start_control` | Start manual voice movement node |
| `/stop_control` | Stop manual voice movement node |
| `/start_guiding` | Start saved-location guiding node |
| `/stop_guiding` | Stop saved-location guiding node |
| `/save_map` | Save the current SLAM map |
| `/get_module_status` | Return module status as JSON |

Example service call:

```bash
ros2 service call /start_robot std_srvs/srv/Trigger
```

## Main ROS Topics

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity command |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan data |
| `/odom` | `nav_msgs/msg/Odometry` | Robot odometry |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Localized robot pose |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM or navigation map |
| `/tf` | `tf2_msgs/msg/TFMessage` | Robot transforms |
| `/voice_motion_cmd` | `std_msgs/msg/String` | Movement commands for `control.py` |
| `/voice_guiding_cmd` | `std_msgs/msg/String` | Guiding commands for `guiding.py` |
| `/guiding_state` | `std_msgs/msg/String` | Current guiding state |
| `/voice_playback_state` | `std_msgs/msg/String` | Used to suspend voice listening during sound playback |
| `/save_location_request` | `std_msgs/msg/String` | Save-location request from web app |
| `/save_location_response` | `std_msgs/msg/String` | Save-location response to web app |
| `/location_manage_request` | `std_msgs/msg/String` | List, rename, or delete saved locations |
| `/location_manage_response` | `std_msgs/msg/String` | Saved-location management response |

## Module Details

### `control.py`

This node receives movement commands from:

```bash
/voice_motion_cmd
```

Supported commands:

```text
forward
backward
left
right
stop
```

It publishes velocity to:

```bash
/cmd_vel
```

Current movement settings:

```text
Forward speed: 0.08 m/s
Backward speed: -0.08 m/s
Turn speed: 0.45 rad/s
Front obstacle stop distance: 0.30 m
Back obstacle stop distance: 0.30 m
```

The robot stops if an obstacle is detected within 30 cm in the moving direction.

### `guiding.py`

This node receives guiding commands from:

```bash
/voice_guiding_cmd
```

It publishes its current state to:

```bash
/guiding_state
```

Guiding states:

```text
READY
GUIDING
PAUSING
RETURNING
```

Supported behavior:

- Navigate to one saved location
- Navigate through multiple saved locations
- Optimize route order using nearest-next distance
- Pause guiding
- Resume after 10 seconds
- Return to dock after finishing a route
- Force return to dock when connection is lost
- Reject invalid locations

Important timing values:

```text
Wait between route places: 3 seconds
Pause wait time: 10 seconds
Return after final destination: 10 seconds
```

Example command:

```bash
ros2 topic pub --once /voice_guiding_cmd std_msgs/msg/String "{data: 'go to table'}"
```

Force return command:

```bash
ros2 topic pub --once /voice_guiding_cmd std_msgs/msg/String "{data: 'force return to dock'}"
```

### `goto_place.py`

This script reads a target from:

```bash
/home/ubuntu/saved_places.yaml
```

and sends a Nav2 action goal to:

```bash
/navigate_to_pose
```

Example:

```bash
python3 /home/ubuntu/goto_place.py table
```

Multiple destinations:

```bash
python3 /home/ubuntu/goto_place.py table kitchen dock
```

Optimized order:

```bash
python3 /home/ubuntu/goto_place.py --optimize table kitchen door
```

If the location does not exist, the script prints:

```text
Location does not exist
```

### `voice_command_node.py`

This node provides offline voice command support using Vosk.

It listens to the USB microphone and publishes commands to:

```bash
/voice_motion_cmd
/voice_guiding_cmd
```

It also supports typed testing through:

```bash
/typed_voice_cmd
```

Example typed test:

```bash
ros2 topic pub --once /typed_voice_cmd std_msgs/msg/String "{data: 'go to the table'}"
```

Common supported phrases:

```text
turn on bring up
turn off bring up
turn on mapping
turn off mapping
turn on navigation
turn off navigation
turn on control
turn off control
turn on guiding
turn off guiding
forward
backward
left
right
stop
go to the dock
go to the table
go to the kitchen
pause guiding
return to the dock
```

### `save_place.py`

This terminal helper saves the current robot pose from:

```bash
/amcl_pose
```

Run:

```bash
python3 /home/ubuntu/save_place.py
```

Requirements:

- Navigation2 must be running
- AMCL must be active
- Initial pose must already be set

### `delete_place.py`

This terminal helper lists saved locations and deletes one selected location.

Run:

```bash
python3 /home/ubuntu/delete_place.py
```

### `saved_places.yaml`

This file stores saved map locations.

Example format:

```yaml
dock:
  frame_id: map
  x: 0.047
  y: -0.012
  z: -0.00001
  w: 0.99999
table:
  frame_id: map
  x: 0.463
  y: -0.012
  z: -0.047
  w: 0.998
```

The values `z` and `w` are the orientation quaternion values used by Nav2.

### `burger.yaml`

This is the Navigation2 parameter file for the TurtleBot3 Burger. It contains configuration for:

- AMCL localization
- BT Navigator
- Controller Server
- Planner Server
- Recovery behaviors
- Local costmap
- Global costmap
- Map server
- Lifecycle manager

Use it with Navigation2 as part of the TurtleBot3 navigation launch configuration.

## Typical Workflow

### 1. Start the web bridge and launcher

```bash
ros2 launch robot_launcher robot_with_bridge.launch.py
```

### 2. Start robot modules from the web app

Use the web app buttons to start:

```text
Bringup
Navigation
Control
Guiding
```

Or call the services manually:

```bash
ros2 service call /start_robot std_srvs/srv/Trigger
ros2 service call /start_navigation std_srvs/srv/Trigger
ros2 service call /start_control std_srvs/srv/Trigger
ros2 service call /start_guiding std_srvs/srv/Trigger
```

### 3. Set the initial pose

Before navigation, set the robot initial pose from the web app or RViz.

### 4. Save important locations

Use the web app Map Editor or run:

```bash
python3 /home/ubuntu/save_place.py
```

### 5. Navigate to saved locations

From the web app or voice command:

```text
go to the table
go to the kitchen
return to the dock
```

Manual terminal test:

```bash
python3 /home/ubuntu/goto_place.py table
```

## Map Saving

To save the current SLAM map through the launcher:

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

The expected output files are usually:

```bash
/home/ubuntu/map.yaml
/home/ubuntu/map.pgm
```

## Audio Feedback

The system uses `aplay` with this ALSA device:

```bash
aplay -D plughw:3,0 <sound_file>.wav
```

Common sound files include:

```text
voice_control_on.wav
voice_control_off.wav
bringup_on.wav
bringup_off.wav
mapping_on.wav
mapping_off.wav
navigation_on.wav
navigation_off.wav
guiding_on.wav
guiding_off.wav
guiding_on_my_way.wav
guiding_reached.wav
returning.wav
pausing.wav
pausing_resume.wav
guiding_invalid.wav
ready.wav
```

Make sure the `.wav` files exist in:

```bash
/home/ubuntu/
```

## Debug Commands

Check running ROS nodes:

```bash
ros2 node list
```

Check guiding state:

```bash
ros2 topic echo /guiding_state
```

Check who publishes/subscribes to guiding state:

```bash
ros2 topic info /guiding_state -v
```

Check guiding commands:

```bash
ros2 topic info /voice_guiding_cmd -v
```

Check module status:

```bash
ros2 service call /get_module_status std_srvs/srv/Trigger
```

Check leftover processes:

```bash
pgrep -af guiding
pgrep -af robot_launcher
pgrep -af navigation
pgrep -af voice_command_node
```

Kill leftover guiding process:

```bash
pkill -f guiding
```

## Notes

- Internet is not required for local ROS 2 navigation after the system is already running.
- If only the web connection is lost, Bringup, Navigation2, and Guiding should continue running.
- If Navigation2 or AMCL is restarted, the initial pose must be set again.
- If the robot is physically moved by hand, the initial pose should be set again.
- If a saved-location command works while Guiding is shown as disabled, check for a leftover `guiding.py` process.
- `rosbridge` is only the communication layer between the web app and ROS 2. Nav2 does not need rosbridge after it receives a goal.
