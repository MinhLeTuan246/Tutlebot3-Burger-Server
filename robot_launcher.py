#!/usr/bin/env python3
import os
import signal
import subprocess
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import yaml
from std_msgs.msg import String

SAVED_PLACES_FILE = "/home/ubuntu/saved_places.yaml"


class RobotLauncher(Node):
    def __init__(self) -> None:
        super().__init__("robot_launcher")

        # Services
        self.start_srv = self.create_service(
            Trigger, "start_robot", self.start_robot_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "stop_robot", self.stop_robot_callback
        )
        self.save_map_srv = self.create_service(
            Trigger, "save_map", self.save_map_callback
        )

        # NEW: SLAM (cartographer) services
        self.start_slam_srv = self.create_service(
            Trigger, "start_slam", self.start_slam_callback
        )
        self.stop_slam_srv = self.create_service(
            Trigger, "stop_slam", self.stop_slam_callback
        )

        # NEW: Navigation (Nav2)
        self.start_nav_srv = self.create_service(
            Trigger, "start_navigation", self.start_navigation_callback
        )
        self.stop_nav_srv = self.create_service(
            Trigger, "stop_navigation", self.stop_navigation_callback
        )

        # NEW: Control and Guiding (Voice command package)
        self.start_control_srv = self.create_service(
            Trigger, "start_control", self.start_control_callback
        )
        self.stop_control_srv = self.create_service(
            Trigger, "stop_control", self.stop_control_callback
        )

        self.start_guiding_srv = self.create_service(
            Trigger, "start_guiding", self.start_guiding_callback
        )
        self.stop_guiding_srv = self.create_service(
            Trigger, "stop_guiding", self.stop_guiding_callback
        )        

        # UPDATE MODULE STATUS: a service to report current status of each module
        self.module_status_srv = self.create_service(
            Trigger, "get_module_status", self.get_module_status_callback
        )        

        # process handles
        self.bringup_process: Optional[subprocess.Popen] = None
        self.slam_process: Optional[subprocess.Popen] = None
        self.nav_process: Optional[subprocess.Popen] = None
        self.control_process: Optional[subprocess.Popen] = None
        self.guiding_process: Optional[subprocess.Popen] = None

        self.get_logger().info(
            "RobotLauncher ready. Services: /start_robot, /stop_robot, /start_slam, /stop_slam, /start_navigation, /stop_navigation, /save_map, /get_module_status"
        )

        # Map editor save-location request/response topics
        self.save_location_req_sub = self.create_subscription(
            String,
            "save_location_request",
            self.save_location_request_callback,
            10
        )

        self.save_location_res_pub = self.create_publisher(
            String,
            "save_location_response",
            10
        ) 

        # Map editor location list/delete/rename request/response topics
        self.location_manage_req_sub = self.create_subscription(
            String,
            "location_manage_request",
            self.location_manage_request_callback,
            10
        )

        self.location_manage_res_pub = self.create_publisher(
            String,
            "location_manage_response",
            10
        )               

    # ---------- helpers ----------
    def _is_running(self, p: Optional[subprocess.Popen]) -> bool:
        return p is not None and p.poll() is None

    def _play_sound(self, wav_file: str) -> None:
        try:
            subprocess.Popen(
                ["aplay", "-D", "plughw:3,0", f"/home/ubuntu/{wav_file}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to play sound {wav_file}: {e}")        

    def _pgrep_running(self, pattern: str) -> bool:
        try:
            completed = subprocess.run(
                ["pgrep", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )
            return completed.returncode == 0
        except Exception:
            return False

    def _bringup_running(self) -> bool:
        return self._is_running(self.bringup_process) or self._pgrep_running(
            "ros2 launch turtlebot3_bringup robot_ld19.launch.py"
        )

    def _slam_running(self) -> bool:
        return self._is_running(self.slam_process) or self._pgrep_running(
            "ros2 launch turtlebot3_cartographer cartographer.launch.py"
        )

    def _navigation_running(self) -> bool:
        return self._is_running(self.nav_process) or self._pgrep_running(
            "ros2 launch turtlebot3_navigation2 navigation2.launch.py"
        )

    def _control_running(self) -> bool:
        return self._is_running(self.control_process)

    def _guiding_running(self) -> bool:
        return self._is_running(self.guiding_process)

    def _stop_process_group(self, p: Optional[subprocess.Popen]) -> None:
        if not self._is_running(p):
            return

        pid = p.pid
        # SIGINT like Ctrl+C
        os.killpg(os.getpgid(pid), signal.SIGINT)

        try:
            p.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(pid), signal.SIGKILL)

    # ---------- bringup ----------
    def start_robot_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._is_running(self.bringup_process):
            response.success = False
            response.message = "Robot bringup is already running."
            return response

        try:
            self.bringup_process = subprocess.Popen(
                ["ros2", "launch", "turtlebot3_bringup", "robot_ld19.launch.py"],
                preexec_fn=os.setsid,  # NEW process group
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            response.success = True
            response.message = f"Robot bringup started (pid={self.bringup_process.pid})."
            self._play_sound("bringup_on.wav")
        except Exception as e:
            self.get_logger().error(f"Failed to start bringup: {e}")
            self.bringup_process = None
            response.success = False
            response.message = f"Failed to start bringup: {e}"

        return response

    def stop_robot_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            self._stop_process_group(self.guiding_process)
        except Exception:
            pass
        finally:
            self.guiding_process = None

        try:
            self._stop_process_group(self.control_process)
        except Exception:
            pass
        finally:
            self.control_process = None

        try:
            self._stop_process_group(self.nav_process)
        except Exception:
            pass
        finally:
            self.nav_process = None
        # Recommended: stop SLAM first if running
        try:
            self._stop_process_group(self.slam_process)
        except Exception:
            pass
        finally:
            self.slam_process = None

        if not self._is_running(self.bringup_process):
            response.success = True
            response.message = "Robot bringup is not running."
            self.bringup_process = None
            return response

        try:
            self._stop_process_group(self.bringup_process)
            response.success = True
            response.message = "Robot bringup stopped."
            self._play_sound("bringup_off.wav")
        except Exception as e:
            self.get_logger().error(f"Failed to stop bringup: {e}")
            response.success = False
            response.message = f"Failed to stop bringup: {e}"
        finally:
            self.bringup_process = None

        return response

    # ---------- SLAM (Cartographer) ----------
    def start_slam_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._is_running(self.slam_process):
            response.success = False
            response.message = "Cartographer is already running."
            return response

        # Safety: require bringup running so /scan, tf, etc. exist
        if not self._is_running(self.bringup_process):
            response.success = False
            response.message = "Bringup is not running. Call /start_robot first."
            return response

        try:
            self.slam_process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "turtlebot3_cartographer",
                    "cartographer.launch.py",
                    "use_sim_time:=false",
                    "rviz:=false",
                ],
                preexec_fn=os.setsid,  # NEW process group
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            response.success = True
            response.message = f"Cartographer started (pid={self.slam_process.pid})."
            self._play_sound("carto_on.wav")
        except Exception as e:
            self.get_logger().error(f"Failed to start cartographer: {e}")
            self.slam_process = None
            response.success = False
            response.message = f"Failed to start cartographer: {e}"

        return response

    def stop_slam_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._is_running(self.slam_process):
            response.success = True
            response.message = "Cartographer is not running."
            self.slam_process = None
            return response

        try:
            self._stop_process_group(self.slam_process)
            response.success = True
            response.message = "Cartographer stopped."
            self._play_sound("carto_off.wav")
        except Exception as e:
            self.get_logger().error(f"Failed to stop cartographer: {e}")
            response.success = False
            response.message = f"Failed to stop cartographer: {e}"
        finally:
            self.slam_process = None

        return response

    # ---------------------- Export map ----------------------
    def save_map_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Save the current /map to ~/map.* using nav2_map_server map_saver_cli."""
        try:
            out_prefix = os.path.expanduser("~/map")
            completed = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", out_prefix],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=20.0,
            )

            if completed.returncode == 0:
                response.success = True
                response.message = f"Map saved: {out_prefix}.yaml / {out_prefix}.pgm"
            else:
                response.success = False
                response.message = (
                    "map_saver_cli failed: "
                    + (completed.stderr.strip() or completed.stdout.strip() or "unknown error")
                )
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = "map_saver_cli timed out."
        except Exception as e:
            response.success = False
            response.message = f"Failed to save map: {e}"

        return response

    # ---------- Navigation (Nav2) ----------
    def start_navigation_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._is_running(self.nav_process):
            response.success = False
            response.message = "Navigation is already running."
            return response

        # Require bringup
        if not self._is_running(self.bringup_process):
            response.success = False
            response.message = "Bringup is not running. Call /start_robot first."
            return response

        try:
            self.nav_process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "turtlebot3_navigation2",
                    "navigation2.launch.py",
                    "use_sim_time:=false",
                    "map:=/home/ubuntu/map.yaml",
                    "rviz:=false",
                ],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            response.success = True
            response.message = f"Navigation started (pid={self.nav_process.pid})."
            self._play_sound("navi_on.wav")
        except Exception as e:
            self.nav_process = None
            response.success = False
            response.message = f"Failed to start navigation: {e}"

        return response

    def stop_navigation_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._is_running(self.nav_process):
            response.success = True
            response.message = "Navigation is not running."
            self.nav_process = None
            return response

        try:
            self._stop_process_group(self.nav_process)
            response.success = True
            response.message = "Navigation stopped."
            self._play_sound("navi_off.wav")
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop navigation: {e}"
        finally:
            self.nav_process = None

        return response

    # ---------- Control (Voice command package) ----------
    def start_control_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._is_running(self.control_process):
            response.success = False
            response.message = "Control is already running."
            return response

        if not self._bringup_running():
            response.success = False
            response.message = "Bringup is not running. Start bringup first."
            return response

        try:
            self.control_process = subprocess.Popen(
                ["ros2", "run", "robot_launcher", "control"],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            response.success = True
            response.message = f"Control started (pid={self.control_process.pid})."
            self._play_sound("control_on.wav")
        except Exception as e:
            self.control_process = None
            response.success = False
            response.message = f"Failed to start control: {e}"

        return response

    def stop_control_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._is_running(self.control_process):
            response.success = True
            response.message = "Control is not running."
            self.control_process = None
            return response

        try:
            self._stop_process_group(self.control_process)
            response.success = True
            response.message = "Control stopped."
            self._play_sound("control_off.wav")
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop control: {e}"
        finally:
            self.control_process = None

        return response

    # ---------- Guiding (Voice command package) ----------
    def start_guiding_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._is_running(self.guiding_process):
            response.success = False
            response.message = "Guiding is already running."
            return response

        if not self._navigation_running():
            response.success = False
            response.message = "Navigation is not running. Start navigation first."
            return response

        try:
            self.guiding_process = subprocess.Popen(
                ["ros2", "run", "robot_launcher", "guiding"],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            response.success = True
            response.message = f"Guiding started (pid={self.guiding_process.pid})."
            self._play_sound("guiding_on.wav")
        except Exception as e:
            self.guiding_process = None
            response.success = False
            response.message = f"Failed to start guiding: {e}"

        return response

    def stop_guiding_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._is_running(self.guiding_process):
            response.success = True
            response.message = "Guiding is not running."
            self.guiding_process = None
            return response

        try:
            self._stop_process_group(self.guiding_process)
            response.success = True
            response.message = "Guiding stopped."
            self._play_sound("guiding_off.wav")
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop guiding: {e}"
        finally:
            self.guiding_process = None

        return response

    def _load_saved_places(self):
        if not os.path.exists(SAVED_PLACES_FILE):
            return {}

        with open(SAVED_PLACES_FILE, "r") as f:
            data = yaml.safe_load(f)

        if data is None:
            return {}

        if not isinstance(data, dict):
            raise ValueError("saved_places.yaml must contain a top-level dictionary")

        return data

    # Save a location requested from the web app.
    def _write_saved_places(self, data):
        with open(SAVED_PLACES_FILE, "w") as f:
            yaml.safe_dump(
                data,
                f,
                sort_keys=False,
                default_flow_style=False
            )

    def _publish_save_location_response(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.save_location_res_pub.publish(msg)

    def save_location_request_callback(self, msg: String):
        try:
            req = json.loads(msg.data)
        except Exception as e:
            self._publish_save_location_response({
                "success": False,
                "exists": False,
                "message": f"Invalid JSON request: {e}",
                "request_id": None,
            })
            return

        request_id = req.get("request_id")
        name = str(req.get("name", "")).strip().lower().replace(" ", "_")
        overwrite = bool(req.get("overwrite", False))
        pose = req.get("pose")

        if not name:
            self._publish_save_location_response({
                "success": False,
                "exists": False,
                "message": "Location name cannot be empty.",
                "request_id": request_id,
            })
            return

        if not isinstance(pose, dict):
            self._publish_save_location_response({
                "success": False,
                "exists": False,
                "message": "No valid pose was provided.",
                "request_id": request_id,
            })
            return

        required = ["frame_id", "x", "y", "z", "w"]
        missing = [k for k in required if k not in pose]
        if missing:
            self._publish_save_location_response({
                "success": False,
                "exists": False,
                "message": f"Pose is missing fields: {missing}",
                "request_id": request_id,
            })
            return

        try:
            places = self._load_saved_places()

            exists = name in places
            if exists and not overwrite:
                self._publish_save_location_response({
                    "success": False,
                    "exists": True,
                    "message": f"Location '{name}' already exists. Overwrite?",
                    "request_id": request_id,
                    "name": name,
                    "pose": pose,
                })
                return

            places[name] = {
                "frame_id": str(pose.get("frame_id", "map")),
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose["z"]),
                "w": float(pose["w"]),
            }

            self._write_saved_places(places)

            self._publish_save_location_response({
                "success": True,
                "exists": exists,
                "message": f"Saved location '{name}' to {SAVED_PLACES_FILE}",
                "request_id": request_id,
                "name": name,
                "pose": places[name],
            })

            self.get_logger().info(f"Saved location '{name}' from web app.")

        except Exception as e:
            self._publish_save_location_response({
                "success": False,
                "exists": False,
                "message": f"Failed to save location: {e}",
                "request_id": request_id,
            })

    #--------- Location list/delete/rename ----------
    def _publish_location_manage_response(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.location_manage_res_pub.publish(msg)

    def _places_to_payload(self, places):
        items = []

        for name, pose in places.items():
            if not isinstance(pose, dict):
                continue

            items.append({
                "name": name,
                "frame_id": str(pose.get("frame_id", "map")),
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "z": float(pose.get("z", 0.0)),
                "w": float(pose.get("w", 1.0)),
            })

        return items

    def location_manage_request_callback(self, msg: String):
        try:
            req = json.loads(msg.data)
        except Exception as e:
            self._publish_location_manage_response({
                "success": False,
                "message": f"Invalid JSON request: {e}",
                "request_id": None,
                "action": None,
            })
            return

        request_id = req.get("request_id")
        action = str(req.get("action", "")).strip().lower()

        try:
            places = self._load_saved_places()

            if action == "list":
                self._publish_location_manage_response({
                    "success": True,
                    "message": "Loaded saved locations.",
                    "request_id": request_id,
                    "action": action,
                    "locations": self._places_to_payload(places),
                })
                return

            if action == "delete":
                name = str(req.get("name", "")).strip().lower().replace(" ", "_")

                if not name:
                    self._publish_location_manage_response({
                        "success": False,
                        "message": "Location name cannot be empty.",
                        "request_id": request_id,
                        "action": action,
                    })
                    return

                if name not in places:
                    self._publish_location_manage_response({
                        "success": False,
                        "message": f"Location '{name}' does not exist.",
                        "request_id": request_id,
                        "action": action,
                    })
                    return

                deleted = places.pop(name)
                self._write_saved_places(places)

                self._publish_location_manage_response({
                    "success": True,
                    "message": f"Deleted location '{name}'.",
                    "request_id": request_id,
                    "action": action,
                    "deleted": {
                        "name": name,
                        "pose": deleted,
                    },
                    "locations": self._places_to_payload(places),
                })

                self.get_logger().info(f"Deleted location '{name}' from web app.")
                return

            if action == "rename":
                old_name = str(req.get("old_name", "")).strip().lower().replace(" ", "_")
                new_name = str(req.get("new_name", "")).strip().lower().replace(" ", "_")
                overwrite = bool(req.get("overwrite", False))

                if not old_name or not new_name:
                    self._publish_location_manage_response({
                        "success": False,
                        "exists": False,
                        "message": "Old name and new name are required.",
                        "request_id": request_id,
                        "action": action,
                    })
                    return

                if old_name not in places:
                    self._publish_location_manage_response({
                        "success": False,
                        "exists": False,
                        "message": f"Location '{old_name}' does not exist.",
                        "request_id": request_id,
                        "action": action,
                    })
                    return

                if old_name == new_name:
                    self._publish_location_manage_response({
                        "success": True,
                        "exists": False,
                        "message": "Name was not changed.",
                        "request_id": request_id,
                        "action": action,
                        "locations": self._places_to_payload(places),
                    })
                    return

                if new_name in places and not overwrite:
                    self._publish_location_manage_response({
                        "success": False,
                        "exists": True,
                        "message": f"Location '{new_name}' already exists. Overwrite?",
                        "request_id": request_id,
                        "action": action,
                        "old_name": old_name,
                        "new_name": new_name,
                    })
                    return

                pose = places.pop(old_name)
                places[new_name] = pose
                self._write_saved_places(places)

                self._publish_location_manage_response({
                    "success": True,
                    "exists": new_name in places,
                    "message": f"Renamed location '{old_name}' to '{new_name}'.",
                    "request_id": request_id,
                    "action": action,
                    "old_name": old_name,
                    "new_name": new_name,
                    "locations": self._places_to_payload(places),
                })

                self.get_logger().info(f"Renamed location '{old_name}' to '{new_name}' from web app.")
                return

            self._publish_location_manage_response({
                "success": False,
                "message": f"Unknown action '{action}'.",
                "request_id": request_id,
                "action": action,
            })

        except Exception as e:
            self._publish_location_manage_response({
                "success": False,
                "message": f"Failed to manage locations: {e}",
                "request_id": request_id,
                "action": action,
            })

    # ---------- Module status ----------
    def get_module_status_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        status = {
            "bringup": self._bringup_running(),
            "cartographer": self._slam_running(),
            "navigation": self._navigation_running(),
            "control": self._control_running(),
            "guiding": self._guiding_running(),            
        }

        response.success = True
        response.message = json.dumps(status)
        return response

def main() -> None:
    rclpy.init()
    node = RobotLauncher()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._stop_process_group(node.guiding_process)
        except Exception:
            pass

        try:
            node._stop_process_group(node.control_process)
        except Exception:
            pass

        try:
            node._stop_process_group(node.nav_process)
        except Exception:
            pass

        try:
            node._stop_process_group(node.slam_process)
        except Exception:
            pass

        try:
            node._stop_process_group(node.bringup_process)
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
