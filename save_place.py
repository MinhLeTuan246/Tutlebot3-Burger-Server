import os
import subprocess
import yaml

SAVED_PLACES_FILE = "/home/ubuntu/saved_places.yaml"


def run_amcl_pose_once():
    print("Reading current robot pose from /amcl_pose...")
    print("Running: ros2 topic echo /amcl_pose --once")

    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "/amcl_pose", "--once"],
            capture_output=True,
            text=True,
            timeout=10
        )
    except subprocess.TimeoutExpired:
        print("ERROR: Timeout while waiting for /amcl_pose.")
        print("Make sure navigation is running, AMCL is active, and initial pose is set.")
        return None

    if result.returncode != 0:
        print("ERROR: Failed to read /amcl_pose.")
        print(result.stderr)
        return None

    return result.stdout


def parse_amcl_pose(output):
    try:
        docs = list(yaml.safe_load_all(output))

        data = None
        for doc in docs:
            if isinstance(doc, dict) and "pose" in doc:
                data = doc
                break

        if data is None:
            print("ERROR: No valid /amcl_pose message found in output.")
            return None

        pose = data["pose"]["pose"]

        x = float(pose["position"]["x"])
        y = float(pose["position"]["y"])

        z = float(pose["orientation"]["z"])
        w = float(pose["orientation"]["w"])

        frame_id = data.get("header", {}).get("frame_id", "map")

        return {
            "frame_id": frame_id,
            "x": x,
            "y": y,
            "z": z,
            "w": w
        }

    except Exception as e:
        print("ERROR: Could not parse /amcl_pose output.")
        print(f"Reason: {e}")
        return None


def load_saved_places():
    if not os.path.exists(SAVED_PLACES_FILE):
        return {}

    try:
        with open(SAVED_PLACES_FILE, "r") as f:
            data = yaml.safe_load(f)

        if data is None:
            return {}

        if not isinstance(data, dict):
            print("ERROR: saved_places.yaml must contain a top-level dictionary.")
            return None

        return data

    except Exception as e:
        print(f"ERROR: Could not read {SAVED_PLACES_FILE}")
        print(f"Reason: {e}")
        return None


def save_saved_places(data):
    try:
        with open(SAVED_PLACES_FILE, "w") as f:
            yaml.safe_dump(
                data,
                f,
                sort_keys=False,
                default_flow_style=False
            )
        return True

    except Exception as e:
        print(f"ERROR: Could not write to {SAVED_PLACES_FILE}")
        print(f"Reason: {e}")
        return False


def ask_location_name():
    while True:
        name = input("Enter location name to save: ").strip()

        if not name:
            print("Location name cannot be empty.")
            continue

        if " " in name:
            print("Avoid spaces. Use names like: dock, pos1, pos2, kitchen_table")
            continue

        return name


def ask_overwrite(name):
    while True:
        answer = input(f"Location '{name}' already exists. Overwrite it? [y/n]: ").strip().lower()

        if answer in ["y", "yes"]:
            return True

        if answer in ["n", "no"]:
            return False

        print("Please enter y or n.")


def main():
    output = run_amcl_pose_once()
    if output is None:
        return

    new_pose = parse_amcl_pose(output)
    if new_pose is None:
        return

    print("\nCurrent pose found:")
    print(f"  frame_id: {new_pose['frame_id']}")
    print(f"  x: {new_pose['x']}")
    print(f"  y: {new_pose['y']}")
    print(f"  z: {new_pose['z']}")
    print(f"  w: {new_pose['w']}")

    places = load_saved_places()
    if places is None:
        return

    name = ask_location_name()

    if name in places:
        overwrite = ask_overwrite(name)
        if not overwrite:
            print("Cancelled. Existing location was not changed.")
            return

    places[name] = new_pose

    if save_saved_places(places):
        print(f"\nSaved location '{name}' to {SAVED_PLACES_FILE}")
        print("\nUpdated saved_places.yaml:")
        print(yaml.safe_dump(places, sort_keys=False, default_flow_style=False))


if __name__ == "__main__":
    main()