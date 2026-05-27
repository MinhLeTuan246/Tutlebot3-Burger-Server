import os
import yaml

SAVED_PLACES_FILE = "/home/ubuntu/saved_places.yaml"


def load_saved_places():
    if not os.path.exists(SAVED_PLACES_FILE):
        print(f"ERROR: {SAVED_PLACES_FILE} does not exist.")
        return None

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


def list_places(places):
    if not places:
        print("No saved locations found.")
        return

    print("\nSaved locations:")
    for i, name in enumerate(places.keys(), start=1):
        p = places[name]
        print(
            f"{i}. {name} "
            f"(x={p.get('x')}, y={p.get('y')}, z={p.get('z')}, w={p.get('w')})"
        )


def ask_place_to_delete(places):
    names = list(places.keys())

    while True:
        choice = input("\nEnter location name or number to delete, or 'q' to cancel: ").strip()

        if choice.lower() in ["q", "quit", "cancel"]:
            return None

        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(names):
                return names[index - 1]
            print("Invalid number.")
            continue

        if choice in places:
            return choice

        print(f"Location '{choice}' not found.")


def ask_confirm_delete(name):
    while True:
        answer = input(f"Delete location '{name}'? This cannot be undone. [y/n]: ").strip().lower()

        if answer in ["y", "yes"]:
            return True

        if answer in ["n", "no"]:
            return False

        print("Please enter y or n.")


def main():
    places = load_saved_places()
    if places is None:
        return

    if not places:
        print("No saved locations to delete.")
        return

    list_places(places)

    name = ask_place_to_delete(places)
    if name is None:
        print("Cancelled.")
        return

    if not ask_confirm_delete(name):
        print("Cancelled. Nothing deleted.")
        return

    deleted = places.pop(name)

    if save_saved_places(places):
        print(f"\nDeleted location '{name}'.")
        print("Removed parameters:")
        print(yaml.safe_dump({name: deleted}, sort_keys=False, default_flow_style=False))

        print("Remaining locations:")
        if places:
            for place_name in places.keys():
                print(f"- {place_name}")
        else:
            print("- none")


if __name__ == "__main__":
    main()