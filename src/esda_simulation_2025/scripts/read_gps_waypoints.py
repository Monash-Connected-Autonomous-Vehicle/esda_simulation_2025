import yaml

class GPSWaypoint:
    def __init__(self):
        self.name = ""
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.quaternion = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self.rotation_rads = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

def read_gps_waypoints_func(file_path: str):
    gps_waypoints = []
    with open(file_path, 'r') as file:
        gps_data = yaml.safe_load(file)
    
    for waypoint_data in gps_data["waypoints"]:
        waypoint = GPSWaypoint()
        waypoint.name = waypoint_data.get("name", "")
        waypoint.position = waypoint_data.get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
        waypoint.quaternion = waypoint_data.get("quaternion", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
        waypoint.rotation_rads = waypoint_data.get("rotation_rads", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        gps_waypoints.append(waypoint)
    
    return gps_waypoints

def print_gps_waypoints(gps_waypoints):
    for waypoint in gps_waypoints:
        print(f"Name: {waypoint.name}")
        print(f"Position: x={waypoint.position['x']}, y={waypoint.position['y']}, z={waypoint.position['z']}")
        print(f"Quaternion: x={waypoint.quaternion['x']}, y={waypoint.quaternion['y']}, z={waypoint.quaternion['z']}, w={waypoint.quaternion['w']}")
        print(f"Rotation (rads): roll={waypoint.rotation_rads['roll']}, pitch={waypoint.rotation_rads['pitch']}, yaw={waypoint.rotation_rads['yaw']}")
        print("-" * 40)

def save_gps_waypoints_to_list(gps_waypoints):
    waypoint_list = []
    for waypoint in gps_waypoints:
        waypoint_list.append([waypoint.position['x'], waypoint.position['y'], waypoint.position['z']])
    return waypoint_list