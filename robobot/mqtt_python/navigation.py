import time
import urllib.request
import json
import numpy as np

class Navigator:
    def __init__(self, host="127.0.0.1", port=7123):
        self.base_url = f"http://{host}:{port}"
        self.robot_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        
    def fetch_data(self):
        try:
            req = urllib.request.Request(f"{self.base_url}/api/aruco/data")
            with urllib.request.urlopen(req) as response:
                data = json.loads(response.read())
                if data.get("enabled"):
                    return data
                return None
        except Exception:
            return None

    def run(self):
        print("% Starting Navigator")
        while True:
            data = self.fetch_data()
            if data and data.get("estimates"):
                estimates = data["estimates"]
                if len(estimates) > 0:
                    xs = [e["robot_x"] for e in estimates]
                    ys = [e["robot_y"] for e in estimates]
                    yaws = [e["robot_yaw"] for e in estimates]
                    
                    # Average pose
                    avg_x = sum(xs) / len(xs)
                    avg_y = sum(ys) / len(ys)
                    # For yaw, simple average might wrap around incorrectly, but okay for naive solution
                    avg_yaw = np.arctan2(np.sum(np.sin(yaws)), np.sum(np.cos(yaws)))
                    
                    self.robot_pose = {"x": avg_x, "y": avg_y, "z": 0, "yaw": avg_yaw}
                    print(f"Pose Est: x={avg_x:.1f}, y={avg_y:.1f}, yaw={np.degrees(avg_yaw):.1f} deg from {len(estimates)} markers")
                    
                    # Optionally, write this pose to a file or a shared memory object, or mqtt topic
                    # so that the stream_server can serve it to the frontend.
                    try:
                        with open("/tmp/robot_pose.json", "w") as f:
                            json.dump(self.robot_pose, f)
                    except:
                        pass
                        
            time.sleep(0.1)

if __name__ == "__main__":
    nav = Navigator()
    nav.run()
