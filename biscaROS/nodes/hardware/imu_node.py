# biscaROS/nodes/hardware/imu_node.py
import time
from biscaROS.core.base_node import BaseNode

class IMUNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="imu_sensor_node")
        
        # Subscribe to the raw teensy topics
        self.client.subscribe("robobot/drive/T0/gyro")
        self.client.subscribe("robobot/drive/T0/acc")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        gg = payload.split(" ")
        
        if len(gg) < 4:
            return

        try:
            # Extract timestamp from payload if downstream nodes need it
            timestamp = float(gg[0])
            val_x, val_y, val_z = float(gg[1]), float(gg[2]), float(gg[3])
            
            if topic.endswith("T0/gyro"):
                # Publish standardized, clean data for navigation.py
                self.publish("biscaROS/sensors/imu/gyro", f"{timestamp},{val_x},{val_y},{val_z}")
            
            elif topic.endswith("T0/acc"):
                # Publish standardized, clean data for navigation.py
                self.publish("biscaROS/sensors/imu/acc", f"{timestamp},{val_x},{val_y},{val_z}")
                
        except ValueError:
            # Malformed data string from serial
            pass

if __name__ == "__main__":
    node = IMUNode()
    node.start()
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.stop()