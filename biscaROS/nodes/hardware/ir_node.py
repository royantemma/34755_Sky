import time
from datetime import datetime
from biscaROS.core.base_node import BaseNode

class IRNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="ir_sensor_node")
        # Existing state variables from sir.py
        self.ir = [0, 0]
        self.irTime = datetime.now()
        
        # Subscribe directly to the raw teensy topic
        self.client.subscribe("robobot/drive/T0/ir")
        self.client.subscribe("robobot/drive/T0/ird")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        
        # Logic from sir.py
        if topic.endswith("T0/ir") or topic.endswith("T0/ird"):
            gg = payload.split(" ")
            if len(gg) >= 3:
                self.ir[0] = float(gg[1])
                self.ir[1] = float(gg[2])
                
                # NEW: Instead of holding state globally, broadcast the parsed data
                # for the navigation node to use.
                self.publish("biscaROS/sensors/ir", f"{self.ir[0]},{self.ir[1]}")

if __name__ == "__main__":
    node = IRNode()
    node.start()
    
    # Keep the process alive
    while True:
        time.sleep(1)