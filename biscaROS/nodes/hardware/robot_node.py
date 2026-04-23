# biscaROS/nodes/hardware/robot_node.py
import time
from datetime import datetime
from biscaROS.core.base_node import BaseNode

class RobotNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="robot_status_node")
        
        self.hbtTime = datetime.now()
        self.hbtInterval = 30.0
        self.hbtUpdCnt = 0
        self.robotName = "Sky"
        self.batVolt = 1.0
        
        # Subscribe to relevant hardware topics
        self.client.subscribe("robobot/drive/T0/hbt")
        self.client.subscribe("robobot/drive/T0/mot")
        self.client.subscribe("robobot/drive/T0/dname")
        self.client.subscribe("robobot/drive/T0/current")
        self.client.subscribe("robobot/drive/T0/mca")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        gg = payload.split(" ")
        
        if topic.endswith("T0/hbt") and len(gg) >= 4:
            try:
                t0 = self.hbtTime
                self.hbtTime = datetime.fromtimestamp(float(gg[0]))
                t1 = self.hbtTime
                
                if self.hbtUpdCnt == 2:
                    self.hbtInterval = (t1 - t0).total_seconds()
                elif self.hbtUpdCnt > 2:
                    self.hbtInterval = (self.hbtInterval * 99 + (t1 - t0).total_seconds()) / 100
                
                self.hbtUpdCnt += 1
                
                self.publish("biscaROS/system/heartbeat", f"{self.hbtTime.timestamp()},{self.hbtInterval}")
            except ValueError:
                pass
                
        elif topic.endswith("T0/dname") and len(gg) >= 2:
            self.robotName = gg[1]
            self.publish("biscaROS/system/robot_name", self.robotName)
            
        # Placeholder for motor and current telemetry, matching the incomplete state in srobot.py
        elif topic.endswith("T0/mot") and len(gg) >= 4:
            pass 
            
        elif topic.endswith("T0/current") and len(gg) >= 4:
            pass

if __name__ == "__main__":
    node = RobotNode()
    node.start()
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.stop()