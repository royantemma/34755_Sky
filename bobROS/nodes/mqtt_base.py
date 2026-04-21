import paho.mqtt.client as mqtt
import time

class MQTTNode:
    def __init__(self, node_name, host="127.0.0.1", port=1883):
        self.node_name = node_name
        self.client = mqtt.Client(client_id=node_name)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.host = host
        self.port = port
        self.running = True

    def on_connect(self, client, userdata, flags, rc):
        print(f"[{self.node_name}] Connected to MQTT broker with result {rc}")

    def on_message(self, client, userdata, msg):
        pass

    def start(self):
        try:
            self.client.connect(self.host, self.port, 60)
            self.client.loop_start()
            self.run()
        except Exception as e:
            print(f"[{self.node_name}] Failed to start: {e}")

    def run(self):
        while self.running:
            time.sleep(1)

    def stop(self):
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()
        print(f"[{self.node_name}] Stopped.")

    def publish(self, topic, payload):
        self.client.publish(topic, payload)