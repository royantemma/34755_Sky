import paho.mqtt.client as mqtt

class BaseNode:
    def __init__(self, node_name, broker_ip="127.0.0.1"):
        self.node_name = node_name
        self.client = mqtt.Client(client_id=node_name)
        self.client.on_message = self._on_message
        self.client.connect(broker_ip, 1883)
        
    def start(self):
        self.client.loop_start()

    def _on_message(self, client, userdata, msg):
        # To be overridden by child nodes
        pass

    def publish(self, topic, payload):
        self.client.publish(topic, payload)

def stop(self):
        print(f"[{self.node_name}] Stopping MQTT client...")
        if hasattr(self, 'client') and self.client is not None:
            # Disconnect immediately, do not wait for pending messages
            self.client.disconnect()
            # Force the background thread to exit without blocking
            self.client.loop_stop(force=True)