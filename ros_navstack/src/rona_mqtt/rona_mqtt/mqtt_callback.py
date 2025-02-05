import json
from paho.mqtt.client import Client

class MqttCallback:
    def __init__(self, mqtt_waypoint_receiver):
        self.mqtt_waypoint_receiver = mqtt_waypoint_receiver

    def on_connect(self, client, userdata, flags, rc):
        self.mqtt_waypoint_receiver.get_logger().info("Connected to MQTT Broker")
        client.subscribe(self.mqtt_waypoint_receiver._topic, 1)

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.mqtt_waypoint_receiver.mqtt_callback(data)
