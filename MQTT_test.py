import paho.mqtt.client as mqtt
import time

BROKER_ADDRESS = "192.168.2.118"
BROKER_PORT = 1883
USERNAME = "admin"
PASSWORD = "TS2018-mqtt"
TEST_TOPIC = "SPM/UPS/test"
TEST_MESSAGE = '{"test": "hello from mqtt_test"}'

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}")

client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect

try:
    client.connect(BROKER_ADDRESS, BROKER_PORT)
except Exception as e:
    print(f"Connection failed: {e}")
    exit(1)

client.loop_start()
time.sleep(1)  # wait for connection
result = client.publish(TEST_TOPIC, TEST_MESSAGE)

status = result[0]
if status == 0:
    print(f"Sent `{TEST_MESSAGE}` to topic `{TEST_TOPIC}`")
else:
    print(f"Failed to send message to topic {TEST_TOPIC}")

time.sleep(2)  # wait to ensure message is sent
client.loop_stop()
client.disconnect()
