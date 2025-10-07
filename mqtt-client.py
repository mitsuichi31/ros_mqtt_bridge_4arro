# ブローカーに定期的にオドメトリとバッテリーデータを送信し、
# 緊急停止状態もランダムに切り替えて送信
# また、cmd_velトピックを購読して、bridge-dated-log-refactored.pyから送られてくる速度目標値を受信

# 以下の通信がシミュレーションされます。
# クライアント mqtt-client.py → ブローカー mosquitto.py → ブリッジ bridge-dated-log-refactored.py → ROSダミー ros-dummy.py:
#   オドメトリ、バッテリー、緊急停止データが送信されます。
# ROSダミー → ブリッジ bridge-dated-log-refactored.py → ブローカー mosquitto → クライアント mqtt-client:
#   速度目標値データが送信されます。クライアントのコンソールに受信した速度が表示されます。

import paho.mqtt.client as mqtt
import json
import time
import random
import threading
import sys

# MQTTブローカーの設定
# bridge-dated-log-refactored.pyの設定と合わせる
# MQTT_BROKER_ADDRESS = "192.168.212.60"
# MQTT_PORT = 1883
MQTT_BROKER_ADDRESS = "192.168.212.1" # doog IPアドレス
MQTT_PORT = 1883
MQTT_USER = "mqtt"
MQTT_PASSWORD = "sI7G@DijuY"
ROBOT_ID = "RMS-XXXX-XXX"

# MQTTトピックの設定
TOPIC_ODOM = f"0/WHISPERER/{ROBOT_ID}/pos2D_DWO"
TOPIC_BATTERY = f"0/WHISPERER/{ROBOT_ID}/battery"
TOPIC_EMERGENCY = f"0/THOUZER_HW/{ROBOT_ID}/status/app"
TOPIC_CMD_VEL = f"0/WHISPERER/{ROBOT_ID}/nav"

# 接続時のコールバック
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully to MQTT Broker!")
        # cmd_velトピックを購読
        client.subscribe(TOPIC_CMD_VEL)
        print(f"Subscribed to topic: {TOPIC_CMD_VEL}")
    else:
        print(f"Failed to connect, return code {rc}")

# メッセージ受信時のコールバック
def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        print(f"Received from bridge on topic '{msg.topic}': {payload}")
        # 例: v_mps: "0.5", w_degps: "10.0"
        v_mps = float(payload.get("v_mps", 0.0))
        w_degps = float(payload.get("w_degps", 0.0))
        print(f"  - Linear velocity: {v_mps} m/s")
        print(f"  - Angular velocity: {w_degps} deg/s")
    except json.JSONDecodeError:
        print(f"Received non-JSON message on topic '{msg.topic}': {msg.payload.decode('utf-8')}")
    except Exception as e:
        print(f"An error occurred while processing message: {e}")

# データ送信のロジック
def publish_data(client):
    x_pos = 0.0
    yaw_deg = 0.0
    voltage = 24.0
    emergency_on = False

    while True:
        # オドメトリデータのシミュレーション
        x_pos += 0.01  # 前進をシミュレート
        yaw_deg += 0.5 # 回転をシミュレート
        odom_payload = {
            "x_m": x_pos,
            "y_m": 0.0,
            "yaw_deg": yaw_deg
        }
        client.publish(TOPIC_ODOM, json.dumps(odom_payload))

        # バッテリーデータのシミュレーション
        voltage -= 0.001 # 電圧降下をシミュレート
        battery_payload = {
            "voltage_v": voltage,
        }
        client.publish(TOPIC_BATTERY, json.dumps(battery_payload))

        # 緊急停止状態のシミュレーション（ランダムに切り替え）
        if random.random() < 0.05: # 5%の確率で状態を切り替え
            emergency_on = not emergency_on
            status = "#finishErrorEmergency" if emergency_on else "#finishSuccess"
            emergency_payload = {
                "app": status
            }
            client.publish(TOPIC_EMERGENCY, json.dumps(emergency_payload))

        print(f"Published odom, battery, and emergency status. (Emergency: {emergency_on})")
        time.sleep(1) # 1秒ごとにデータを送信

# メイン関数
def main():
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER_ADDRESS, MQTT_PORT, 60)
    except Exception as e:
        print(f"Could not connect to MQTT Broker: {e}")
        sys.exit(1)

    # データ送信を別のスレッドで実行
    publish_thread = threading.Thread(target=publish_data, args=(client,), daemon=True)
    publish_thread.start()

    # MQTTクライアントのループを開始
    client.loop_forever()


if __name__ == '__main__':
    main()