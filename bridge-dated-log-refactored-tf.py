# ROS2とMQTT間の通信を仲介するノード
#
# パブリッシャとサブスクライバ
# ROS2パブリッシャ: MCUから受信したMQTTメッセージをROS2トピックとして公開します。
#   self.odom_pub: オドメトリデータ (nav_msgs/msg/Odometry) を/robot/odomトピックに公開します 。
#   self.battery_state_pub: バッテリーの状態 (sensor_msgs/msg/BatteryState) を/battery_stateトピックに公開します 。
#   self.emergency_button_status_pub: 緊急停止状態 (std_msgs/Bool) を/emergency_button_statusトピックに公開します 。
# ROS2サブスクライバー: NavstackからのROS2トピックを購読し、MQTTメッセージとしてMCUに送信します。
#   self.cmd_vel_sub: 速度目標値 (geometry_msgs/msg/Twist) を/robot/cmd_velトピックから購読します 。
#
# ログキュー: 通信スレッドがログデータをこのキューに追加します。
# ログスレッド: 独立したスレッドがキューからデータを取得し、タイムスタンプ付きでファイルに書き込みます。
# スレッドセーフティ: スレッド間で共有されるデータ（ログキュー）は、queueモジュールを使って安全に扱います。
#
# ログファイル名: communication_log_YYYY-MM-DD_HH-MM-SS.txtという形式で、ログファイル作成時の日時がファイル名に含まれます。
# ログファイルの管理:
#   ログワーカーが起動する際に、既存のログファイルをチェックします。
#   ログファイル名の日時を基に、最新の5つのファイルを除いて、それより古いファイルをすべて削除します。
# 非同期ロギング: 従来どおり、ログデータはqueueを通じて独立したログスレッドに渡され、I/O処理は非同期で行われます。
#

# Node to bridge communication between ROS2 and MQTT
#
# Publishers and Subscribers
# ROS2 Publishers: Publish MQTT messages received from the MCU as ROS2 topics.
#   self.odom_pub: Publishes odometry data (nav_msgs/msg/Odometry) to the /robot/odom topic.
#   self.battery_state_pub: Publishes battery state (sensor_msgs/msg/BatteryState) to the /battery_state topic.
#   self.emergency_button_status_pub: Publishes emergency stop status (std_msgs/Bool) to the /emergency_button_status topic.
# ROS2 Subscribers: Subscribe to ROS2 topics from Navstack and send them as MQTT messages to the MCU.
#   self.cmd_vel_sub: Subscribes to velocity commands (geometry_msgs/msg/Twist) from the /robot/cmd_vel topic.
#
# Log Queue: Communication threads add log data to this queue.
# Log Thread: A separate thread retrieves data from the queue and writes it to a file with a timestamp.
# Thread Safety: Data shared between threads (the log queue) is handled safely using the queue module.
#
# Log Filename: The log filename is formatted as communication_log_YYYY-MM-DD_HH-MM-SS.txt, including the date and time of its creation.
# Log File Management:
#   When the log worker starts, it checks for existing log files.
#   Based on the timestamp in the filenames, it deletes all but the 5 most recent files.
# Asynchronous Logging: As before, log data is passed to a separate logging thread via the queue, and I/O operations are performed asynchronously.

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
import json
from tf_transformations import quaternion_from_euler
import threading
import queue
import time
import datetime
import os
import glob
# --- TF関連のインポートを追加 ---
import tf2_ros
from geometry_msgs.msg import TransformStamped
# ----------------------------

# MQTT broker settings for communication with the MCU
# MQTT_BROKER_ADDRESS = "192.168.212.60"
# MQTT_PORT = 1883
# ROBOT_ID = "RMS-XXXX-XXX" # Replace with the actual robot ID
MQTT_BROKER_ADDRESS = "192.168.212.1"
MQTT_PORT = 1883
ROBOT_ID = "RMS-10E2-AAY34" # Replace with the actual robot ID
MQTT_USER = "mqtt"
MQTT_PASSWORD = "sI7G@DijuY"
LOG_FILE_DIRECTORY = "." # Directory where logs will be stored. Can be changed.
MAX_LOG_FILES = 5

# --- ★★★ 追加: MQTT送信間隔（秒）★★★ ---
MQTT_SEND_INTERVAL = 0.02  # 20ミリ秒
# 
FORCE_DISABLE_INTERVAL = 2.0 # 2 seconds

class McuNavstackBridge(Node):
    """
    ROS2 node to bridge communication between an MCU and Navstack.
    """
    def __init__(self):
        super().__init__('mcu_navstack_bridge_node')
        self.get_logger().info("Starting MCU-Navstack Bridge Node")

        # ★★★ 追加: ロギング制御フラグ ★★★
        self.logging_enabled = False # True: ロギングを実行, False: ロギングをスキップ

        # Log queue and thread setup
        self.log_queue = queue.Queue()
        self.log_thread_stop_event = threading.Event()
        
        # Ensure log directory exists
        try:
            os.makedirs(LOG_FILE_DIRECTORY, exist_ok=True)
        except OSError as e:
            self.get_logger().error(f"Failed to create log directory {LOG_FILE_DIRECTORY}: {e}")
            raise # Critical error, cannot log

        self.log_thread = threading.Thread(target=self.log_worker)
        self.log_thread.start()
        self.get_logger().info("Log worker thread started.")

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        try:
            self.mqtt_client.connect(MQTT_BROKER_ADDRESS, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("MQTT client started successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT Broker at startup: {e}")
            # Depending on requirements, you might want to exit or retry
            raise # Re-raise the exception to prevent the node from running without MQTT

        # ROS2 Publishers (for topics the MCU publishes)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)
        self.battery_state_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.emergency_button_status_pub = self.create_publisher(Bool, '/emergency_button_status', 10)

        # ROS2 Subscribers (for topics the MCU subscribes to)
        self.cmd_vel_sub = self.create_subscription(Twist, '/robot/cmd_vel', self.twist_callback, 10)

        # --- TF Broadcasterの初期化 ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.get_logger().info("TF Broadcasters initialized.")
        
        # 静的座標変換を公開（Lidarフレームを含む）
        self.publish_static_transforms()
        # ----------------------------

        # --- ★★★ 修正: 状態変数とタイムアウト処理の変更 ★★★ ---
        self.external_control_enabled = False
        self.last_mqtt_send_time = 0.0
        self.last_twist_time = 0.0  # 最後にTwistメッセージを受信した時刻
        self.timeout_check_timer = self.create_timer(0.5, self.check_timeout_callback) # 0.5秒ごとにタイムアウトをチェック


    # --------------------------------------------------------------------
    # 新規メソッド: 静的座標変換 (Static TF) の公開
    # base_link、base_footprint、およびLidarフレームを定義します。
    # --------------------------------------------------------------------
    def publish_static_transforms(self):
        # (x, y, z, roll_deg, pitch_deg, yaw_deg)
        # Lidarは通常、base_linkからの静的なオフセットを持つ。
        LIDAR_CONFIG = {
            # Lidar 1: Helios32 (top)
            'rslidar_1': ( 0.4605, 0.0,   1.6850,   0.0,   0.0,   0.0),
            # Lidar 2: Airy (front)
            'rslidar_2': ( 0.5574, 0.0,   1.5985,   0.0,  90.0,   0.0),
            # Lidar 3: Airy (left)
            'rslidar_3': ( 0.1975, 0.097, 1.5985, -90.0,   0.0,  90.0),
            # Lidar 4: Airy (right)
            'rslidar_4': ( 0.1975, 0.097, 1.5985,  90.0,   0.0, -90.0),
            # Lidar 5: Airy (back)
            'rslidar_5': (-0.1624, 0.0,   1.5985,   0.0, -90.0,   0.0), 
        }

        static_transforms = []

        # --- 標準的な静的TF: base_link -> base_footprint ---
        # base_footprintはbase_linkと同じだがZ軸が地面上にあるフレーム。
        t_footprint = TransformStamped()
        t_footprint.header.stamp = self.get_clock().now().to_msg()
        t_footprint.header.frame_id = 'base_link'
        t_footprint.child_frame_id = 'base_footprint'
        t_footprint.transform.translation.x = 0.0
        t_footprint.transform.translation.y = 0.0
        # base_linkが地面から0.1m浮いていると仮定して、base_footprintを地面(Z=0)に設定
        t_footprint.transform.translation.z = -0.1125
        q_footprint = quaternion_from_euler(0, 0, 0)
        t_footprint.transform.rotation.x = q_footprint[0]
        t_footprint.transform.rotation.y = q_footprint[1]
        t_footprint.transform.rotation.z = q_footprint[2]
        t_footprint.transform.rotation.w = q_footprint[3]
        static_transforms.append(t_footprint)
        # --- Lidar Framesの追加 (rslidar_1からrslidar_5) ---
        
        for frame_id, (x, y, z, roll_deg, pitch_deg, yaw_deg) in LIDAR_CONFIG.items():
            t_lidar = TransformStamped()
            t_lidar.header.stamp = self.get_clock().now().to_msg()
            t_lidar.header.frame_id = 'base_link' # base_linkに対する相対位置
            t_lidar.child_frame_id = frame_id
            
            t_lidar.transform.translation.x = x
            t_lidar.transform.translation.y = y
            t_lidar.transform.translation.z = z
            
            # 角度をラジアンに変換
            q_lidar = quaternion_from_euler(
                roll_deg * 3.14159 / 180.0, 
                pitch_deg * 3.14159 / 180.0, 
                yaw_deg * 3.14159 / 180.0
            )
            t_lidar.transform.rotation.x = q_lidar[0]
            t_lidar.transform.rotation.y = q_lidar[1]
            t_lidar.transform.rotation.z = q_lidar[2]
            t_lidar.transform.rotation.w = q_lidar[3]
            static_transforms.append(t_lidar)

        # 静的変換を一度にブロードキャスト
        self.static_tf_broadcaster.sendTransform(static_transforms)
        self.get_logger().info(f'Published static transforms for base_footprint and {len(LIDAR_CONFIG)} Lidar frames to /static_tf.')



    def manage_log_files(self):
        """Deletes old log files, keeping only the latest MAX_LOG_FILES."""
        path_pattern = os.path.join(LOG_FILE_DIRECTORY, "communication_log_*.txt")
        # log_files = sorted(glob.glob(path_pattern), key=os.path.getmtime, reverse=True)
        # ★★★ ファイル名を直接ソートするように変更 ★★★
        # `key`引数を削除し、ファイルパスの文字列でソートします。
        # ファイル名の日時フォーマット(YYYY-MM-DD_HH-MM-SS)は、文字列としてソートしても時系列順になる
        log_files = sorted(glob.glob(path_pattern), reverse=True)
        
        if len(log_files) > MAX_LOG_FILES:
            files_to_delete = log_files[MAX_LOG_FILES:]
            self.get_logger().info(f"Found {len(files_to_delete)} old log files to delete.")
            # for old_file in log_files[MAX_LOG_FILES:]:
            for old_file in files_to_delete:
                try:
                    os.remove(old_file)
                    self.get_logger().info(f"Deleted old log file: {old_file}")
                except OSError as e:
                    self.get_logger().error(f"Error deleting file {old_file}: {e}")

    def enable_logging(self, enable: bool):
        """
        Dynamically enables or disables file logging.
        """
        self.logging_enabled = enable
        state = "Enabled" if enable else "Disabled"
        self.get_logger().info(f"File logging has been {state}.")

    def log_message(self, direction, topic, data):
        """Adds a log entry to the queue."""

        # ★★★ 修正: ロギングが有効でない場合は処理をスキップ ★★★
        if not self.logging_enabled:
            return
        
        timestamp = datetime.datetime.now().isoformat()
        # Ensure data is JSON serializable if it's an object/dict for consistent logging
        try:
            data_str = json.dumps(data)
        except TypeError:
            data_str = str(data) # Fallback to string representation

        log_entry = f"[{timestamp}] [{direction}] Topic: {topic}, Data: {data_str}\n"
        self.log_queue.put(log_entry)

    def log_worker(self):
        """Worker function for the log thread."""
        self.manage_log_files() # Check and delete old files at startup
        
        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_filename = os.path.join(LOG_FILE_DIRECTORY, f"communication_log_{timestamp_str}.txt")
        
        try:
            with open(log_filename, "a") as log_file:
                self.get_logger().info(f"Writing logs to: {log_filename}")
                while not self.log_thread_stop_event.is_set():
                    try:
                        log_entry = self.log_queue.get(timeout=1)
                        log_file.write(log_entry)
                        log_file.flush() # Ensure data is written to disk
                    except queue.Empty:
                        continue
                    except Exception as e: # Catch any other writing errors
                        self.get_logger().error(f"Error writing to log file {log_filename}: {e}")
            self.get_logger().info("Log worker thread stopped.")
        except IOError as e:
            self.get_logger().error(f"Failed to open log file {log_filename}: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred in log worker: {e}")


    def on_connect(self, client, userdata, flags, rc):
        """Callback function for MQTT connection."""
        if rc == 0:
            self.get_logger().info("Successfully connected to MQTT Broker!")
            self.log_message("INFO", "MQTT", "Successfully connected to MQTT Broker!")
            # Subscribe to MCU status topics
            client.subscribe(f"0/WHISPERER/{ROBOT_ID}/pos2D_DWO")
            client.subscribe(f"0/WHISPERER/{ROBOT_ID}/battery")
            client.subscribe(f"0/THOUZER_HW/{ROBOT_ID}/status/app")
        else:
            self.get_logger().error(f"Failed to connect to MQTT Broker, return code {rc}")
            self.log_message("ERROR", "MQTT", f"Failed to connect to MQTT Broker, return code {rc}")

    def on_message(self, client, userdata, msg):
        """Callback function for incoming MQTT messages."""

        # Uncomment for debugging
        # self.get_logger().info(f"Received MQTT message on topic {msg.topic}")
        
        data = {}
        try:
            data = json.loads(msg.payload.decode())
            self.log_message("RECV", msg.topic, data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON from MQTT message on topic {msg.topic}: {e}. Payload: {msg.payload.decode()}")
            self.log_message("ERROR", msg.topic, f"JSON Decode Error: {e}")
            return # Skip processing if JSON is invalid
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Failed to decode payload as UTF-8 on topic {msg.topic}: {e}. Payload: {msg.payload}")
            self.log_message("ERROR", msg.topic, f"Unicode Decode Error: {e}")
            return # Skip processing if payload cannot be decoded

        # Handle Odometry data from MQTT and publish to ROS2
        if f"0/WHISPERER/{ROBOT_ID}/pos2D_DWO" in msg.topic:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            try:
                odom_msg.pose.pose.position.x = float(data.get("x_m", 0.0))
                odom_msg.pose.pose.position.y = float(data.get("y_m", 0.0))
                yaw_deg = float(data.get("yaw_deg", 0.0))
                q = quaternion_from_euler(0, 0, yaw_deg * 3.14159 / 180.0)
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]
                self.odom_pub.publish(odom_msg)

                # --- Dynamic TF Broadcaster (odom -> base_link) の追加 ---
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link' # Odometryメッセージのchild_frame_idと一致させる [17]

                # 位置と姿勢をOdometryメッセージからコピー
                t.transform.translation.x = odom_msg.pose.pose.position.x
                t.transform.translation.y = odom_msg.pose.pose.position.y
                t.transform.translation.z = 0.0 # 2Dオドメトリでは通常Z=0
                t.transform.rotation = odom_msg.pose.pose.orientation

                # 動的変換をブロードキャスト
                self.tf_broadcaster.sendTransform(t)
                # --------------------------------------------------------

            except ValueError as e:
                self.get_logger().error(f"Odometry data conversion error: {e}. Data: {data}")
                self.log_message("ERROR", msg.topic, f"Data conversion error: {e}")
            except TypeError as e: # Catch cases where get() returns non-string for float conversion
                self.get_logger().error(f"Odometry data type error: {e}. Data: {data}")
                self.log_message("ERROR", msg.topic, f"Data type error: {e}")
            
        # Handle Battery State from MQTT and publish to ROS2
        elif f"0/WHISPERER/{ROBOT_ID}/battery" in msg.topic:
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            try:
                battery_msg.voltage = float(data.get("voltage_v", 0.0))
                self.battery_state_pub.publish(battery_msg)
            except ValueError as e:
                self.get_logger().error(f"Battery voltage data conversion error: {e}. Data: {data}")
                self.log_message("ERROR", msg.topic, f"Voltage conversion error: {e}")
            except TypeError as e:
                self.get_logger().error(f"Battery voltage data type error: {e}. Data: {data}")
                self.log_message("ERROR", msg.topic, f"Voltage type error: {e}")
        
        # Handle Emergency Stop status from MQTT
        elif f"0/THOUZER_HW/{ROBOT_ID}/status/app" in msg.topic:
            # `data.get("app", "")` is generally safe as it returns an empty string if "app" is missing
            is_emergency = "#finishErrorEmergency[BL10-SL1]" in data.get("app", "")
            emergency_msg = Bool()
            emergency_msg.data = is_emergency
            self.emergency_button_status_pub.publish(emergency_msg)

    def external_control(self, enable):
        """
        Enable or disable external control mode on the MCU via MQTT.
        """
        command = "app-whisperer" if enable else ""
        mqtt_topic = f"0/THOUZER_HW/{ROBOT_ID}/exec/cmd"
        mqtt_payload = {"app": command}
        try:
            # 状態を更新
            if enable:
                self.get_logger().info("Enabling external control mode.")
                self.external_control_enabled = True
            else:
                self.get_logger().info("Disabling external control mode due to timeout.")
                self.external_control_enabled = False
                self.disable_timer = None # タイマーをクリア

            self.mqtt_client.publish(mqtt_topic, json.dumps(mqtt_payload))
            self.log_message("SEND", mqtt_topic, mqtt_payload)
            self.get_logger().info(f"Published external control command to MQTT topic: {mqtt_topic} (enable={enable})")
        except Exception as e:
            self.get_logger().error(f"Failed to publish MQTT message to {mqtt_topic}: {e}")
            self.log_message("ERROR", mqtt_topic, f"MQTT publish error: {e}")

    def check_timeout_callback(self):
        """
        0.5秒ごとに呼び出され、Twistメッセージの受信が2秒以上途絶えたかを監視する。
        """
        # 外部制御が有効な場合のみチェックを実行
        if self.external_control_enabled:
            current_time = self.get_clock().now().nanoseconds / 1e9
            # 最後にメッセージを受信してから2秒以上経過していたら無効化
            if (current_time - self.last_twist_time) > FORCE_DISABLE_INTERVAL:
                self.get_logger().info("Control timed out. Disabling external control.")
                self.external_control(enable=False)

    def twist_callback(self, twist_msg):
        """
        /robot/cmd_vel トピックのコールバック。
        - 最終受信時刻を更新し、必要であれば外部制御を有効化する。
        - 速度指令のMQTT送信は20ms間隔に間引く。
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.last_twist_time = current_time  # 常に最終受信時刻を更新

        # --- 外部制御の有効化 ---
        if not self.external_control_enabled:
            self.get_logger().info("Control starting: Sending enable command.")
            self.external_control(enable=True)

        # --- 速度指令の間引き処理 ---
        if (current_time - self.last_mqtt_send_time) < MQTT_SEND_INTERVAL:
            return

        # --- 間引き後のMQTT送信処理 ---
        self.last_mqtt_send_time = current_time

        self.get_logger().info("Sending throttled Twist message to MQTT.")
        
        v_mps = twist_msg.linear.x
        w_degps = twist_msg.angular.z * 180 / 3.14159

        mqtt_payload = {
            "v_mps": f"{v_mps}",
            "w_degps": f"{w_degps}"
        }
        
        mqtt_topic = f"0/WHISPERER/{ROBOT_ID}/nav"
        
        try:
            self.mqtt_client.publish(mqtt_topic, json.dumps(mqtt_payload))
            self.log_message("SEND", mqtt_topic, mqtt_payload)
        except Exception as e:
            self.get_logger().error(f"Failed to publish MQTT message: {e}")

        def on_shutdown(self):
            """Stops the log worker thread cleanly."""
            self.log_thread_stop_event.set()
            self.log_thread.join()
            self.get_logger().info("Log worker thread shut down.")
            # Optionally, disconnect MQTT client cleanly on shutdown
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

def main(args=None):
    rclpy.init(args=args)
    mcu_navstack_bridge = None # Initialize to None
    try:
        mcu_navstack_bridge = McuNavstackBridge()
        rclpy.spin(mcu_navstack_bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if mcu_navstack_bridge:
            mcu_navstack_bridge.get_logger().fatal(f"A fatal error occurred: {e}")
        else:
            print(f"A fatal error occurred during initialization: {e}")
    finally:
        if mcu_navstack_bridge:
            mcu_navstack_bridge.on_shutdown()
            mcu_navstack_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()