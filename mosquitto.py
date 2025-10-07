# mosquittoをバックグラウンドプロセスとして起動する方法
#
# パスワードファイルの自動生成: mosquitto_passwd コマンドを使用して、
#    スクリプト実行時に認証に必要なパスワードファイル（mosquitto_passwd）を自動的に作成
# 設定ファイルの動的生成: 実行時にmosquitto.confファイルを作成し、
#    mosquittoがリスニングするIPアドレスとポートを設定

import subprocess
import time
import os
import signal
import atexit
import shutil

# MQTTブローカーの設定
MQTT_BROKER_ADDRESS = "192.168.212.60"
MQTT_PORT = 1883
MQTT_USER = "mqtt"
MQTT_PASSWORD = "sI7G@DijuY"
CONFIG_FILE = "mosquitto.conf"

def create_mosquitto_config():
    """
    mosquitto.confファイルを作成する関数
    """
    # 認証用のパスワードファイルを作成
    password_file_path = "mosquitto_passwd"
    if not os.path.exists(password_file_path):
        try:
            # -cオプションで新規作成、-bオプションで非対話モードでパスワードを設定
            subprocess.run(["mosquitto_passwd", "-c", "-b", password_file_path, MQTT_USER, MQTT_PASSWORD], check=True)
            print(f"Created password file: {password_file_path}")
        except FileNotFoundError:
            print("Warning: 'mosquitto_passwd' command not found. Authentication will be disabled.")
            password_file_path = None
        except subprocess.CalledProcessError:
            print("Error: Failed to create password file.")
            password_file_path = None
    
    # 設定ファイルの内容
    config_lines = [
        f"listener {MQTT_PORT} {MQTT_BROKER_ADDRESS}",
        "allow_anonymous false"
    ]
    if password_file_path:
        config_lines.append(f"password_file {os.path.abspath(password_file_path)}")
    else:
        # パスワードファイルが作成できなかった場合は匿名アクセスを許可
        config_lines = [
            f"listener {MQTT_PORT} {MQTT_BROKER_ADDRESS}",
            "allow_anonymous true"
        ]

    with open(CONFIG_FILE, "w") as f:
        f.write("\n".join(config_lines))
    print(f"Created mosquitto configuration file: {CONFIG_FILE}")
    
    return password_file_path

def start_mosquitto_broker():
    """
    mosquittoブローカーをsubprocessとして起動する関数
    """
    global mosquitto_process
    
    password_file = create_mosquitto_config()

    try:
        mosquitto_process = subprocess.Popen(
            ["mosquitto", "-c", CONFIG_FILE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        print("Starting mosquitto broker...")
        
        time.sleep(2)
        
        if mosquitto_process.poll() is not None:
            print("Error: Mosquitto broker failed to start.")
            stdout, stderr = mosquitto_process.communicate()
            print(f"Stdout: {stdout}")
            print(f"Stderr: {stderr}")
            mosquitto_process = None
            return False
            
        print(f"Mosquitto broker started on {MQTT_BROKER_ADDRESS}:{MQTT_PORT}")
        print("Press Ctrl+C to stop the broker.")
        return True

    except FileNotFoundError:
        print("Error: 'mosquitto' command not found. Please install Mosquitto.")
        return False
    except Exception as e:
        print(f"An unexpected error occurred while starting Mosquitto: {e}")
        return False

def stop_mosquitto_broker():
    """
    mosquittoブローカープロセスを終了する関数
    """
    global mosquitto_process
    if mosquitto_process and mosquitto_process.poll() is None:
        print("Stopping mosquitto broker...")
        mosquitto_process.terminate()
        mosquitto_process.wait()
        print("Mosquitto broker stopped.")
    
    # 終了時に作成したファイルを削除
    if os.path.exists(CONFIG_FILE):
        os.remove(CONFIG_FILE)
        print(f"Removed configuration file: {CONFIG_FILE}")
    if os.path.exists("mosquitto_passwd"):
        os.remove("mosquitto_passwd")
        print("Removed password file: mosquitto_passwd")

if __name__ == "__main__":
    mosquitto_process = None
    atexit.register(stop_mosquitto_broker)

    if start_mosquitto_broker():
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nReceived keyboard interrupt.")