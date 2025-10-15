# Python Script: MCU-Navstack Bridge Node Documentation

## 1. Overview

This script operates as a **ROS 2 node** (`mcu_navstack_bridge_node`) designed to mediate the real-time, bi-directional exchange of critical robot data. Its primary role is bridging communication between a **high-level ROS 2 environment** (such as the Navstack) and the **low-level MCU** (Microcontroller Unit), which communicates using the **MQTT protocol**. This separation allows ROS 2 to handle advanced navigation functions while the MCU manages actual hardware control, resulting in a robust system integration.

---

## 2. Main Functions and Operating Principles

The bridge node fulfills three major roles in the system:

### 2.1. Bi-directional Data Relay (MQTT $\rightleftharpoons$ ROS 2)

The node acts as both a **ROS 2 Publisher** (for MCU data) and a **ROS 2 Subscriber** (for command data).

| Data Flow | Source/Destination | Topic/Message Type | Processing/Action | Purpose |
| :---: | :---: | :---: | :--- | :--- |
| MCU $\to$ ROS 2 | MQTT | Odometry (`pos2D_DWO`) | Extracts position data (`x_m`, `y_m`, `yaw_deg`) from the JSON payload and publishes it to the ROS 2 `/robot/odom` topic (`nav_msgs/msg/Odometry`). | Provides the robot's position and orientation data to the navigation stack. |
| MCU $\to$ ROS 2 | MQTT | Battery Information (`battery`) | Extracts voltage (`voltage_v`) from the JSON payload and publishes it to the ROS 2 `/battery_state` topic (`sensor_msgs/msg/BatteryState`). | Provides the robot's energy status to monitoring systems. |
| MCU $\to$ ROS 2 | MQTT | Emergency Stop Status (`status/app`) | Checks the received message for a specific emergency string (`#finishErrorEmergency[BL10-SL1]`) and publishes the status to `/emergency_button_status` (`std_msgs/Bool`). | Transmits the state of the emergency stop across the entire ROS ecosystem. |
| ROS 2 $\to$ MCU | ROS 2 | Velocity Command (`/robot/cmd_vel`) | Subscribes to speed commands (`geometry_msgs/msg/Twist`) from the Navstack, converts the linear (X) and angular (Z) velocities into a JSON payload, and publishes it to the MCU's MQTT topic (`.../nav`). | Executes movement plans commanded by ROS on the physical robot. |

### 2.2. TF (Coordinate Transformation) Publication

The node manages two types of coordinate transformations:

* **Static TF (Static Transform)**: Upon startup, the node calls `publish_static_transforms` to publish predefined transformations for frames like `base_link`, `base_footprint`, and Lidar sensors to the `/tf_static` topic. Lidar frame offsets (e.g., `rslidar_1`, `rslidar_2`) are specified using X, Y, Z, Roll, Pitch, and Yaw (degrees).
* **Dynamic TF**: Every time an odometry message is received, the node calculates and broadcasts the dynamic coordinate transformation from the `odom` frame to the `base_link` frame.

### 2.3. External Control Safety Management (Twist Timer)

The `twist_callback` includes a crucial security feature: a **2.0-second safety timer**:

1.  When a Twist message is received, the node first sends an MQTT command to enable external control mode on the MCU (if it is not already enabled).
2.  The 2.0-second timer (`disable_timer`) is reset.
3.  If a subsequent Twist message is not received within the 2.0 second limit, the timer callback (`disable_external_control_callback`) executes, sending an MQTT command to disable external control mode on the MCU.
4.  This mechanism ensures that the autonomous control safely halts if communication is interrupted or the controlling ROS node crashes, thereby preventing the robot from running away.

### 2.4. Asynchronous Logging

The script implements asynchronous I/O handling for logging using a dedicated **log worker thread**:

* Received and transmitted MQTT data are added to a **thread-safe queue** (`log_queue`).
* The independent log worker **thread** retrieves data from this queue and writes it securely to a file.
* **Log File Management**: Upon startup, the log worker checks for existing log files. It automatically deletes older files, ensuring only the **latest 5 log files** (`MAX_LOG_FILES = 5`) are retained.
* Log files are created using a dated format: `communication_log_YYYY-MM-DD_HH-MM-SS.txt`.

---

## 3. Usage Instructions

### 3.1. Prerequisites

The script requires:

* A running **ROS 2 environment** (using Python's `rclpy`).
* An accessible **MQTT broker** (using `paho.mqtt.client`).

### 3.2. Required Configuration

The connection parameters must be confirmed and set within the script:

| Constant | Example Value | Description |
| :--- | :--- | :--- |
| `MQTT_BROKER_ADDRESS` | `192.168.212.1` | The IP address of the MQTT broker. |
| `ROBOT_ID` | `RMS-10E2-AAY34` | The unique identifier for the specific robot. |
| `MQTT_USER`, `MQTT_PASSWORD` | `mqtt`, `sI7G@DijuY` | Authentication credentials for connecting to the MQTT broker. |

### 3.3. Execution and Shutdown

1.  **Startup**: The `main` function should be executed within the ROS 2 environment.
2.  **Verification**: The node and its advertised topics can be verified using ROS 2 command-line tools (e.g., `ros2 node list` and `ros2 topic list`). The node is named `/mcu_navstack_bridge_node`.
3.  **Shutdown**: The script is designed to shut down safely during normal termination or interrupts (e.g., `KeyboardInterrupt` via Ctrl+C). It ensures the log worker **thread** is stopped cleanly and the MQTT client is disconnected.

### 3.4. Key Data Structures

| ROS 2 Topic | Type | Description | Data Source |
| :--- | :--- | :--- | :--- |
| `/robot/odom` | `nav_msgs/msg/Odometry` | MCU odometry data (`pos2D_DWO`) | MQTT (MCU) |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Battery voltage (`voltage_v`) | MQTT (MCU) |
| `/emergency_button_status` | `std_msgs/Bool` | Emergency stop flag | MQTT (MCU) |
| `/robot/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands (Linear X, Angular Z) | ROS 2 (Navstack, etc.) |
| `/tf_static` | `geometry_msgs/msg/TransformStamped` | Fixed sensor coordinate transformations (e.g., Lidar) | Internal configuration |

---

## 4. Communication Frequency and Real-time Requirements

The operational frequency of the bridge node depends on the incoming MCU data and the required rate for command reception.

### 4.1. MCU Data Reception Frequency (MQTT Subscribe)

Analysis of the logs indicates that the primary data sent from the MCU (Odometry and Battery information) is transmitted at a stable frequency:

* **Odometry** (`../pos2D_DWO`): **Approximately 1.0 Hz**. Data is received roughly every one second.
* **Battery Status** (`.../battery`): **Approximately 1.0 Hz**. This data is received nearly simultaneously with the odometry data.

Consequently, the ROS 2 topics `/robot/odom` and `/battery_state` are updated at **approximately 1.0 Hz**.

### 4.2. ROS 2 Command Frequency (MQTT Publish)

The frequency of receiving velocity commands (`/robot/cmd_vel`) is critical for maintaining external control stability.

* **Requirement for Control Maintenance**: Due to the internal safety timer, continuous robot operation requires successive speed command messages to arrive within the **2.0 second time limit**.
* **Safety Implication**: To ensure continuous external control, the navigation stack must publish messages to `/robot/cmd_vel` at a period shorter than 2.0 seconds (i.e., at a **frequency higher than 0.5 Hz**).