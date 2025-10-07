# このスクリプトは、以下のことを行います。
# 速度目標値の公開: Twistメッセージを/robot/cmd_velトピックに定期的に公開し、
#   ブリッジスクリプトがこれを受信してMQTT経由でMCUに転送することを確認します。
# オドメトリの購読: /robot/odomトピックからOdometryメッセージを購読し、
#   MCUからMQTT経由でブリッジスクリプトに送られたデータが正しくROS2に公開されていることを確認します。
# バッテリー状態の購読: /battery_stateトピックからBatteryStateメッセージを購読し、
#   データが正しく公開されていることを確認します。
# 非常停止状態の購読: /emergency_button_statusトピックからBoolメッセージを購読し、
#   データが正しく公開されていることを確認します。

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

class RosTestNode(Node):
    """
    ROS2 node to test the MCU-Navstack bridge script.
    """
    def __init__(self):
        super().__init__('ros_test_node')
        self.get_logger().info("Starting ROS Test Node")

        # Create a publisher for sending velocity commands to the bridge script
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd_vel) # Publish every 1 second

        # Create subscribers to receive data from the bridge script
        self.create_subscription(Odometry, '/robot/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(Bool, '/emergency_button_status', self.emergency_callback, 10)

        self.get_logger().info("ROS Test Node initialized. Publishing Twist messages and subscribing to MCU data.")

    def publish_cmd_vel(self):
        """
        Callback for the timer to publish a Twist message.
        """
        twist_msg = Twist()
        # Set some sample velocity values to send to the robot
        twist_msg.linear.x = 0.0  # Linear velocity of 0.2 m/s
        twist_msg.angular.z = 0.0  # Angular velocity of 0.5 rad/s
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Published Twist message: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

    def odom_callback(self, msg):
        """
        Callback function for the /robot/odom topic.
        """
        self.get_logger().info(f"Received Odometry message: Position(x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}), Orientation(z={msg.pose.pose.orientation.z})")

    def battery_callback(self, msg):
        """
        Callback function for the /battery_state topic.
        """
        self.get_logger().info(f"Received BatteryState message: Voltage={msg.voltage}V")

    def emergency_callback(self, msg):
        """
        Callback function for the /emergency_button_status topic.
        """
        self.get_logger().info(f"Received Emergency Stop Status message: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    ros_test_node = RosTestNode()
    try:
        rclpy.spin(ros_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()