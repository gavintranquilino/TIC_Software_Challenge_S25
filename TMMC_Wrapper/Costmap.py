import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')
        self.get_logger().info('Costmap Node Initialized')

        # Publisher for the /costmap topic
        self.costmap_publisher_ = self.create_publisher(String, '/costmap', 10)

        # Subscriber to the /scan topic (LiDAR)
        self.lidar_subscription_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Call the callback function every timer cycle
        self.timer_period_ = 1.0  # seconds
        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

        self.get_logger().info('Subscribed to /scan and will publish to /costmap')

    def lidar_callback(self, msg: LaserScan):
        msg = String()
        msg.data = f'Received LiDAR scan: {len(msg.ranges)} points. Angle min: {msg.angle_min:.2f}, max: {msg.angle_max:.2f}, increment: {msg.angle_increment:.4f}'
        self.costmap_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    costmap_node = CostmapNode()
    try:
        rclpy.spin(costmap_node)  # Keeps the node alive and processing callbacks
    except KeyboardInterrupt:
        costmap_node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Cleanly destroy the node and shutdown rclpy
        costmap_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
