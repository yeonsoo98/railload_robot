import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import velodyne_pointcloud.msg as vlp_msg

class VelodyneNode(Node):
    def __init__(self):
        super().__init__('velodyne_node')

        self.publisher = self.create_publisher(PointCloud2, 'velodyne_points', 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'velodyne'

        # Create a sample point cloud message
        pcl_msg = PointCloud2()
        pcl_msg.header = header
        pcl_msg.height = 1
        pcl_msg.width = 4  # Assuming 4 points for simplicity
        pcl_msg.is_bigendian = False
        pcl_msg.point_step = 32  # Size of a point in bytes
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width

        # Fill point cloud data (x, y, z, intensity)
        points_data = [
            1.0, 2.0, 3.0, 100,
            4.0, 5.0, 6.0, 150,
            7.0, 8.0, 9.0, 200,
            10.0, 11.0, 12.0, 255
        ]

        pcl_msg.data = bytearray(points_data)

        self.publisher.publish(pcl_msg)
        self.get_logger().info('Publishing Velodyne points')

def main(args=None):
    rclpy.init(args=args)

    velodyne_node = VelodyneNode()

    try:
        rclpy.spin(velodyne_node)
    except KeyboardInterrupt:
        pass

    velodyne_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
