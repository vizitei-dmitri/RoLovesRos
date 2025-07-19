#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class AprilTagToNav2(Node):
    def __init__(self):
        super().__init__('apriltag_to_nav2')
        
        # Параметры
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('tag_size_m', 0.16)
        self.declare_parameter('focal_length', 500.0)
        
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_size = self.get_parameter('tag_size_m').value
        self.focal_length = self.get_parameter('focal_length').value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers and publishers
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10)
        
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
        
        self.get_logger().info("AprilTag to Nav2 converter ready!")

    def tag_callback(self, msg):
        if not msg.detections:
            return
            
        tag = msg.detections[0]
        self.get_logger().info(f"Detected tag ID: {tag.id}")
        
        # Create PoseStamped in camera frame
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.camera_frame
        
        # Calculate position (simplified approach)
        # Use the center point and tag size to estimate position
        width_px = abs(tag.corners[0].x - tag.corners[1].x)
        distance = (self.tag_size * self.focal_length) / width_px
        
        # Normalized coordinates
        x_norm = (tag.centre.x - (self.focal_length/2)) / self.focal_length
        y_norm = (tag.centre.y - (self.focal_length/2)) / self.focal_length
        
        # Set pose (note the correct message structure)
        goal_pose.pose.position.x = x_norm * distance
        goal_pose.pose.position.y = y_norm * distance
        goal_pose.pose.position.z = distance
        goal_pose.pose.orientation.w = 1.0  # Neutral orientation
        
        try:
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time())
            
            # Transform pose
            transformed_pose = do_transform_pose(goal_pose, transform)
            
            # Publish goal
            self.goal_pub.publish(transformed_pose)
            self.get_logger().info(f"Published Nav2 goal to {self.target_frame}")
            
        except TransformException as ex:
            self.get_logger().error(f"Transform failed: {ex}")

def main():
    rclpy.init()
    node = AprilTagToNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()