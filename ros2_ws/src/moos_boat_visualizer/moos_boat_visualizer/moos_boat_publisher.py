import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class MOOSBoatPosePublisher(Node):
    def __init__(self):
        super().__init__('moos_boat_pose_publisher')

        self.x = None
        self.y = None
        self.heading = None
        self.origin_x = None
        self.origin_y = None

        self.publisher = self.create_publisher(PoseStamped, 'boat_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float64, 'moos2ros/nav_x', self.x_callback, 10)
        self.create_subscription(Float64, 'moos2ros/nav_y', self.y_callback, 10)
        self.create_subscription(Float64, 'moos2ros/nav_heading', self.heading_callback, 10)

        self.timer = self.create_timer(0.1, self.publish_pose)

        self.path_pub = self.create_publisher(Path, 'boat_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def x_callback(self, msg):
        if self .x is None:
            self.origin_x = msg.data
        self.x = msg.data

    def y_callback(self, msg):
        if self.y is None:
            self.origin_y = msg.data
        self.y = msg.data

    def heading_callback(self, msg):
        self.heading = msg.data
    
    def publish_pose(self):
        if self.x is None or self.y is None or self.heading is None:
            return 
        
        # publish the pose and path
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x - self.origin_x
        msg.pose.position.y = self.y - self.origin_y
        msg.pose.position.z = 0.0
        # Convert heading to quaternion
        yaw = math.radians(-1*self.heading+90)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(msg)
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(msg)
        self.path_pub.publish(self.path_msg)

        # Publish the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'boat_base_link'
        # Relative to the origin(where the boat started)
        t.transform.translation.x = self.x - self.origin_x
        t.transform.translation.y = self.y - self.origin_y
        t.transform.translation.z = 0.0
        yaw = math.radians(self.heading)
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        

def main(args=None):
    rclpy.init(args=args)
    node = MOOSBoatPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()