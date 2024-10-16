import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose  # This will hold the camera position (translation, orientation)

class CameraPosePublisher(Node):

    def __init__(self):
        super().__init__('camera_pose_publisher')
        self.publisher_ = self.create_publisher(Pose, 'camera_pose_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer as needed

    def timer_callback(self):
        msg = Pose()
        # Set the camera position data
        # Example: camera_pose = [x, y, z, qw, qx, qy, qz]
        camera_pose = self.get_camera_pose()  # Function to extract camera pose from Pegasus
        msg.position.x = camera_pose[0]
        msg.position.y = camera_pose[1]
        msg.position.z = camera_pose[2]
        msg.orientation.w = camera_pose[3]
        msg.orientation.x = camera_pose[4]
        msg.orientation.y = camera_pose[5]
        msg.orientation.z = camera_pose[6]

        self.publisher_.publish(msg)

    def get_camera_pose(self):
        # Replace this with Pegasus API call to get camera position
        # Here’s just an example dummy data
        return [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    camera_pose_publisher = CameraPosePublisher()
    rclpy.spin(camera_pose_publisher)
    camera_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()