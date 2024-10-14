import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math

class PosePlanningNode(Node):
    def __init__(self):
        super().__init__('pose_planning')
        self.is_init = False
        self.amplitude = 0.05
        self.frequency = 0.2
        self.sampling_time = 0.01
        self.phase = 0.0

        self.pose_pub = self.create_publisher(Pose, 'command/pose', 1)
        self.pose_sub = self.create_subscription(Pose, 'state/pose', self.on_pose, 1)

    def on_pose(self, msg):
        if not self.is_init:
            self.initial_pose = msg
            self.is_init = True
        else:

            cartesian_pose_command = Pose()
            cartesian_pose_command.position.x = self.initial_pose.position.x
            cartesian_pose_command.position.y = self.initial_pose.position.y
            cartesian_pose_command.position.z = self.initial_pose.position.z
            print('...')
            print(msg.position.z)

            self.phase += 2 * math.pi * self.frequency * self.sampling_time
            cartesian_pose_command.position.z += self.amplitude * math.sin(self.phase)

            cartesian_pose_command.orientation = self.initial_pose.orientation

            self.pose_pub.publish(cartesian_pose_command)
            print(cartesian_pose_command.position.z)

def main(args=None):
    rclpy.init(args=args)
    node = PosePlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
