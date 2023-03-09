#!/usr/bin/python3
import rclpy
from rclpy import qos
from rclpy.node import Node
import optas

import numpy as np

from lbr_fri_msgs.msg import LBRState

from geometry_msgs.msg import Wrench, Point
from visualization_msgs.msg import Marker


class ExtTorqueExampleNode(Node):
    def __init__(self):
        super().__init__("lbr_ext_torque_example")
        self.declare_parameter("lbr_description")
        ee_link = "storz_tilt_endoscope_link_cm_optical"
        self.robot = optas.RobotModel(
            urdf_string=self.get_parameter("lbr_description").value
        )
        self._position = self.robot.get_global_link_position_function(ee_link)
        self._jacobian = self.robot.get_global_geometric_jacobian_function(ee_link)
        self.pub_f_ext = self.create_publisher(
            Wrench, "tip_force_external", qos.qos_profile_system_default
        )
        self.marker_pub = self.create_publisher(
            Marker, "arrow", qos.qos_profile_system_default
        )
        self.marker = Marker()
        self.marker.header.frame_id = 'world'
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.color.a = 0.75
        self.marker.color.r = 1.
        self.marker.scale.x = 0.01
        self.marker.scale.y = 0.02

        self.sub = self.create_subscription(
            LBRState, "lbr_state", self._callback, qos.qos_profile_system_default
        )

    def p(self, q):
        return self._position(q).toarray().flatten()

    def J(self, q):
        return self._jacobian(q).toarray()

    def _callback(self, msg):
        q = np.array(msg.measured_joint_position)
        p = self.p(q)
        ext_tau = np.array(msg.external_torque)
        J = self.J(q)
        Jinv = np.linalg.pinv(J)#, rcond=0.05)
        f_ext = Jinv.T @ ext_tau
        msg = Wrench()
        msg.force.x = f_ext[0]
        msg.force.y = f_ext[1]
        msg.force.z = f_ext[2]
        msg.torque.x = f_ext[3]
        msg.torque.y = f_ext[4]
        msg.torque.z = f_ext[5]
        self.pub_f_ext.publish(msg)
        start = Point(x=p[0], y=p[1], z=p[2])
        temp = p + f_ext[:3]
        end = Point(x=temp[0], y=temp[1], z=temp[2])
        self.marker.points = [start, end]
        self.marker.header.stamp = self.get_clock().now().to_msg()        
        self.marker_pub.publish(self.marker)
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ExtTorqueExampleNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
