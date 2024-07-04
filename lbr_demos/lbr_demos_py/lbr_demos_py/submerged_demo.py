import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Pose
from lbr_fri_idl.srv import MoveToPose
import datetime
import numpy as np
import threading
from lbr_demos_py.asbr import * 
from time import sleep
import copy





class PrintLines(Node):

    def __init__(self):
        super().__init__('submerged_demo')
        self.move_client = self.create_client(MoveToPose, 'move_to_pose')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = MoveToPose.Request()
        self.reach_subs = self.create_subscription(Bool, 'goal_reached_top', self.goal_reach_update, 1)
        self.curr_pose_subs = self.create_subscription(Pose, 'state/pose', self.curr_pose_update, 1)

        self.goal_state = False
        self.commiunication_rate = 0.01
        # self.printer_pub = self.create_publisher(Float32, 'syringeVel', 1) 
        self.curr_pose = Pose()

    def curr_pose_update(self, msg):
        self.curr_pose = msg

    
    def send_request(self, goal_pose, lin_vel):
        self.request.goal_pose = goal_pose
        self.request.lin_vel = Float32(data = lin_vel)
        self.future = self.move_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def goal_reach_update(self, msg):
        if msg.data == True:
            self.goal_state = True

    def wait_for_goal(self):
        while not self.goal_state:
            rclpy.spin_once(self, timeout_sec = self.commiunication_rate)
        self.goal_state = False
        return 

    def go_home(self, home_pose, lin_vel = 0.005):
        response = self.send_request(home_pose, lin_vel)
        print(response)
        self.wait_for_goal()    
        return 



    def print_lines_arc(self, start_pos, lin_vel):
        #rectangle layer by layer
        global entered
        entered = False
        
        radius = 0.002
        height = 0.06
        line_length = 100 * 0.001
        num_lines = 2
        # print_vel = 0.25 #TODO

        center = copy.deepcopy(start_pos)
        center.position.z -= height * 2

        sleep(0.2)
        response = self.send_request(center, lin_vel*5)
        self.wait_for_goal()
        print('reached center node')

        number_of_layers = 2
        inc = 0.002
        for i in range(number_of_layers):

            first_node = Pose()
            first_node.position.x = center.position.x
            first_node.position.y = center.position.y
            first_node.position.z = center.position.z + i*inc
            first_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(first_node, lin_vel)
            self.wait_for_goal()
            print('reached first node')     
            a = input('wait for enter:') 


            second_node = Pose()
            second_node.position.x = center.position.x + 0.04
            second_node.position.y = center.position.y 
            second_node.position.z = center.position.z + i*inc
            second_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(second_node, lin_vel)
            self.wait_for_goal()
            print('reached second node')

            third_node = Pose()
            third_node.position.x = center.position.x + 0.04
            third_node.position.y = center.position.y + 0.04
            third_node.position.z = center.position.z + i*inc
            third_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(third_node, lin_vel)
            self.wait_for_goal()
            print('reached third node')

            fourth_node = Pose()
            fourth_node.position.x = center.position.x
            fourth_node.position.y = center.position.y + 0.04
            fourth_node.position.z = center.position.z - i*inc
            fourth_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(fourth_node, lin_vel)
            self.wait_for_goal()
            print('reached fourth node')


        # sleep(0.2)  
        # response = self.send_request(first_node, lin_vel*5)
        # self.wait_for_goal()
        # print('reached first node')      
        # wait_input = input('Press Enter to Print:')

        # sleep(0.2)  
        # response = self.send_request(center, lin_vel)
        # self.wait_for_goal()
        # print('reached center node')      
        # # wait_input = input('Press Enter to Print:')

        # sleep(0.2)  
        # response = self.send_request(second_node, lin_vel)
        # self.wait_for_goal()
        # print('reached second node')      
        # wait_input = input('Press Enter to Print:')   


        # for line_num in range(num_lines):
        #     wait_input = input('Press Enter to Print:')

        #     print('line_num: ', line_num)
        #     angle = line_num * 2 * 3.141592 / num_lines

        #     first_node = Pose()
        #     first_node.position.x = center.position.x + radius * np.cos(angle)
        #     first_node.position.y = center.position.y + radius * np.sin(angle)
        #     first_node.position.z = center.position.z - height/2.0
        #     first_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

        #     second_node = copy.deepcopy(first_node)
        #     second_node.position.z = second_node.position.z - height/2.0

        #     # sleep(0.2)
        #     # response = self.send_request(first_node, lin_vel)
        #     # self.wait_for_goal()
        #     # print('reached first node')

        #     sleep(0.2)  
        #     response = self.send_request(second_node, lin_vel)
        #     self.wait_for_goal()
        #     print('reached second node')      
        #     wait_input = input('Press Enter to Print:')      

        #     sleep(0.2)
        #     response = self.send_request(first_node, lin_vel)
        #     self.wait_for_goal()
        #     print('reached first node')
        #     wait_input = input('Press Enter to Print:')

        #     sleep(0.2)
        #     response = self.send_request(center, lin_vel)
        #     self.wait_for_goal()
        #     print('reached center node')
        #     wait_input = input('Press Enter to Print:')

            # self.printer_pub.publish(Float32(data=print_vel))  # TODO
            # self.printer_pub.publish(Float32(data = 0.0))


    def print_lines(self, start_pos, lin_vel):
        #rectangle layer by layer

        
        height = 0.06

        center = copy.deepcopy(start_pos)
        center.position.z -= height * 2

        sleep(0.2)
        response = self.send_request(center, lin_vel*5)
        self.wait_for_goal()
        print('reached center node')
        a = input('wait for enter:') 

        first_node = Pose()
        first_node.position.x = center.position.x
        first_node.position.y = center.position.y
        first_node.position.z = center.position.z + height - 0.015
        first_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
        response = self.send_request(first_node, lin_vel)
        self.wait_for_goal()
        print('reached first node')     


        second_node = Pose()
        second_node.position.x = center.position.x + 0.04
        second_node.position.y = center.position.y 
        second_node.position.z = center.position.z + height - 0.015
        second_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
        response = self.send_request(second_node, lin_vel)
        self.wait_for_goal()
        print('reached second node')

        return


def main(args=None):
    rclpy.init(args=args)
    node = PrintLines()
    lin_vel = 0.001
    home_pose = Pose()
    home_pose.position.x = 0.65
    home_pose.position.y = 0.0
    home_pose.position.z = 0.40
    home_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

    node.go_home(home_pose, lin_vel*5)
    print('Home position finished')
    sleep(2)
    wait_input = input('Press Enter to Print:')

    node.print_lines(home_pose, lin_vel)

    sleep(2)
    node.go_home(home_pose, lin_vel*2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
