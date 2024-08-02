import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Pose
from lbr_fri_idl.srv import MoveToPose, FreeFormMove
import datetime
import numpy as np
import threading
from lbr_demos_py.asbr import * 
from time import sleep
import copy
from scipy.interpolate import splprep, splev






class PrintLines(Node):

    def __init__(self):
        super().__init__('submerged_demo')
        self.move_client = self.create_client(MoveToPose, 'move_to_pose')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.free_form_client = self.create_client(FreeFormMove, 'free_form_move')
        self.request = MoveToPose.Request()
        self.FFM_request = FreeFormMove.Request()
        self.reach_subs = self.create_subscription(Bool, 'goal_reached_top', self.goal_reach_update, 1)
        self.curr_pose_subs = self.create_subscription(Pose, 'state/pose', self.curr_pose_update, 1)

        self.goal_state = False
        self.commiunication_rate = 0.01
        self.printer_pub = self.create_publisher(Float32, '/syringevel', 1) 
        self.curr_pose = Pose()

    def curr_pose_update(self, msg):
        self.curr_pose = msg

    def send_request_FFM(self, goal_poses, lin_vel):
        self.FFM_request.goal_pose = goal_poses
        if isinstance(lin_vel, list):
            self.FFM_request.lin_vel = [Float32(data=v) for v in lin_vel]
        elif isinstance(lin_vel,float):
            self.FFM_request.lin_vel = [Float32(data=lin_vel)] * len(goal_poses)
        self.future = self.free_form_client.call_async(self.FFM_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    
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

    def spline_interpol(self,way_poses, num_final_poses = 50):
        # Keeps the orientation of the first pose
        orien = way_poses[0].orientation

        # Extract positions
        positions = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in way_poses])

        # Fit spline
        tck, u = splprep(positions.T, s=0, k=2)

        # Evaluate spline
        u_fine = np.linspace(0, 1, num_final_poses)  
        x_fine, y_fine, z_fine = splev(u_fine, tck)
        refined_points = np.array([x_fine, y_fine, z_fine]).T

        final_poses = []

        for point in refined_points:
            temp = Pose()
            temp.position.x = point[0]
            temp.position.y = point[1]
            temp.position.z = point[2]
            temp.orientation = orien
            final_poses.append(temp)

        return final_poses






    def print_lines_arc(self, start_pos, lin_vel):
        #rectangle layer by layer
        
        # radius = 0.002
        height = 0.06
        # line_length = 100 * 0.001
        # num_lines = 2
        # print_vel = 0.25 #TODO

        center = copy.deepcopy(start_pos)
        center.position.z -= height 

        sleep(0.2)
        response = self.send_request(center, lin_vel*5)
        self.wait_for_goal()
        print('reached center node')

        number_of_layers = 1
        inc = 0.002
        for i in range(number_of_layers):
            self.printer_pub.publish(Float32(data=0.2))

            first_node = Pose()
            first_node.position.x = center.position.x
            first_node.position.y = center.position.y
            first_node.position.z = center.position.z + i*inc
            first_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(first_node, lin_vel/2)
            self.wait_for_goal()
            print('reached first node')     
            a = input('wait for enter:') 


            second_node = Pose()
            second_node.position.x = center.position.x + 0.02
            second_node.position.y = center.position.y 
            second_node.position.z = center.position.z + i*inc
            second_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(second_node, lin_vel/2)
            self.wait_for_goal()
            print('reached second node')

            third_node = Pose()
            third_node.position.x = center.position.x + 0.02
            third_node.position.y = center.position.y + 0.02
            third_node.position.z = center.position.z + i*inc
            third_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(third_node, lin_vel/2)
            self.wait_for_goal()
            print('reached third node')

            fourth_node = Pose()
            fourth_node.position.x = center.position.x
            fourth_node.position.y = center.position.y + 0.02
            fourth_node.position.z = center.position.z + i*inc
            fourth_node.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
            response = self.send_request(fourth_node, lin_vel/2)
            self.wait_for_goal()
            print('reached fourth node')
            self.printer_pub.publish(Float32(data=0.0))


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

    def print_liver_vasc(self, start_pos, lin_vel):

        height = 0.03
        radius = 0.02

        upper_center = copy.deepcopy(start_pos)

        center = copy.deepcopy(start_pos)
        center.position.z -= height

        # sleep(0.2)
        # response = self.send_request(center, lin_vel*5)
        # self.wait_for_goal()
        # print('reached center node')
        # a = input('wait for enter:')

        num_lines = 4
        for line_num in range(num_lines):
            angle = line_num * 2 * 3.141592 / num_lines
            way_poses = []
            way_poses.append(center)
            for temp_counter in range(2):
                temp_pose = copy.deepcopy(center)
                temp_pose.position.x += ((temp_counter+1) * radius * np.cos(angle) / 3)
                temp_pose.position.y += ((temp_counter+1) * radius * np.sin(angle) / 3)
                temp_pose.position.z -= ((temp_counter+1) * height / 5)
                way_poses.append(temp_pose)

            node_2 = copy.deepcopy(center)
            node_2.position.x += radius * np.cos(angle)
            node_2.position.y += radius * np.sin(angle)
            node_2.position.z -= height
            way_poses.append(node_2)
            way_poses = self.spline_interpol(way_poses)

            node_1 = copy.deepcopy(upper_center)
            node_1.position.x += radius * np.cos(angle)
            node_1.position.y += radius * np.sin(angle)


            # response = self.send_request_FFM(way_poses, lin_vel)
            # print(f'Success: {response.success}')
            # self.wait_for_goal()
            # a = input('Enter to continue: ')

            way_poses.reverse()

            response = self.send_request(node_1, lin_vel*4)
            print(response)
            self.wait_for_goal()
            sleep(0.2)

            response = self.send_request(node_2, lin_vel*2)
            print(response)
            self.wait_for_goal()
            sleep(0.2)

            a=input('Press Enter:')
            response = self.send_request_FFM(way_poses, lin_vel)
            print(f'Success: {response.success}')
            self.wait_for_goal()
            sleep(3.0)

            response = self.send_request(upper_center, lin_vel*2)
            print(response)
            self.wait_for_goal()
            sleep(0.2)

        return

    def layer_by_layer(self, start_pos, lin_vel, dummy_vel):
        #Prints 3 pillars in layer by layer method. No transverse motions in the media
        #dummy_vel: velocity in between places which the robot can move pretty fast!
        height = 0.04
        layer_height = 0.0012
        number_of_layers = int(height / (layer_height * 2))

        inj_rate = 0.65

        radius = 0.006

        center = copy.deepcopy(start_pos)

        for layer_num in range(number_of_layers):
            node_1_up = copy.deepcopy(center)
            node_1_up.position.x += radius

            node_1_down = copy.deepcopy(node_1_up)
            node_1_down.position.z -= height - (layer_num+1)*layer_height


            node_2_up = copy.deepcopy(center)
            node_2_up.position.x -= radius

            node_2_down = copy.deepcopy(node_2_up)
            node_2_down.position.z -= height - (layer_num+1)*layer_height


            node_3_up = copy.deepcopy(center)
            node_3_up.position.y += radius - 2*(layer_num+1)*radius/number_of_layers

            node_3_down = copy.deepcopy(node_3_up)
            node_3_down.position.z -= height - (layer_num+1)*layer_height


            response = self.send_request(node_1_up, dummy_vel)
            print(response)
            self.wait_for_goal()
            sleep(0.2)
            a = input('Enter to proceed')

            response = self.send_request(node_1_down, lin_vel)
            print(response)
            self.wait_for_goal()
            # self.printer_pub.publish(Float32(data=0.1))  # TODO
            # sleep(2.0)
            # self.printer_pub.publish(Float32(data=0.0))
            # sleep(12.0)
            a = input('Waiting:')

            response = self.send_request(node_1_up, lin_vel)
            print(response)
            self.wait_for_goal()
            sleep(0.2)

            a = input('Enter to proceed')


            # response = self.send_request(node_2_up, dummy_vel)
            # print(response)
            # self.wait_for_goal()
            # sleep(0.2)

            # a = input('Enter to proceed')

            # response = self.send_request(node_2_down, lin_vel)
            # print(response)
            # self.wait_for_goal()
            # # self.printer_pub.publish(Float32(data=0.1))  # TODO
            # # sleep(2.0)
            # # self.printer_pub.publish(Float32(data=0.0))
            # # sleep(12.0)
            # a = input('Waiting:')

            # response = self.send_request(node_2_up, lin_vel)
            # print(response)
            # self.wait_for_goal()
            # sleep(0.2)


            # a = input('Enter to proceed')

            # response = self.send_request(node_3_up, dummy_vel)
            # print(response)
            # self.wait_for_goal()
            # sleep(0.2)

            # a = input('Enter to proceed')

            # response = self.send_request(node_3_down, lin_vel)
            # print(response)
            # self.wait_for_goal()
            # # self.printer_pub.publish(Float32(data=0.1))  # TODO
            # # sleep(2.0)
            # # self.printer_pub.publish(Float32(data=0.0))
            # # sleep(12.0)
            # a = input('Waiting:')

            # response = self.send_request(node_3_up, lin_vel)
            # print(response)
            # self.wait_for_goal()
            # sleep(0.2)

        return


def main(args=None):
    rclpy.init(args=args)
    node = PrintLines()
    lin_vel = 0.002
    home_pose = Pose()
    home_pose.position.x = 0.65
    home_pose.position.y = 0.0
    home_pose.position.z = 0.354 #was 40
    home_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

    node.go_home(home_pose, 0.01)
    print('Home position finished')
    sleep(2)
    wait_input = input('Press Enter to Print:')

    node.print_lines_arc(home_pose, lin_vel)

    sleep(2)
    node.go_home(home_pose, 0.005)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
