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



first_timestamp = None

def listen_for_enter():
    global entered
    input("Press Enter to record time stamp:")
    entered = True

    
def create_relative_timestamp():
    global first_timestamp
    
    current_time = datetime.datetime.now()
    
    if first_timestamp is None:
        first_timestamp = current_time
    
    time_difference = (current_time - first_timestamp).total_seconds() * 1000
    
    return time_difference



class PrintLines(Node):

    def __init__(self):
        super().__init__('long_lines')
        self.move_client = self.create_client(MoveToPose, 'move_to_pose')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = MoveToPose.Request()
        self.reach_subs = self.create_subscription(Bool, 'goal_reached_top', self.goal_reach_update, 1)
        self.curr_pose_subs = self.create_subscription(Pose, 'state/pose', self.curr_pose_update, 1)

        self.goal_state = False
        self.commiunication_rate = 0.01
        self.printer_pub = self.create_publisher(Float32, 'syringeVel', 1) 
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



    def print_lines(self, start_pos, z_table_height, z_surface_height, needle_surface_dist_ideal, lin_vel):
        global entered
        entered = False
        
        line_distance = 50 * 0.001
        print_length = 50 * 0.001
        line_length = 100 * 0.001
        num_lines = 2
        direction = 1
        needle_position_file_counter = 0
        print_vel = 0.25 #TODO
        
        file = open("time_stamps.txt", "w")

        file.write('Settings:\n')
        file.write('line_distance: ' + str(line_distance) + '\n')
        file.write('print_length: ' + str(print_length) + '\n')
        file.write('line_length: ' + str(line_length) + '\n')
        file.write('print_vel: ' + str(print_vel) + '\n')
        file.write('z_table_height: ' + str(z_table_height) + '\n')
        file.write('z_surface_height: ' + str(z_surface_height) + '\n')
        file.write('needle_surface_dist_ideal: ' + str(needle_surface_dist_ideal) + '\n')
        file.write('\nTime Stamps: \n \n')

        needle_pos_time = np.array([self.curr_pose.position.x,self.curr_pose.position.y,self.curr_pose.position.z,create_relative_timestamp()])

        thread = threading.Thread(target=listen_for_enter)
        thread.daemon = True
        thread.start()
        # needle_height = z_surface_height + z_table_height + needle_surface_dist_ideal
        needle_height = start_pos.position.z - 0.02


        for line_num in range(num_lines):
            print('line_num: ', line_num)
            file.write('\nLine number: ' + str(line_num+1) + ' out of ' + str(num_lines) + '\n\n')
            line_x = start_pos.position.x + line_distance * line_num
            line_start_y = start_pos.position.y + (-line_length if direction == -1 else 0)
            line_end_y = start_pos.position.y + (-line_length if direction == 1 else 0)
            print_end_y = line_start_y + (-print_length if direction == 1 else print_length)
            print(line_end_y,print_end_y)

            print_start_pose = Pose()
            print_start_pose.position.x = line_x
            print_start_pose.position.y = line_start_y
            print_start_pose.position.z = needle_height
            print_start_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

            print_end_pose = Pose()
            print_end_pose.position.x = line_x
            print_end_pose.position.y = print_end_y
            print_end_pose.position.z = needle_height
            print_end_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

            line_end_pose = Pose()
            line_end_pose.position.x = line_x
            line_end_pose.position.y = line_end_y
            line_end_pose.position.z = needle_height
            line_end_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

            # self.goal_state = False
            response = self.send_request(print_start_pose, lin_vel)
            self.wait_for_goal()


            sleep(0.2)
            file.write('Starting at time: ' + str(create_relative_timestamp()) + '\nline_x: ' + str(line_x) + '\nline_start_y: ' + str(line_start_y) + '\nprint_end_y: ' + str(print_end_y) + '\nline_end_y: ' + str(line_end_y) + '\n')
            file.write('Actual position: x: ' + str(self.curr_pose.position.x) + ' y: ' + str(self.curr_pose.position.y) + ' z: ' + str(self.curr_pose.position.z) + '\n')
            file.write('\n')

            self.printer_pub.publish(Float32(data=print_vel))  # TODO
            # self.goal_state = False
            response = self.send_request(print_end_pose, lin_vel)

            while(not self.goal_state):
                if (needle_position_file_counter%10==0):
                    needle_pos_time = np.vstack((needle_pos_time,np.asarray([self.curr_pose.position.x,self.curr_pose.position.y,self.curr_pose.position.z,create_relative_timestamp()])))
                if(entered):
                    file.write('Time stamp per user request: ' + str(create_relative_timestamp()) + '\n')
                    file.write('Actual position: x: ' + str(self.curr_pose.position.x) + ' y: ' + str(self.curr_pose.position.y) + ' z: ' + str(self.curr_pose.position.z) + '\n')
                    file.write('\n')
                    entered = False
                    thread = threading.Thread(target=listen_for_enter)
                    thread.daemon = True
                    thread.start()
                
                needle_position_file_counter = needle_position_file_counter + 1
                rclpy.spin_once(self, timeout_sec = self.commiunication_rate)
            self.goal_state = False


            file.write('Injection stopped at time: ' + str(create_relative_timestamp()) + '\n')
            file.write('Actual position: x: ' + str(self.curr_pose.position.x) + ' y: ' + str(self.curr_pose.position.y) + ' z: ' + str(self.curr_pose.position.z) + '\n')
            file.write('\n')
            self.printer_pub.publish(Float32(data = 0.0))

            response = self.send_request(line_end_pose, lin_vel)
            self.wait_for_goal()

            file.write('End line reached at time: ' + str(create_relative_timestamp()) + '\n')
            file.write('Actual position: x: ' + str(self.curr_pose.position.x) + ' y: ' + str(self.curr_pose.position.y) + ' z: ' + str(self.curr_pose.position.z) + '\n')
            file.write('\n')
            sleep(1)
            direction = -direction
        sleep(0.1)
        file.close()
        # needle_position_file.close()

        
        print('Take me to shooting pose!')
        shooting_pose = Pose()
        shooting_pose.position.x = 0.550
        shooting_pose.position.y = 0.0
        shooting_pose.position.z = 0.50
        shooting_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

        response = self.send_request(shooting_pose, lin_vel)
        self.wait_for_goal()
        


        
        needle_pos_time = np.vstack((needle_pos_time,np.asarray([self.curr_pose.position.x,self.curr_pose.position.y,self.curr_pose.position.z,create_relative_timestamp()])))
        np.save('needle_array_data.npy', needle_pos_time)
        sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = PrintLines()
    lin_vel = 0.005
    home_pose = Pose()
    home_pose.position.x = 0.6
    home_pose.position.y = 0.05
    home_pose.position.z = 0.45
    home_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()

    node.go_home(home_pose, lin_vel)
    print('Home position finished')
    sleep(2)

    node.print_lines(home_pose, 0.01, 0.02, 0.4, 0.005)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
