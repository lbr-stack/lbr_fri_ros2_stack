import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Pose
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
        super().__init__('long_lines_node')
        self.goalReached = False
        self.curr_pose = Pose()
        self.goal_pub = self.create_publisher(Pose, 'command/Goal_Pose', 1)
        self.printer_pub = self.create_publisher(Float32, 'syringeVel', 1) 
        self.reach_sub = self.create_subscription(Bool, 'state/Goal_Reached', self.goalReached_callback, 1)
        self.pose_sub = self.create_subscription(Pose, 'state/pose', self.update_curr_pose, 1)

        # Use an event to signal the spin thread to stop
        self.spin_event = threading.Event()

        # Start a new thread for spinning
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()

    def spin(self):
        while not self.spin_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
    def update_curr_pose(self, msg):
        # self.curr_pose
        self.curr_pose = msg

    def goalReached_callback(self,msg):
        if msg.data:
            self.goalReached = True

    def wait_for_goal(self, timeout=60.0):
        start_time = datetime.datetime.now()
        while not self.goalReached:
            elapsed_time = (datetime.datetime.now() - start_time).total_seconds()
            if elapsed_time > timeout:
                self.get_logger().warn('Timeout while waiting for goal to be reached')
                return False
            sleep(0.001)
        return True

    def is_close_pos(self, xyz_array, pos_thresh = 0.0005):
        translation_vec = np.asarray([xyz_array[0] - self.curr_pose.position.x, 
                                      xyz_array[1] - self.curr_pose.position.y, 
                                      xyz_array[2] - self.curr_pose.position.z])
        
        return np.linalg.norm(translation_vec)<pos_thresh


    def print_lines(self, start_pos, z_table_height, z_surface_height, needle_surface_dist_ideal):
        
        
        
        global entered
        entered = False

        
        line_distance = 50 * 0.001
        print_length = 140 * 0.001
        line_length = 400 * 0.001
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
        needle_height = z_surface_height + z_table_height + needle_surface_dist_ideal

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

            sleep(0.5)
            self.goalReached = False
            self.goal_pub.publish(print_start_pose)

            if not self.wait_for_goal():
                self.get_logger().error('Failed to reach goal within timeout')
                return


            sleep(0.2)
            file.write('Starting at time: ' + str(create_relative_timestamp()) + '\nline_x: ' + str(line_x) + '\nline_start_y: ' + str(line_start_y) + '\nprint_end_y: ' + str(print_end_y) + '\nline_end_y: ' + str(line_end_y) + '\n')
            file.write('Actual position: x: ' + str(self.curr_pose.position.x) + ' y: ' + str(self.curr_pose.position.y) + ' z: ' + str(self.curr_pose.position.z) + '\n')
            file.write('\n')

            self.printer_pub.publish(Float32(data=print_vel))  # TODO
            self.goalReached = False
            self.goal_pub.publish(line_end_pose)

            flag = 0
            while(not self.goalReached):
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
                
                if(self.is_close_pos(print_end_pose.position) and flag==0):
                    file.write('Injection stopped at time: ' + str(create_relative_timestamp()) + '\n')
                    file.write('Actual position: x: ' + str(self.curr_pose.position.x) + ' y: ' + str(self.curr_pose.position.y) + ' z: ' + str(self.curr_pose.position.z) + '\n')
                    file.write('\n')
                    printer_pub.publish(0)
                    flag = 1
                needle_position_file_counter = needle_position_file_counter + 1
                rclpy.spin_once(self)
            
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
        shooting_pose.position.x = -0.650
        shooting_pose.position.y = 0.0
        shooting_pose.position.z = 0.150
        shooting_pose.orientation = (Rotation.from_ABC([180,0,180],True)).as_geometry_orientation()
        self.goalReached = False
        self.goal_pub.publish(shooting_pose)
        if not self.wait_for_goal():
            self.get_logger().error('Failed to reach goal within timeout')
            return


        
        needle_pos_time = np.vstack((needle_pos_time,np.asarray([self.curr_pose.position.x,self.curr_pose.position.y,self.curr_pose.position.z,create_relative_timestamp()])))
        np.save('needle_array_data.npy', needle_pos_time)
        sleep(2)

    def stop_spin_thread(self):
        self.spin_event.set()
        self.spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = PrintLines()
    node.print_lines(z_table_height=0.01, z_surface_height=0.02, needle_surface_dist_ideal=0.4)
    node.stop_spin_thread()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
