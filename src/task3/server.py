#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi


class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup: 
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.turned = False

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
    
    def scan_callback(self, scan_data):
        self.left_arc = scan_data.ranges[0:20]
        self.right_arc = scan_data.ranges[-20:]
        self.front_arc = np.array(self.left_arc[::-1] + self.right_arc[::-1])
        self.min_distance = self.front_arc.min()
        # Create another numpy array which represents the angles 
        # (in degrees) associated with each of the data-points in 
        # the "front_arc" array above:
        self.arc_angles = np.arange(-20, 20)
        
        # determine the angle at which the minimum distance value is located
        # in front of the robot:
        self.object_angle = self.arc_angles[np.argmin(self.front_arc)]

    
    def action_server_launcher(self, goal: SearchGoal):
        
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid velocity.  Select a value between 0 and 0.26 m/s.")
            success = False
        if goal.approach_distance <= 0.2:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print(f"Request to move at {goal.fwd_velocity:.3f}m/s "
                f"and stop {goal.approach_distance:.2f}m "
                f"infront of any obstacles")

        # Get the current robot odometry:
        #self.posx0 = self.tb3_odom.posx
        #self.posy0 = self.tb3_odom.posy
        self.posx0 = 0.0
        self.posy0 = 0.0
        #self.vel_controller.publish() 

        print("The robot will start to move now...")
        # set the robot velocity:
        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
        reference_time = 0
        current_time = rospy.get_rostime().secs
        print(current_time)
        if reference_time == 0:
            reference_time = current_time
        while self.tb3_lidar.min_distance > goal.approach_distance:
            run_time = current_time - reference_time
            #print(run_time)
            print(f'Minimum distance = {self.tb3_lidar.min_distance}')
            self.vel_controller.publish()
            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the camera sweep.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break
            while self.tb3_lidar.min_distance <= 0.5:
                if min(self.tb3_lidar.left_arc) >= min(self.tb3_lidar.right_arc):
                    print('turning')
                    self.turnleft()
                    self.vel_controller.publish() 
                    self.turned = True
                #self.vel_controller.publish() 
                if min(self.tb3_lidar.left_arc) < min(self.tb3_lidar.right_arc):
                    print('turning')
                    self.turnright()
                    self.vel_controller.publish() 
                    self.turned = True
                #self.vel_controller.publish() 
            if self.turned:
                print('here')
                self.turned = False
                
                
                
                self.vel_controller.set_move_cmd(0.2, 0)
                if run_time >= (1.8):                           
                    self.turnleft()
                    self.vel_controller.publish()  
                else:
                    self.vel_controller.set_move_cmd(0.2, 0)
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            # populate the feedback message and publish it:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

        

        if success:
            rospy.loginfo("approach completed sucessfully.")
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.tb3_lidar.min_distance
            self.result.closest_object_angle = self.tb3_lidar.closest_object_position

            self.actionserver.set_succeeded(self.result)
            

            self.vel_controller.publish() 
            
            self.vel_controller.stop()
           

                

            
    #Adjust direction to avoid being hit 
    def turnleft(self): 
        print ( "left")        
        #If condition for Determine which direction to turn     
        if abs(self.theta_z0 - self.theta_z) <= pi/2 :
            self.vel_controller.set_move_cmd(0, -2.0)
            self.vel_controller.publish()  
        else:
            self.vel_controller.set_move_cmd(0, 0)
            self.vel_controller.publish() 
            

    def turnright(self):       
        print ( "right")    
        #If condition for Determine which direction to turn     
        if abs(self.theta_z0 - self.theta_z) <= pi/2  :
            self.vel_controller.set_move_cmd(0, 2.0)
            self.vel_controller.publish()  
        else:
            self.vel_controller.set_move_cmd(0, 0)
            self.vel_controller.publish() 
           
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
