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
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi


class BeaconSearchServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.posx = 0.0
        self.posy = 0.0

        self.posx0 = 0.0
        self.posy0 = 0.0

        self.theta_z = 0.0
        self.theta_z0 = 0.0

        self.x = 0.0
        self.y = 0.0

        self.in_front = 0.0
        self.left_arc = 0.0
        self.right_arc = 0.0
        self.front_arc = 0.0
        self.min_distance = 0.0
        self.max_distance = 0.0

        self.ctrl_c = False
        self.turned = False
        
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
            self.posx0 = self.x
            self.posy0 = self.y
            self.theta_z0 = self.theta_z

    def scan_callback(self, scan_data):
        self.left_arc = scan_data.ranges[0:51]
        self.right_arc = scan_data.ranges[-50:]
        self.front_arc = np.array(self.left_arc[::-1] + self.right_arc[::-1])
        self.in_front = np.abs[self.front_arc[round(len(self.front_arc/2), 0)]]
        self.min_distance = np.argmin[self.front_arc]
        self.max_distance = np.argmax[self.front_arc]

        self.min_object_angle = self.arc_angles[np.argmin(scan_data)]
        
        # determine the angle at which the maximum distance value is located
        # in front of the robot:
        self.max_object_angle = self.arc_angles[np.argmax(self.front_arc)]

    
    def action_server_launcher(self, goal: SearchGoal):
        
        r = rospy.Rate(10) #Hz

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

        # Get the current robot odometry:
        self.posx = self.x
        self.posy = self.y

        # set the robot velocity:
        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)

        while self.tb3_lidar[0] >= goal.approach_distance:
            self.vel_controller.publish()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the search.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break

        while self.tb3_lidar.front_arc < goal.approach_distance:

            while self.tb3_lidar[0] != self.max_distance:

                self.vel_controller.set_move_cmd(0.0, 0.5)
                self.vel_controller.publish()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the search.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break
        
        while self.tb3_lidar.min_distance <= 0.8:
            print('turning')
            self.turn()
            self.vel_controller.publish() 
            self.turned = True
            #self.vel_controller.publish() 

            if self.turned:
                print('here')
                self.turned = False
                while abs(self.theta_z0 - self.theta_z) <= pi/2  :
                    self.vel_controller.set_move_cmd(0, 1.0)
                    self.vel_controller.publish()  
                self.vel_controller.set_move_cmd(0.2, 0)
                self.vel_controller.publish() 

        while  self.tb3_lidar.min_distance < 0.8 and self.tb3_lidar.min_distance >= 0.5 :
            print(f'Minimum distance = {self.tb3_lidar.min_distance}')
            self.turn()
            self.vel_controller.publish() 
            self.turned = True
                #self.vel_controller.publish() 

            if self.turned:
                print('here')
                self.turned = False
                self.vel_controller.set_move_cmd(-0.1, 0) 
                self.vel_controller.publish()

        self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        # populate the feedback message and publish it:
        self.feedback.current_distance_travelled = self.distance
        self.actionserver.publish_feedback(self.feedback)

        while self.tb3_lidar.min_distance <= goal.approach_distance and self.tb3_lidar.min_distance >= 0.3:
            self.vel_controller.set_move_cmd(-0.1, 0) 
            self.vel_controller.publish()

        if success:
            rospy.loginfo("approach completed sucessfully.")
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.tb3_lidar.min_distance
            self.result.closest_object_angle = self.tb3_lidar.closest_object_position

            self.actionserver.set_succeeded(self.result)
            

            self.vel_controller.publish() 
            
            self.vel_controller.stop()
           

                

            
    #Adjust direction to avoid being hit 
    def turn(self): 
        
        left_degree_distance  = self.tb3_lidar.left_arc
        right_degree_distance = self.tb3_lidar.right_arc
        
        #If condition for Determine which direction to turn     
        if left_degree_distance < right_degree_distance :
            print('j')
            self.vel_controller.set_move_cmd(0, -2.0)
            self.vel_controller.publish() 
        elif right_degree_distance == left_degree_distance :
            print("i")
            self.vel_controller.set_move_cmd(0, 2.0)
            self.vel_controller.publish() 
        else:
            self.vel_controller.set_move_cmd(0, -2.0)
            self.vel_controller.publish() 
            print('stop')



            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    BeaconSearchServer()
    rospy.spin()
