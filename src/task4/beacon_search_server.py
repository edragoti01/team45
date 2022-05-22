#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
from multiprocessing.connection import wait
import rospy
import actionlib

import time

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry

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

        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()


        self.posx = 0.0
        self.posy = 0.0

        self.posx0 = 0.0
        self.posy0 = 0.0

        self.theta_z = 0.0
        self.theta_z0 = 0.0

        self.x = 0.0
        self.y = 0.0

        self.front_arc = []
        self.left_arc_array = []
        self.right_arc_array = []
        self.back_arc_array = []

        self.min_distance = 0.0
        self.max_distance = 0.0

        self.left_min_distance = 0.0
        self.left_max_distance = 0.0

        self.right_min_distance = 0.0
        self.right_max_distance = 0.0

        self.back_min_distance = 0.0

        self.max_object_angle = 0.0
        self.min_object_angle = 0.0

        self.closest_object_position = 0
        self.furthest_object_position = 0

        self.furthest_object_distance = 0.0

        self.ex_left_array = []
        self.ex_right_array = []

        self.ex_left_min_distance = 0.0
        self.ex_left_max_distance = 0.0

        self.ex_right_min_distance = 0.0
        self.ex_right_max_distance = 0.0



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
        #note: ex means extended

        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]

        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        self.left_arc_array = np.array(left_arc[::-1])
        self.right_arc_array = np.array(right_arc[::-1])
        
        self.min_distance = self.front_arc.min()
        self.max_distance = self.front_arc.max()

        self.left_min_distance = self.left_arc_array.min()
        self.left_max_distance = self.left_arc_array.max()

        self.right_min_distance = self.right_arc_array.min()
        self.right_max_distance = self.right_arc_array.max()

        arc_angles = np.arange(-20, 21)
        self.max_object_angle = arc_angles[np.argmax(scan_data)]
        self.min_object_angle = arc_angles[np.argmin(scan_data)]

        back_arc = scan_data.ranges[160:200]
        self.back_arc_array = np.array(back_arc[::-1])
        self.back_min_distance = self.back_arc_array.min()

        self.closest_object_position = np.argmin(self.front_arc)

        self.furthest_object_position = np.argmax(self.front_arc)

        ex_left_arc = scan_data.ranges[21:81]
        ex_right_arc = scan_data.ranges[-80:-20]

        self.ex_left_array = np.array(ex_left_arc[::-1])
        self.ex_right_array = np.array(ex_right_arc[::-1])

        self.ex_left_min_distance = self.ex_left_array.min()
        self.ex_left_max_distance = self.ex_left_array.max()

        self.ex_right_min_distance = self.ex_right_array.min()
        self.ex_right_max_distance = self.ex_right_array.max()

    
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

        #while the distance in front of the robot is larger or equal to the minimum approach distance
        while self.min_distance > goal.approach_distance:
            self.vel_controller.publish()

            #if the difference between the max distances for the left and right arc is negligible: 
            #if abs(self.left_max_distance - self.right_max_distance) < 0.1:
                #if min distance of extended left arc is bigger than min of extended right arc
                #if self.ex_left_min_distance > self.ex_right_min_distance:
                    #turn left
                #self.vel_controller.set_move_cmd(0.0, 0.8)
                #if the min distance in extended left arc is smaller than the min distance in extended right arc
                #elif self.ex_left_max_distance < self.ex_right_max_distance:
                    #turn right
                #    self.vel_controller.set_move_cmd(0.0, -0.8)

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the search.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break

        #Counts used to make sure robot doesn't get stuck in corners
        loop_count = 0

        stuck_count = 0

        #while the distance in front of the robot is less than the minimum approach distance
        while self.min_distance <= goal.approach_distance:
            loop_count += 1

            self.vel_controller.publish()

            #if loop has run 50000 times (meaning robot is probably stuck)
            if loop_count >= 50000:
                stuck_count += 1

                print("Give me a second, I'm stuck")

                #turn right
                self.vel_controller.set_move_cmd(0.0, -0.9)

                #wait for 1 second and try again
                time.sleep(1)

                if stuck_count > 4:
                    print("Just a minute")

                    #turn left and wait a second
                    self.vel_controller.set_move_cmd(0.0, 0.9)

                    time.sleep(1)


            #if the max distance of the left and right arcs are equal 
            if self.left_max_distance == self.right_max_distance:
                #if min distance of extended left arc is bigger than min of extended right arc
                if self.ex_left_min_distance > self.ex_right_min_distance:
                    #turn left
                    self.vel_controller.set_move_cmd(0.0, 0.9)
                #if the min distance in extended left arc is smaller than the min distance in extended right arc
                elif self.ex_left_max_distance < self.ex_right_max_distance:
                    #turn right
                    self.vel_controller.set_move_cmd(0.0, -0.9)

            #if the min distance in the right arc is smaller than the min distance in the left arc
            elif self.left_min_distance > self.right_min_distance:
                #turn left
                self.vel_controller.set_move_cmd(0.0, 0.9)
            #if the min distance in the left arc is smaller than the min distance in the right arc
            elif self.left_min_distance < self.right_min_distance:
                #turn right
                self.vel_controller.set_move_cmd(0.0, -0.9)



            #if the robot is too close to an object at the back
            while self.back_min_distance <= 0.1:
                self.vel_controller.stop()

                #if min distance in front of robot is greater than the min distance at the back
                if self.min_distance > self.back_min_distance:
                    #move forward slowly
                    self.vel_controller.set_move_cmd(0.1, 0.0)
                #if min distance in front is smaller than min distance at the back
                elif self.min_distance < self.back_min_distance:
                    #determine which arc (left or right) has the max distance and turn that way

                    #if left max is bigger than right max
                    if self.left_max_distance > self.right_max_distance:
                        #turn left until the front min distance is bigger than the back min distance
                        while self.min_distance < self.back_min_distance:
                            self.vel_controller.set_move_cmd(0.0, 0.2)
                    
                    #if right max is bigger than left max               
                    elif self.left_max_distance > self.right_max_distance:
                        #turn right until the front min distance is bigger than the back min distance
                        while self.min_distance < self.back_min_distance:
                            self.vel_controller.set_move_cmd(0.0, -0.2)

                    #if left and right max are equal
                    else:
                        #turn left until front min is bigger than back min
                        while self.min_distance < self.back_min_distance:
                            self.vel_controller.set_move_cmd(0.0, 0.2)




            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the search.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break



        self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        # populate the feedback message and publish it:
        self.feedback.current_distance_travelled = self.distance
        self.actionserver.publish_feedback(self.feedback)

        if success:
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.min_distance
            self.result.closest_object_angle = self.closest_object_position

            self.actionserver.set_succeeded(self.result)
            

            self.vel_controller.publish() 
            
            self.vel_controller.stop()
           
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    BeaconSearchServer()
    rospy.spin()
