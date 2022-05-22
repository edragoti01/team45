#!/usr/bin/env python3

import rospy
from pathlib import Path
import actionlib


from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback

import numpy as np

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

image_path = Path("/home/student/catkin_ws/src/team45/src/snaps/task4_target_colour.jpg")


class Beaconing(object):

    def __init__(self):
        node_name = "beaconing_node"
        rospy.init_node(node_name)

        topic_name = "/camera/rgb/image_raw"
        self.camera_sub = rospy.Subscriber(topic_name,
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.rate = rospy.Rate(10) # hz

        self.m00 = 0
        self.m00_min = 100000

        self.initial_yaw = 1000.0
        self.current_yaw = 0.0
        self.reference_yaw = 0.0
        self.image_captured = False
        self.first_loop = True

        self.capture_image = False
        self.image_count = 0

        self.look_for_target = False
        self.target_in_view = False

        self.goal = SearchGoal()
        self.client = actionlib.SimpleActionClient("/search_action_server", 
                    SearchAction)
        self.client.wait_for_server()

        self.action_complete = False

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.laser_scan = Tb3LaserScan()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.colour_identified = False
        self.target_colour = ""
        self.target_colour_index = 0

        self.masks = [
            ["red", (0,175,100), (5,255,255)],
            ["yellow", (25,150,100), (35,255,255)],
            ["green", (55,130,100), (65,255,255)],
            ["turquoise", (85,125,100), (95,255,255)],
            ["blue", (115,200,100), (125,255,255)],
            ["purple", (145,160,100), (155,255,255)]
        ]

        #self.colour_count = 6

        print("The beaconing node is active....")


    def shutdown_ops(self):
        self.vel_controller.stop()
        self.ctrl_c = True
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
        print(f"The beaconing node is shutting down...")

    
    def camera_callback(self, image_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(image_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, _ = cv_img.shape
        crop_width = width - 1500
        crop_height = 100

        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]

        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.capture_image:
            cv2.imwrite(str(image_path), hsv_img)

            self.capture_image = False
            self.image_count += 1

        if self.look_for_target:

            lower_bound = self.masks[self.target_colour_index][1]
            upper_bound = self.masks[self.target_colour_index][2]

            mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

            m = cv2.moments(mask)
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)

            if self.m00 > self.m00_min:

                self.target_in_view = True




    def find_target_colour(self, image):

        for i in range(len(self.masks)):

            lower_bound = self.masks[i][1]
            upper_bound = self.masks[i][2]

            mask = cv2.inRange(image, lower_bound, upper_bound)

            m = cv2.moments(mask)
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)

            if self.m00 > self.m00_min:

                target_index = i
                self.colour_identified = True

        return target_index


    def taking_target_pic(self):

        self.current_yaw = round(self.tb3_odom.yaw, 0)

        if self.first_loop and self.current_yaw != 0.0:
            self.initial_yaw = self.current_yaw
            self.first_loop = False

        if self.initial_yaw < 0:
            self.reference_yaw = self.initial_yaw + 180
        elif self.initial_yaw >= 0:
            self.reference_yaw = self.initial_yaw - 180


        if self.current_yaw != self.reference_yaw:
            self.vel_controller.set_move_cmd(0.0, 0.3)
        elif self.current_yaw == self.reference_yaw:
            self.capture_image = True

        if self.current_yaw == self.initial_yaw and self.image_count >= 1:
            self.vel_controller.set_move_cmd(0.0, 0.0)
            self.image_captured = True

        self.vel_controller.publish()

    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled


    def search_for_beacon(self):
    
        self.send_goal(velocity = 0.2, approach = 0.8)
        prempt = False
        while self.client.get_state() < 2:
            self.rate.sleep()
        
        self.action_complete = True
        #print(f"RESULT: Action State = {self.client.get_state()}")
        #if prempt:
        #    print("RESULT: Action preempted after travelling 2 meters")
        #else:
        #    result = self.client.get_result()
        #    print(f"RESULT: closest object {result.closest_object_distance:.3f} m away "
        #            f"at a location of {result.closest_object_angle:.3f} degrees")


    def beaconing(self):

        a = 1
    
    def main(self):
        while not self.ctrl_c:

            while not self.image_captured:
                self.taking_target_pic()

            target_colour_image = cv2.imread(f'{image_path}')

            while not self.colour_identified:
                self.target_colour_index = self.find_target_colour(target_colour_image)
                self.target_colour = self.masks[self.target_colour_index][0]

            if self.image_captured and self.colour_identified:
                print(f"SEARCH INITIATED: The target beacon colour is {self.target_colour}.")
                while not self.target_in_view:
                    self.search_for_beacon()
                print("TARGET DETECTED: Beaconing initiated.")
                self.beaconing()

if __name__ == '__main__':
    beacon_instance = Beaconing()
    try:
        beacon_instance.main()
    except rospy.ROSInterruptException:
        pass
