#!/usr/bin/env python3

import rospy
from pathlib import Path

import numpy as np

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

base_image_path = Path("/home/student/catkin_ws/src/team45/src/snaps/")


class Beaconing(object):

    def __init__(self):
        node_name = "beaconing_node"
        rospy.init_node(node_name)

        topic_name = "/camera/rgb/image_raw"
        self.camera_sub = rospy.Subscriber(topic_name,
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.laser_scan = Tb3LaserScan()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5) # hz

        self.m00 = 0
        self.m00_min = 100000

        self.initial_yaw = 1000.0
        self.current_yaw = 0.0
        self.reference_yaw = 0.0
        self.image_captured = False

        self.capture_image = False
        self.image_count = 0

        #self.is_red = False
        #self.is_yellow = False
        #self.is_green = False
        #self.is_turquoise = False
        #self.is_blue = False
        #self.is_purple = False

        #OR

        self.target_colour = ""

        self.masks = [
            ["red", (0,175,100), (5,255,255)],
            ["yellow", (25,150,100), (35,255,255)],
            ["green", (55,130,100), (65,255,255)],
            ["turquoise", (85,125,100), (95,250,255)],
            ["blue", (115,200,100), (125,255,255)],
            ["purple", (145,160,100), (155,255,255)]
        ]

        #self.colour_count = 6

        print("The beaconing node is active....")


    def shutdown_ops(self):
        self.vel_controller.stop()
        self.ctrl_c = True
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
            self.save_image(hsv_img)
            self.capture_image = False
            self.image_count += 1


    def find_target_colour(self, image):

        for i in range(len(self.masks)):

            lower_bound = mask[i][1]
            upper_bound = mask[i][2]

            mask = cv2.inRange(image, lower_bound, upper_bound)

            m = cv2.moments(mask)
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(image, (int(self.cy), 200), 10, (0, 0, 255), 2)

                self.target_colour = mask[i][0]

                cv2.imshow("masked image", image)
                cv2.waitKey(1)
    

    def save_image(self, image):
        image_name ="task4_target_colour"
        full_image_path = base_image_path.joinpath(f"{image_name}.jpg")

        cv2.imwrite(str(full_image_path), image)


    def search_for_beacon(self):

        a = 1
        #Needed?


    def beaconing(self):

        a = 1
    
    def main(self):
        first_loop = True

        while not self.ctrl_c:

            while not self.image_captured:

                self.current_yaw = round(self.tb3_odom.yaw, 0)
                print(f"Current = {self.current_yaw}")
                print(f"Image count = {self.image_count}")

                if first_loop and self.current_yaw != 0.0:
                    self.initial_yaw = self.current_yaw
                    first_loop = False

                if self.initial_yaw < 0:
                    self.reference_yaw = self.initial_yaw + 180
                elif self.initial_yaw >= 0:
                    self.reference_yaw = self.initial_yaw - 180

                if self.current_yaw != self.reference_yaw:
                    self.vel_controller.set_move_cmd(0.0, 0.2)
                elif self.current_yaw == self.reference_yaw:
                    self.capture_image = True

                
                if self.current_yaw == self.initial_yaw and self.image_count >= 1:
                    self.vel_controller.set_move_cmd(0.0, 0.0)
                    self.image_captured = True

                self.vel_controller.publish()







if __name__ == '__main__':
    beacon_instance = Beaconing()
    try:
        beacon_instance.main()
    except rospy.ROSInterruptException:
        pass
