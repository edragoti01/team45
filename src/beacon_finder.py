#! /usr/bin/env python3

from ast import Str
from xml.etree.ElementTree import tostring
import rospy

from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

import sys
import argparse

base_image_path = Path("/home/student/catkin_ws/src/team45/src/snaps/")

beacon_found = False

class beacon_finder(object):

    def __init__(self):
        node_name = "beacon_finder"
        rospy.init_node(node_name)

        topic_name = "/camera/color/image_raw"
        self.camera_sub = rospy.Subscriber(topic_name,
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        cli = argparse.ArgumentParser()

        cli.add_argument("-target_colour", metavar="COL", type=str, default="blue")

        self.args = cli.parse_args(rospy.myargv()[1:])

        self.rate = rospy.Rate(5) # hz

        self.m00 = 0
        self.m00_min = 100000

        self.is_red = False
        self.is_yellow = False
        self.is_green = False
        self.is_blue = False

        print("The find beacon node is active....")

    def shutdown_ops(self):
        cv2.destroyAllWindows()
        self.ctrl_c = True
        print(f"The find beacon node is shutting down...")

    def save_image(self, img):
        
        full_image_path = base_image_path.joinpath("the_beacon.jpg")

        cv2.imwrite(str(full_image_path), img)
        print(f"Saved an image of a {self.target_colour} beacon to '{full_image_path}")

    def camera_callback(self, img_data):
        global beacon_found

        #Thresholds for [red, yellow, green, blue]

        #There is blue in every image, so for blue, mask out all colours and test
        #for a blank image

        lower_bound = [(0,115,100), (17,116,100), (77,113,100), (96,150,100)]

        upper_bound = [(7,240,255), (27,245,255), (87,255,255), (104,255,255)]

        #sim_lower_bound = [(0,185,100), (25,100,100), (55,150,100), (115,200,100)]

        #sim_upper_bound = [(10,255,255), (35,255,255), (65,255,255), (130,255,255)]

        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, _ = cv_img.shape
        crop_width = width - 1500
        crop_height = 100

        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        self.target_colour= self.args.target_colour

        if self.target_colour == "red":
            mask = cv2.inRange(hsv_img, lower_bound[0], upper_bound[0])

        elif self.target_colour == "yellow":
            mask = cv2.inRange(hsv_img, lower_bound[1], upper_bound[1])

        elif self.target_colour == "green":
            mask = cv2.inRange(hsv_img, lower_bound[2], upper_bound[2])

        elif self.target_colour == "blue":
            red_mask = cv2.inRange(hsv_img, lower_bound[0], upper_bound[0])
            yellow_mask = cv2.inRange(hsv_img, lower_bound[1], upper_bound[1])
            green_mask = cv2.inRange(hsv_img, lower_bound[2], upper_bound[2])
            blue_mask = cv2.inRange(hsv_img, lower_bound[3], upper_bound[3])

            if self.is_img_blue(crop_img, red_mask, yellow_mask, green_mask):
                mask = blue_mask

        m = cv2.moments(mask)

        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

            self.save_image(cv_img)

            cv2.imshow("cropped image", crop_img)
            cv2.waitKey(1)

            beacon_found = True

    #There is blue in every image, so for blue, mask out all colours and test
    #for a blank image
    def is_img_blue(self, img, red_mask, yellow_mask, green_mask):

        m = cv2.moments(red_mask)
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(img, (int(self.cy), 200), 10, (0, 0, 255), 2)

            self.is_red = True

            cv2.imshow("red image", img)
            cv2.waitKey(1)


        m = cv2.moments(yellow_mask)
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(img, (int(self.cy), 200), 10, (0, 0, 255), 2)

            self.is_yellow = True

            cv2.imshow("yellow image", img)
            cv2.waitKey(1)


        m = cv2.moments(green_mask)
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            
            self.is_green = True

            cv2.imshow("green image", img)
            cv2.waitKey(1)


        if self.is_red or self.is_yellow or self.is_green:

            self.is_blue = False
        
        else:

            self.is_blue = True

        return self.is_blue

    
    def main(self):
        global beacon_found

        while not self.ctrl_c:

            if beacon_found:
                self.ctrl_c = True

if __name__ == '__main__':
    search_instance = beacon_finder()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
