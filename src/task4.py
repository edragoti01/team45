#!/usr/bin/env python3

import rospy
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from tb3 import Tb3Move

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

#from nav_msgs.msg import Odometry


class BeaconSearch(object):


    def __init__(self):


    def shutdownops(self):

    
    def find_target_colour(self):
        node_name = "task4_"
        rospy.init_node(node_name)

        #Actual robot topic_name = "/camera/color/image_raw"

        topic_name = "/camera/rgb/image_raw"
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

        print("The find beacon node is active....")

    
    def beaconing(self):







