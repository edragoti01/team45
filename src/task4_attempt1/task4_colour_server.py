#!/usr/bin/env python3

import rospy
from pathlib import Path
import actionlib

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

from com2009_msgs.msg import CameraSweepFeedback, CameraSweepResult, CameraSweepAction, CameraSweepGoal

image_path = "/home/student/catkin_ws/src/team45/src/snaps/task4"
base_image_path = Path(image_path)
base_image_path.mkdir(parents=True, exist_ok=True)

class ColourActionServer(object):
    feedback = CameraSweepFeedback() 
    result = CameraSweepResult()


    def __init__(self):

        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            CameraSweepAction, self.action_server_launcher, auto_start=False)

        self.actionserver.start()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()

        self.feedback.current_image = 0
        self.ref_orx = 100000.0

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
                Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        
    def camera_callback(self, img_data):

        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        if self.tb3_odom.yaw == self.ref_orx:
        
            full_image_path = base_image_path.joinpath(f"target_colour.jpg")

            cv2.imwrite(str(full_image_path), cv_img)

            self.feedback.current_image = 0


    def action_server_launcher(self, goal: CameraSweepGoal):

        r = rospy.Rate(1) # Hz

        success = True

        if goal.image_count <= 0 or goal.sweep_angle > 360:
            print("Invalid image count value, must be at least 1.")
            success = False

        if goal.sweep_angle < 0 or goal.sweep_angle > 360:
            print("Invalid sweep value, must be between 0 and 360.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print(f"Request to take {goal.image_count} images at sweep angle {goal.sweep_angle}")

        # Get the current robot odometry:
        self.orx0 = self.tb3_odom.yaw

        self.ref_orx = -(self.orx0)

        print("The robot will start to move now...")
        # set the robot velocity:
        self.vel_controller.set_move_cmd(0.0, 0.3)
        

        while self.tb3_odom.yaw != self.ref_orx:
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
            
            # populate the feedback message and publish it:
            #self.feedback.current_image += 1
            self.feedback.current_angle = self.tb3_odom.yaw
            self.actionserver.publish_feedback(self.feedback)

        self.vel_controller.stop()

        

        if success:
            rospy.loginfo("Pictures captured successfully")
            self.result.image_path = image_path

            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()



if __name__ == '__main__':
    rospy.init_node("colour_action_server")
    ColourActionServer()
    rospy.spin()

            







