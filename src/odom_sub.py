#!/usr/bin/env python3

import rospy
# import Odometry from the nav_msgs package instead:
from nav_msgs.msg import Odometry
# additionally, import the "euler_from_quaternion" function from the tf library
# for converting the raw orientation values from the odometry message:
from tf.transformations import euler_from_quaternion

class Subscriber:

    def callback(self, odom_data):

        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y


    def __init__(self):
        # in the initialisation, the node name can be changed, but this isn't essential:
        rospy.init_node('odom_subscriber_node', anonymous=True)
        # When setting up the subscriber, the "odom" topic needs to be specified
        # and the message type (Odometry) needs to be provided
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        # an optional status message:
        rospy.loginfo("odom subscriber is active...")
        self.counter = 0

    def main_loop(self):
        # set the node to remain active until closed manually:
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()