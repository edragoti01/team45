#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class Circle:


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

        #if the callback function hasn't run before, initialise the odometry values
        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

        

    def __init__(self):

        node_name = "move_eight"
        
        self.startup = True
        self.turn = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the 'move_circle' node is active...")
        print(f"x = {self.vel.linear.x:.2f}, y = {self.vel.linear.y:.2f}, yaw = {self.vel.angular.z:.2f} [degrees]")

    def shutdownhook(self):
       self.pub.publish(Twist())
       rospy.loginfo("the 'move_circle' node is shutting down...")
       self.ctrl_c = True

    def print_info(self):
        print(f"x = {self.x:.2f} [m], y = {self.y:.2f} [m], yaw = {self.theta_z:.2f} [degrees]")

    def main_loop(self):
        reference_time = 0
        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
            else: 

                current_time = rospy.get_rostime().secs
  
                if reference_time == 0:
                    reference_time = current_time

                run_time = current_time - reference_time

                if run_time <= (29.5):

                    # specify the radius of the circle:
                    path_rad = 0.5 # m

                    # linear velocity must be below 0.26m/s:
                    lin_vel = 0.11 # m/s
                    self.vel = Twist()
                    self.vel.linear.x = lin_vel

                    #clockwise direction
                    self.vel.angular.z = lin_vel / path_rad # rad/s

                elif run_time <= (30):

                    #stop turning for half a second and go faster
                    self.vel.linear.x = 0.115 # m/s
                    self.vel.angular.z = 0.0 # rad/s

                elif run_time <= (60):

                    # specify the radius of the circle:
                    path_rad = 0.5 # m

                    # linear velocity must be below 0.26m/s:
                    lin_vel = 0.11 # m/s
                    self.vel = Twist()
                    self.vel.linear.x = lin_vel 

                    #antclockwise direction                  
                    self.vel.angular.z = -(lin_vel / path_rad) # rad/s

                else:
                    # Once figure of eight is complete, stop the robot
                    self.vel.linear.x = 0.0 # m/s
                    self.vel.angular .z = 0.0 # rad/s
                                
            self.pub.publish(self.vel)
            self.print_info.rate = rospy.Rate(1) #hz
            self.print_info()
            self.rate.sleep()

if __name__ == '__main__':
    eight_instance  = Circle()
    try:
        eight_instance.main_loop()
    except rospy.ROSInterruptException:
        pass