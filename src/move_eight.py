#!/usr/bin/env python3

from turtle import circle
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi
#imports
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

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

        

    def __init__(self):
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided

        self.StartTime = 0

        node_name = "move_eight"
        
        self.startup = True
        self.turn = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz

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

    def shutdownhook(self):
       self.pub.publish(Twist())
       self.ctrl_c = True

    def print_stuff(self, a_message):
        print(a_message)
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")

    def main_loop(self):
        current_time = rospy.get_rostime().secs 
        #self.StartTime = rospy.get_rostime().secs 
        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
                #status = "init"
            else: 
                
                if abs(rospy.get_rostime().secs  - self.StartTime) <= (35):
                                         
                    # specify the radius of the circle:
                    path_rad = 0.5 # m
                    # linear velocity must be below 0.26m/s:
                    lin_vel = 0.1 # m/s
                    self.vel = Twist()
                    self.vel.linear.x = lin_vel
                    #clockwise directiom
                    self.vel.angular.z = lin_vel / path_rad # rad/s
                    #self.circle1= True
                    # current_time += (2*pi*path_rad)/lin_vel
                else :            
                    # specify the radius of the circle:
                    path_rad = 0.5# m
                    # linear velocity must be below 0.26m/s:
                    lin_vel = 0.1 # m/s
                    self.vel = Twist()
                    self.vel.linear.x = lin_vel     
                    #antclockwise directiom
                    self.vel.angular.z = -(lin_vel / path_rad) # rad/s
                    #w#self.StartTime += (2*pi*path_rad)/lin_vel
        


            self.pub.publish(self.vel)
            #self.print_stuff(self.vel.angular.z)
            #print('test print of angular: ', self.vel.angular)
            #print(self.theta_z0)
            self.rate.sleep()
           

if __name__ == '__main__':
    eight_instance  = Circle()
    try:
        eight_instance.main_loop()
    except rospy.ROSInterruptException:
        pass