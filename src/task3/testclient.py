#! /usr/bin/env python3

import rospy
import actionlib


from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class action_client(object):
       

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0
            print(f"FEEDBACK: Currently travelled {self.distance:.3f} m")

    def __init__(self):
        node_name = "searc client"
        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)
        self.goal = SearchGoal()
        self.client = actionlib.SimpleActionClient("/search_action_server", 
                    SearchAction)
        self.client.wait_for_server()

        self.startup = True
        self.turn = False


        self.ctrl_c = False

        rospy.on_shutdown(self.shutdown_ops) 
        self.distance = 0.0
        self.i = 0


    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")


    


    def print_stuff(self, a_message):
        print(a_message)
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")

    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(velocity = 0.1, approach = 0.4)
        while not self.goal==self.send_goal :
            self.send_goal(velocity = 0.1, approach = 0.4)
            prempt = False
            while self.client.get_state() < 3:   
                print(f"FEEDBACK: Currently travelled {self.distance:.3f} m, "
                        f"STATE: Current state code is {self.client.get_state()}")
                """
                if self.distance >= 4:
                    rospy.logwarn("Cancelling goal now...")
                    self.client.cancel_goal()
                    rospy.logwarn("Goal Cancelled")
                    prempt = True
                    break
                """
            self.rate.sleep()

            #self.rate.sleep()
            
            self.action_complete = True


            print(f"RESULT: Action State = {self.client.get_state()}")
            if prempt:
                print("RESULT: Action preempted after travelling 2 meters")
            else:
                result = self.client.get_result()

if __name__ == '__main__':
    client_instance = action_client()
    try:
        client_instance.main()
    except rospy.ROSInterruptException:
        pass