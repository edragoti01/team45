#! /usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback
from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback



        #self.masks = [
        #    ["red", (0,175,100), (5,255,255)],
        #    ["yellow", (25,150,100), (35,255,255)],
        #    ["green", (55,130,100), (65,255,255)],
        #    ["turquoise", (85,125,100), (95,250,255)],
        #    ["blue", (115,200,100), (125,255,255)],
        #    ["purple", (145,160,100), (155,255,255)]
        #]

        #self.colour_count = 6

        #print("The beacon colour search node is active....")

class BeaconActionClient(object):
   
    def __init__(self):
        node_name = "beacon_action_client"
        rospy.init_node(node_name)
        self.rate = rospy.Rate(1)

        self.cam_action_complete = False
        self.search_action_complete = False

        self.captured_images = 0

        self.cam_goal = CameraSweepGoal()

        self.cam_client = actionlib.SimpleActionClient("/search_action_server", 
                    CameraSweepAction)

        self.cam_client.wait_for_server()

        #self.search_goal = SearchGoal()

        #self.search_client = actionlib.SimpleActionClient("/search_action_server", 
                    #SearchAction)
        #self.search_client.wait_for_server()




        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        if not self.cam_action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goals...")
            self.cam_client.cancel_goal()
            rospy.logwarn("Goals Cancelled")


    def send_cam_goal(self, images, angle):

        self.cam_goal.sweep_angle = angle
        self.cam_goal.image_count = images
        
        # send the goal to the action server:
        self.cam_client.send_goal(self.cam_goal, feedback_cb=self.cam_feedback_cb)


    def cam_feedback_cb(self, feedback_data: CameraSweepFeedback):
        self.captured_images = feedback_data.current_image
        print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
            f"Image(s) captured so far: {self.captured_images}...")
        img_limit = 360
        if self.captured_images >= img_limit:
            rospy.loginfo(f"{self.captured_images} image(s) captured, "
                        f"cancelling the goal...")
            self.cam_client.cancel_goal()
            rospy.loginfo("Goal Cancelled.")

    def main(self):

        self.send_cam_goal(images = 1, angle = 360.0)

        while self.cam_client.get_state() < 2:
            print(f"STATE: Current camera server state code is {self.cam_client.get_state()}")

            self.rate.sleep()
        
        self.cam_action_complete = True
        print(f"RESULT: Action State = {self.cam_client.get_state()}")
        print(f"RESULT: {self.captured_images} image(s) saved.")

if __name__ == '__main__':
    client_instance = BeaconActionClient()
    try:
        client_instance.main()
    except rospy.ROSInterruptException:
        pass
