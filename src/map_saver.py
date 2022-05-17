#!/usr/bin/env python3

import rospy
import roslaunch


launch = roslaunch.scriptapi.ROSLaunch()

map_path = "/home/student/catkin_ws/src/team45/maps/task5"

class map_saver(object):

    def __init__(self):
        node_name = "map_saver"
        rospy.init_node(node_name, anonymous=True)
        
        launch.start()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5) # hz

        print("The map saver node is active....")
    
    def shutdown_ops(self):
        self.ctrl_c = True
        print(f"The map saver node is shutting down...")

    def main(self):

        while not self.ctrl_c:
            rospy.sleep(5) # seconds

            print(f"Saving map at time: {rospy.get_time()}...")

            node = roslaunch.core.Node(package="map_server", 
                node_type="map_saver", args=f"-f {map_path}")

            launch.launch(node)



if __name__ == '__main__':
    search_instance = map_saver()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass


