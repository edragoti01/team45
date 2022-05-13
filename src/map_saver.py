#!/usr/bin/env python3

import rospy
import roslaunch


map_path = "/home/student/catkin_ws/src/team45/maps/"

rospy.init_node("map_saver", anonymous=True)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

launch_status = True
print(f"Launch status: {launch_status}")

wait = 0

while launch_status:
    if (wait%10) == 0:

        print(f"Saving map at time: {rospy.get_time()}...")

        node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=f"-f {map_path}")

        process = launch.launch(node)

    wait+=1