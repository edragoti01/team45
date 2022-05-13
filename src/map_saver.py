#!/usr/bin/env python3

import rospy
import roslaunch

map_path = "team45/maps/task5_map"

rospy.init_node("map_saver", anonymous=True)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start

launch_status = True

wait = 0

while launch_status:
    if (wait%10) == 0:

        print(f"Saving map at time: {rospy.get_time()}...")

        node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=f"-f {map_path}")

        process = launch.launch(node)

    wait+=1