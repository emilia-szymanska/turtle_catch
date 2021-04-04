#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
import sys


def spawn_turtle_client(x, y, theta, name):
    rospy.wait_for_service('spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        new_turtle = spawn_turtle(x, y, theta, name)
        return new_turtle
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node("spawn_turtle")
    if len(sys.argv) != 5:
        x = 2
        y = 2
        theta = 0
        turtle_name = "hunter"
    else:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3])
        turtle_name = sys.argv[4]

    print("Spawning turtle %s in positions x = %s, y = %s, theta = %s"%(turtle_name, str(x), str(y), str(theta)))
    spawn_turtle_client(x, y, theta, turtle_name)
