#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
import sys

#def add_two_ints_client(x, y):
#    rospy.wait_for_service('add_two_ints')
#       try:
#           add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
#           return resp1.sum
#       except rospy.ServiceException as e:
#           print("Service call failed: %s"%e)
   
#def usage():
#    return "%s [x y]"%sys.argv[0]
   
#   if __name__ == "__main__":
       #if len(sys.argv) == 3:
#           x = int(sys.argv[1])
#           y = int(sys.argv[2])
#       else:
#           print(usage())
#           sys.exit(1)
#       print("Requesting %s+%s"%(x, y))
#       print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))


def spawn_turtle_client():
    rospy.wait_for_service('spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        new_turtle = spawn_turtle(2, 2, 0, "emilka")
        return new_turtle
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    spawn_turtle_client()
