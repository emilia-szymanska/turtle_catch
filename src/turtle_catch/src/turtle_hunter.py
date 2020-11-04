#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn

def get_distance(self, goal_x, goal_y):
    distance = sqrt(pow((goal_x - self.pose_x), 2) + (goal_y - self.pose_y), 2)) 
    return distance

def get_ang_distance(self, goal_x, goal_y):
    ang_distance = (atan2(goal_y - self.pose_y, goal_x - self.pose_x) - self.pose_theta) 
    return ang_distance

def x(req):
#    def handle_add_two_ints(req):
#   9     print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
#  10     return AddTwoIntsResponse(req.a + req.b)
#  11 
#  12 def add_two_ints_server():
#  13     rospy.init_node('add_two_ints_server')
#  14     s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)i
#  '{x: 2.0, y: 2.0, theta: 0.0, name: "emilka"}'
    print("meeeh")
    return '{x: 2.0, y: 2.0, theta: 0.0, name: "emilka"}'
    
    #print(req)
    #return SpawnResponse(req.x, req.y, req.z, req.name)

def x():
    print("ajajaj")


def spawn_turtle():
    rospy.init_node('spawn_turtle')
    print("ehhh")
    service = rospy.Service('spawn_turtle', Spawn, x)
    print("ahhh")
    rospy.spin()

if __name__ == "__main__":
    spawn_turtle()
