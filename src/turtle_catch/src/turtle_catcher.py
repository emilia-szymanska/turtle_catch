#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, pi
from numpy import sign

class TurtleCatcher:
    def __init__(self):
        self.vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        
        self.pose = Pose()

        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0
        self.pose.linear_velocity = 0
        self.pose.angular_velocity = 0

        self.catcher = rospy.Subscriber('pose', Pose, self.pose_callback) 
        self.prey = rospy.Subscriber('turtle1/pose', Pose, self.catching_callback) 

    def pose_callback(self, msg_received):
        self.pose.x = msg_received.x
        self.pose.y = msg_received.y
        self.pose.theta = msg_received.theta
        self.pose.linear_velocity = msg_received.linear_velocity
        self.pose.angular_velocity = msg_received.angular_velocity
        

    def catching_callback(self, msg_received):
#        rospy.loginfo("Received something: x = " + str(msg_received.x) + " y = " + str(msg_received.y))
        x = msg_received.x
        y = msg_received.y

        new_theta = self.get_ang_distance(x, y)
        
        msg = Twist()
        msg.linear.x = self.cmd_vel.linear.x
        msg.linear.y = self.cmd_vel.linear.y
        msg.linear.z = self.cmd_vel.linear.z

        msg.angular.x = self.cmd_vel.angular.x
        msg.angular.y = self.cmd_vel.angular.y
       
        alfa = self.pose.theta
        beta = new_theta

        if alfa * beta > 0:
            ang_diff = beta - alfa
        elif abs(alfa) + abs(beta) < pi:
            ang_diff = sign(beta) * (abs(alfa) + abs(beta))
        else:
            ang_diff = sign(beta) * (-2 * pi + (abs(alfa) + abs(beta)))
            
        msg.angular.z = ang_diff
        self.cmd_vel.angular.z = ang_diff

        self.vel.publish(msg)        

        rospy.loginfo("Theta: " + str(new_theta))

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2)) 
        return distance

    def get_ang_distance(self, goal_x, goal_y):
        ang_distance = atan2(goal_y - self.pose.y, goal_x - self.pose.x) #- self.pose.theta) 
        return ang_distance

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    rospy.init_node('turtle_catcher')

   # try:
    catcher = TurtleCatcher()
    catcher.run()
