#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point
from math import atan2, pi, sqrt
from numpy import sign

DEFAULT_LIN_VEL = 1
BORDER = 0.5

class TurtleCatcher:
    def __init__(self):
        self.vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.dist = rospy.Publisher('turtles_distance', Point, queue_size = 1)
        
        self.catcher = rospy.Subscriber('pose', Pose, self.pose_callback) 
        self.prey = rospy.Subscriber('turtle1/pose', Pose, self.catching_callback) 
        
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


    def pose_callback(self, msg_received):
        self.pose.x = msg_received.x
        self.pose.y = msg_received.y
        self.pose.theta = msg_received.theta
        self.pose.linear_velocity = msg_received.linear_velocity
        self.pose.angular_velocity = msg_received.angular_velocity
        

    def catching_callback(self, msg_received):
        x = msg_received.x
        y = msg_received.y

        new_theta = self.get_ang_distance(x, y)
        dist = self.get_distance(x, y)

        if dist < BORDER:
            self.cmd_vel.angular.z = 0
            self.cmd_vel.linear.x = 0
        else:
            alpha = self.pose.theta
            beta = new_theta

            if alpha * beta > 0:
                ang_diff = beta - alpha
            elif abs(alpha) + abs(beta) < pi:
                ang_diff = sign(beta) * (abs(alpha) + abs(beta))
            else:
                ang_diff = sign(beta) * (-2 * pi + (abs(alpha) + abs(beta)))
            
            self.cmd_vel.angular.z = ang_diff
            self.cmd_vel.linear.x = DEFAULT_LIN_VEL
        
        dist_msg = Point()
        dist_msg.x = abs(x - self.pose.x)
        dist_msg.y = abs(y - self.pose.y)
        dist_msg.z = 0

        self.dist.publish(dist_msg)
        self.vel.publish(self.cmd_vel)        


    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2)) 
        return distance


    def get_ang_distance(self, goal_x, goal_y):
        ang_distance = atan2(goal_y - self.pose.y, goal_x - self.pose.x) 
        return ang_distance


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    rospy.init_node('turtle_catcher')
    
    catcher = TurtleCatcher()
    
    try:
        catcher.run()
    except rospy.ROSInterruptException:
        pass

