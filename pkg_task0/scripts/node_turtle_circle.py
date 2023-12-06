#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from math import pi


class TurtleCircle():
    '''This class contains method to move the turtle in a circle.'''

    def __init__(self):
        '''Constructor method'''
        # A subscriber to the topic '/turtle1/pose'. 'pose_callback'
        # is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.pose_callback)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # Call service to change the turtle pen
        self.set_pen_service = rospy.ServiceProxy(
            '/turtle1/set_pen', SetPen)

        self.set_pen_service.call(255, 0, 0, 7, 0)
        self.pose = Pose()
        self.twist = Twist()
        self.previous_theta = -1
        self.flag_circle_done = False
        self.rate = rospy.Rate(10)  # Rate in Hz

    def pose_callback(self, msg):
        '''This method runs when a new pose is received.'''
        self.pose = msg

        # Changes (-pi, pi) to (0, 2*pi)
        self.pose.theta += 2*pi if self.pose.theta < 0 else 0
        # rospy.loginfo(self.pose)

    def make_circle(self, lin_x, ang_z):
        '''This method will move the turtle in a circle.'''
        self.twist.linear.x = lin_x
        self.twist.angular.z = ang_z
        # Move the turtle
        self.velocity_publisher.publish(self.twist)
        self.rate.sleep()
        # Stop when the circle is finished
        if self.pose.theta < self.previous_theta:
            rospy.loginfo("Circle done!")
            self.flag_circle_done = True
        self.previous_theta = self.pose.theta


def main():
    # Creates a node with name 'node_turtle_circle' and make
    # sure it is a unique node (using anonymous=True)
    rospy.init_node('node_turtle_circle', anonymous=True)

    tc = TurtleCircle()

    # Wait until Ctrl+C
    while not rospy.is_shutdown():
        if not tc.flag_circle_done:
            tc.make_circle(0.5, 0.3)


if __name__ == '__main__':
    main()
