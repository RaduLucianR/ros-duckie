#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped

class VelocityControllerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(VelocityControllerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub1 = rospy.Publisher('/db6/kinematics_node/velocity', Twist2DStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/db6/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    def run(self):
        #publish every 2 secs
        rate = rospy.Rate(2)
        count = 0
        while not rospy.is_shutdown():
            message1 = Twist2DStamped()
            message2 = WheelsCmdStamped()
            #message1.seq(count)
            message1.omega = 0
            if (count % 50 >= 25):  
                message1.v = 0.1
                message2.vel_left = 0.1
                message2.vel_right = 0.1
            else:
                message1.v = 0.6
                message2.vel_left = 0.6
                message2.vel_right = 0.6
            #rospy.loginfo("Publishing message1: '%s'" % message1)
            rospy.loginfo("Publishing message2: '%s'" % message2)
            self.pub1.publish(message1)
            self.pub2.publish(message2)
            count = count + 1
            rate.sleep()
        message1.v = 0
        message2.vel_left = 0
        message2.vel_right = 0
        
        

if __name__ == '__main__':
    # create the node
    node = VelocityControllerNode(node_name='velocity_controller')
    # run node
    node.run()
    # keep spinning
    rospy.spin()