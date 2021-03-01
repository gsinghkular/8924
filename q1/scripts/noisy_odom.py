#!/usr/bin/env python3

import rospy
import numpy as np
from math import sqrt
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class NoisyOdom:
    def __init__(self, pub, sigma):
        self._pub = pub
        self.first_run = True
        self.previous_x = 0
        self.previous_y = 0
        self.total_noise = 0
        self._sigma = sigma

    def callback(self, msg):
        # add the calculate noise to (x, y, theta) - initial value be 0 (from constructor)
        msg.pose.pose.position.x += self.total_noise
        msg.pose.pose.position.y += self.total_noise
        msg.pose.pose.orientation.z += self.total_noise
        self._pub.publish(msg)

        # calculate distance travelled since last message
        distance = self.distance(msg)
        # calculate noise and accumulate it to total noise
        self.total_noise += np.random.normal(0, distance*self._sigma)
    
    # helper method to calculate distance from Odom data (prev and current)
    def distance(self, msg):
        if(self.first_run):
            self.previous_x = msg.pose.pose.position.x
            self.previous_y = msg.pose.pose.position.y
            self.first_run = False
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist = sqrt((x - self.previous_x) * (x - self.previous_x) + (y - self.previous_y) * (y - self.previous_y))
        self.previous_x = msg.pose.pose.position.x
        self.previous_y = msg.pose.pose.position.y
        return dist

def main():
    rospy.init_node("odom_monitor")

    # get parameter(s)
    sigma = rospy.get_param("~sigma", 0.1)
    
    # create a publisher to publish to noise odom
    pub = rospy.Publisher("/noisy_odom", Odometry, queue_size=10)

    # create an instance of handler class, with pub and sigma values
    ins = NoisyOdom(pub, sigma)
    rospy.Subscriber("/odom", Odometry, ins.callback)
    rospy.spin()

if __name__ == "__main__":
    main()