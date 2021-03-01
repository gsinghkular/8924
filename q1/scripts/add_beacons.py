#!/usr/bin/env python3

import rospy
import os

def add_beacon(id, x, y):
    os.system(f"rosrun gazebo_ros spawn_model -database construction_cone -sdf -model beacon_{id} -x {x} -y {y}")

if __name__ == '__main__':
    rospy.init_node('add_beacons')
    add_beacon(1, 1, 1)
    add_beacon(2, 5, 1)
    add_beacon(3, -5, 1)
    rospy.signal_shutdown("beacons added")