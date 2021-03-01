#!/usr/bin/env python3

import rospy
import time
from math import sqrt, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

def get_orientation(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    if yaw < 0:
            yaw = 2 * pi + yaw
    rospy.loginfo(yaw)


def main():
    rospy.init_node('make_square')

    rospy.Subscriber('/odom', Odometry, get_orientation)

    rospy.spin()

if __name__ == "__main__":
    main()






# Old code
# rospy.init_node('publisher')

# publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# rotate_right = True
# iterations = 0

# while not rospy.is_shutdown() and iterations < 5:
#     msg = Twist()    
#     msg.linear.x = 0.2
#     publisher.publish(msg)
#     time.sleep(10)
#     msg.linear.x = 0
#     msg.angular.z = 0 if iterations == 0 or iterations == 4 else 0.5236
#     publisher.publish(msg)
#     time.sleep(3)
#     msg.angular.z = 0
#     publisher.publish(msg)
#     iterations += 1
#       
#   rosrun q1 track_robot.py __name:='beacon_0' _beacon_id:='beacon_0' _cord_x:=1 _cord_y:=1
#   rosrun q1 track_robot.py __name:='beacon_1' _beacon_id:='beacon_1' _cord_x:=1 _cord_y:=1
#   rosrun q1 track_robot.py __name:='beacon_2' _beacon_id:='beacon_2' _cord_x:=1 _cord_y:=1
#