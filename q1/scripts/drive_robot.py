#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from math import sqrt, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# helper method determine orientation change
def orientation_change(prev, new, ang_vel):
    change = abs(new - prev)
    if change < ang_vel:
        return change
    else:
        return 2 * pi - change #if change is more than angular velocity that means angle started from 0 again, i.e. it went from 2pi to 0

# helper method to get orientation angle from quaternion
def get_orientation(msg):
    orientation = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    # make -pi:pi to 0:2pi
    if yaw < 0:
        yaw = 2 * pi + yaw
    return yaw

class MoveRobot:
    def __init__(self, publisher):
        self._publisher = publisher
        self._init_odom = None
        self._distance_travelled = 0
        self._count = 0
        self._prev_orientation = 0          # orientation in last odom message
        self._total_orientation = 0         # total orientation robot has changed so far
        self._turn_upto = 0                 # angle upto which robot needs to turn to
        self._turn = False                  # Boolean value to hold when to turn
        self._x = []                        # list to store values of x from odom
        self._y = []                        # list to store values of y from odom

    def plot(self):
        # plot the odom data
        plt.plot(self._x, self._y)
        plt.xlabel('x')
        plt.ylabel('y') 
        plt.title('Odom Data') 
        plt.show()

    def callback(self, msg):
        # check if required motion commands has been completed 4 times
        if self._count == 4:
            # command robot to stop
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self._publisher.publish(twist)
            rospy.signal_shutdown("motion complete")

        # init default values on first message
        if self._init_odom is None:
            self._prev_orientation = get_orientation(msg)
            self._turn_upto = pi/2
            self._init_odom = msg

        # update robots current distance and orientation change since last message
        self.update_distance(msg)
        self.update_total_orientation(msg)

        # keep moving robot if it has travelled less than 2m (0.2m/s * 10s = 2m)
        if self._distance_travelled < 2:
            twist = Twist()
            twist.linear.x = 0.2
            self._publisher.publish(twist)
        # after robot has moved 2m, turn it 90 degrees
        elif self._turn and self._total_orientation < self._turn_upto:
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = -30 * pi / 180      # rotation speed of 30deg/s
            self._publisher.publish(twist)
        # once robot has moved 2meters and turned 90deg, then reset the move settings and increase counter
        else:
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self._distance_travelled = 0
            self._turn_upto += pi/2
            self._count += 1
            self._init_odom = msg
            self._publisher.publish(twist)

    # helper method to update distance travelled by robot
    def update_distance(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._x.append(x)
        self._y.append(y)
        init_x = self._init_odom.pose.pose.position.x
        init_y = self._init_odom.pose.pose.position.y
        self._distance_travelled = sqrt((x - init_x) * (x - init_x) + (y - init_y) * (y - init_y))
        # if the robot has travelled for 2m, then it needs to turn
        self._turn = True if self._distance_travelled >= 2 else False

    def update_total_orientation(self, msg):
        curr = get_orientation(msg)
        self._total_orientation += orientation_change(self._prev_orientation, curr, 30*pi/180)
        self._prev_orientation = curr

def main():
    rospy.init_node('make_square')

    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    move_robot = MoveRobot(publisher)
    rospy.Subscriber('/noisy_odom', Odometry, move_robot.callback)

    rospy.on_shutdown(move_robot.plot)
    rospy.spin()

if __name__ == "__main__":
    main()
