#!/usr/bin/env python3

import rospy
import message_filters
import math
from messages.msg import Position
from ast import literal_eval as make_tuple

# helper method to parse coordinates and distance data from message
def parse_message(data):
    data_str = str(data)
    s = data_str.find("location ")
    e = data_str.find(" distance")
    (x, y) = make_tuple(data_str[s+9:e])
    d = float(data_str[e+9:-1])
    return (x, y, d)

# helper method to calculate intersection of 3 circles
def get_intersections(x0, y0, r0, x1, y1, r1, x2, y2, r2):
    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)

    a=(r0**2-r1**2+d**2)/(2*d)
    h=math.sqrt(r0**2-a**2)
    x2=x0+a*(x1-x0)/d   
    y2=y0+a*(y1-y0)/d   
    x3=x2+h*(y1-y0)/d     
    y3=y2-h*(x1-x0)/d 

    x4=x2-h*(y1-y0)/d
    y4=y2+h*(x1-x0)/d
    
    #compare to third circle
    final1= abs(math.sqrt(math.pow((x3-x2),2)+(math.pow((y3-y2),2)))-r2)
    final2= abs(math.sqrt(math.pow((x4-x2),2)+(math.pow((y4-y2),2)))-r2)

    if final1 < final2:
      return (x3,y3)
    else:
      return (x4,y4)

def callback(dat0, dat1, dat2):
    (x0, y0, r0) = parse_message(dat0.position)
    (x1, y1, r1) = parse_message(dat1.position)
    (x2, y2, r2) = parse_message(dat2.position)

    # estimate robot's position from the intersection of 3 circles
    (x, y) = get_intersections(x0, y0, r0, x1, y1, r1, x2, y2, r2)
    rospy.loginfo("est: {} {}".format(x,y))

def main():
    rospy.init_node('estimate_odom')
    # create subscribers to messages from beacons
    b1_sub = message_filters.Subscriber("/beacon_0/range", Position)
    b2_sub = message_filters.Subscriber('/beacon_1/range', Position)
    b3_sub = message_filters.Subscriber('/beacon_2/range', Position)

    # syncrhonize the messages
    ts = message_filters.TimeSynchronizer([b1_sub, b2_sub, b3_sub], 10)

    ts.registerCallback(callback)

    rospy.spin()
  
if __name__ == '__main__':
    main()

