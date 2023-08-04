#!/usr/bin/python3
import rospy

from sensor_msgs.msg import LaserScan

from second.msg import dis_dir

from math import radians

import math

global perception_publisher
def callback(msg: LaserScan):

    value = min(msg.ranges)
    index = msg.ranges.index(min(msg.ranges))
    sent_thing = dis_dir()
    sent_thing.distance = value
    sent_thing.direction = 2 * math.pi - msg.angle_increment * index
    perception_publisher.publish(sent_thing)




if __name__ == '__main__':
    
    rospy.init_node('sensor' , anonymous=False)
    perception_publisher = rospy.Publisher("/ClosestObstacle" , dis_dir , queue_size=10)
    rospy.Subscriber("/scan" , LaserScan , callback=callback)
    rospy.spin()
