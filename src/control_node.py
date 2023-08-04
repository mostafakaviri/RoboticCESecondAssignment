#!/usr/bin/python3

import rospy
import tf

from second.msg import dis_dir

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import radians
import math

class Controller :

    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)

        self.laser_subscriber = rospy.Subscriber("/ClosestObstacle" , dis_dir , callback=self.callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        self.angular_speed = .2
        self.velocity = .5

        self.closest_distance = 0
        self.closest_direction = 3

        self.totat_rotation = 0

        self.initial_angel = 0
        self.goal_rotation = 0

        self.GO, self.ROTATE = 0, 1
        self.state = self.GO
        rospy.sleep(1)
     
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        ))

        return yaw

    def callback(self, msg : dis_dir) :

        self.closest_distance = msg.distance
        self.closest_direction = msg.direction

    def run(self) :

        while not rospy.is_shutdown() :

            # print(self.closest_distance)
            # print(self.get_heading())
            # print(self.totat_rotation)

            if self.state == self.GO :

                twist = Twist()

                twist.linear.x = self.velocity

                self.cmd_publisher.publish(twist)

                # print( self.closest_direction)
                print(self.closest_distance)
                print(self.closest_direction)

                if self.closest_distance <= 2 and (self.closest_direction < .5 * math.pi or self.closest_direction > 1.5 * math.pi) :

                    print("an")

                    if self.closest_direction < math.pi :

                        self.goal_rotation = -self.closest_direction + math.pi

                    else :

                        self.goal_rotation = -self.closest_direction + 3*math.pi

                    self.totat_rotation = 0

                    self.initial_angel = self.get_heading()

                    self.cmd_publisher.publish(Twist())

                    self.state = self.ROTATE
            

            if self.state == self.ROTATE :

                # print(self.totat_rotation)

                if abs(self.totat_rotation - self.goal_rotation) < .1 :

                    self.cmd_publisher.publish(Twist())
                                       
                    self.state = self.GO

                    continue 

                # print(abs(self.totat_rotation - self.goal_rotation))
                twist = Twist()

                twist.angular.z = self.angular_speed

                self.cmd_publisher.publish(twist)

                if self.initial_angel >= 0 :
                    if self.get_heading() > 0 :
                        self.totat_rotation = abs(self.get_heading() - self.initial_angel)
                    if self.get_heading() < 0 :
                        self.totat_rotation = 2 * math.pi + self.get_heading() - self.initial_angel
                
                else :
                    if self.get_heading() < 0 :
                        self.totat_rotation = abs(self.get_heading() - self.initial_angel)
                    if self.get_heading() > 0 :
                        self.totat_rotation = 2 * math.pi - self.get_heading() + self.initial_angel
                # self.initial_angel = self.get_heading()
                # rospy.sleep(1)
                


if __name__ == "__main__":

    controller = Controller()
    
    controller.run()

