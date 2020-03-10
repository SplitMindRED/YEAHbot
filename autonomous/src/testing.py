#!/usr/bin/env python
import rospy
import threading
import time

import sys
import tf.transformations as tftr
from numpy import *

from geometry_msgs.msg import Pose, Point, Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from nav_msgs.msg import Odometry
# from bac_task.msg import CartesianTrajectory
# from bac_task.msg import CameraFeatures

lock = threading.Lock()

class YoubotController:

    def __init__(self):
        #self.cart_trajectory = None
        self.odometry = Odometry()
        self.next_t = 2.0
        self.CONST_VELOCITY = 0.05
        self.CONST_OMEGA = 0.15

        "ROS stuff"
        #self.trajectory_sub = rospy.Subscriber("/robotino/trajectory", CartesianTrajectory, self.cart_trajectory_callback)
        self.odometry_sub = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        #rospy.Subscriber('chatter', String, callback)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # def cart_trajectory_callback(self, msg):
    #     """
    #     Trajectory for Robotino.
    #     Gets trajectory from robotino_trajectory_generator_node and saves to self variable.
    #     """
    #     lock.acquire()
    #     self.cart_trajectory = msg
    #     lock.release()

    def odometry_callback(self, msg):
        """
        Odometry (state) for Robotino.
        Can be used for feedback for trajectory controller
        """
        lock.acquire()
        #print(msg.pose)
        self.odometry = msg
        lock.release()

    def stop(self):
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.angular.z = 0.0
        self.velocity_pub.publish(velocity)

    def send_velocity(self, Vx, Vy, Wz):
        velocity = Twist()
        velocity.linear.x = Vx
        velocity.linear.y = Vy
        velocity.angular.z = Wz
        self.velocity_pub.publish(velocity)

    def update(self, t):
        #print(t)

        #GOAL POSITION
        theta = 90 * 3.14 / 180
        theta = 0.0
        goal_pose = Point(0.0, 0.0, theta)

        #print(self.odometry.pose.pose.position)
        current_position = self.odometry.pose.pose.position
        q = self.odometry.pose.pose.orientation     # quaternion
        rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw
        # print(xyz.x, xyz.y)
        #print(self.odometry_x)

        #ERROR I WORLD COORDINATES
        x_error_world = goal_pose.x - current_position.x#self.odometry.pose.pose.position.x
        y_error_world = goal_pose.y - current_position.y#self.odometry.pose.pose.position.y
        theta_error = goal_pose.z - rpy[2]              #self.odometry.pose.pose.orientation.z
        distance = sqrt(x_error_world ** 2 + y_error_world ** 2)
        #print(rpy[2])
        print("distance: " + str(distance) + " th_er: " + str(theta_error))
        
        if theta_error >= math.pi:
            theta_error -= 2*math.pi
        elif theta_error < -math.pi:
            theta_error += 2*math.pi

        #EVALUATE VELOCITY
        Vx = x_error_world * self.CONST_VELOCITY / distance
        Vy = y_error_world * self.CONST_VELOCITY / distance
        # Vx = x_error * 0.1
        # Vy = y_error * 0.1
        if theta_error == 0.0:
            Wz = 0.0
        else:
            Wz = self.CONST_OMEGA * theta_error / abs(theta_error)

        #TRANSFORM VELOCITY TO LOCAL COORDINATES
        Vx_local = Vx * cos(-rpy[2]) - Vy * sin(-rpy[2])
        Vy_local = Vx * sin(-rpy[2]) + Vy * cos(-rpy[2])

        if distance > 0.02:
            if abs(theta_error) > 0.01:
                self.send_velocity(Vx_local, Vy_local, Wz)
                #print("distance", distance)
                #print("send vel")
            else:
                self.send_velocity(Vx_local, Vy_local, 0.0)
        else:
            if abs(theta_error) > 0.01:
                self.send_velocity(0.0, 0.0, Wz)
            else:
                self.stop()

        # if distance > 0.02:
        #     self.send_velocity(Vx_local, Vy_local, 0.2)
        # else:
        #     self.stop()


if __name__ == '__main__':
    rospy.init_node("first_python")
    rospy.loginfo("Start...")
    RATE = rospy.get_param('/rate', 50)
    dt = rospy.get_param('/dt', 0.02)
    rate = rospy.Rate(RATE)

    youbot = None
    youbot = YoubotController()
    time_start = rospy.get_time()
    while not rospy.is_shutdown():
        try:
            t = rospy.get_time() - time_start            
            youbot.update(t)
            rate.sleep()
        except rospy.ROSInterruptException as e:
            youbot.odometry_sub.unregister()
            del youbot
            print('End of programm')