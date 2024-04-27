#!/usr/bin/env python

import rospy
import random
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from scipy import linalg as lnr

class Driver(object):
    # constructor
    def __init__(self):
        self.time_save = 0
        self.name = 'cylinderRobot'
        self.mass = 10  # This was missing in your initial script

        # Define publisher object. Publish the state of robot to topic "/gazebo/set_model_state"
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        # Define subscriber object. Subscribe the signal from topic "/robot/control" sent by teleop
        rospy.Subscriber('/robot/control', Twist, self.callback_control, queue_size=10)

        # member variable saving system state.
        self.state = np.zeros([4, 1])    
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0]
        ])
        self.B = np.array([
            [0, 0],
            [1 / self.mass, 0],
            [0, 0],
            [0, 1 / self.mass]
        ])
        self.Sigma_w = np.eye(4) * 0.00001
        self.n, self.m = self.B.shape

    # callback function of subscriber
    def callback_control(self, twist):
        # print("########")
        # print(twist)
        # print("########")
        if self.time_save == 0:
            self.time_save = rospy.get_time()
        else:
            # measure the dt since the last sampling time
            dt = rospy.get_time() - self.time_save
            self.time_save = rospy.get_time()

            # get the control signal: u[0] = f_x, u[1] = f_y
            u = np.array([[twist.linear.x], [twist.angular.z]])

            # forward dynamics
            self.state = self.forward_dynamics(u, dt)
            # generate and send the robot state to Gazebo
            self.sendStateMsg()

    # forward dynamics of robot.
    def forward_dynamics(self, u, dt):
        Atilde, Btilde, Sigma_w_tilde = self._discretization_Func(dt)

        # generate the Gaussian noise
        w = np.random.multivariate_normal(np.zeros([self.n]), Sigma_w_tilde).reshape([self.n, 1])

        x = np.dot(Atilde, self.state) + np.dot(Btilde, u) #+w

        return x

    # discretization function
    def _discretization_Func(self, dt):
        Atilde = np.array([
            [1, 0, 0, 0],
            [dt, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, dt, 1]
        ])
        Btilde = np.array([
            [dt / self.mass, 0],
            [dt * dt / 2 / self.mass, 0],
            [0, dt / self.mass],
            [0, dt * dt / 2 / self.mass]
        ])
        q1 = 0.00001
        q2 = 0.00001
        q3 = 0.00001
        q4 = 0.00001
        Sigma_w_tilde = np.array([
            [dt * q1, dt * dt / 2 * q1, 0, 0],
            [dt * dt / 2 * q1, (dt * q2) + (dt * dt * dt / 3 * q1), 0, 0],
            [0, 0, dt * q3, dt * dt / 2 * q3],
            [0, 0, dt * dt / 2 * q3, (dt * q4) + (dt * dt * dt / 3 * q3)],
        ])
        return Atilde, Btilde, Sigma_w_tilde

    def sendStateMsg(self):
        msg = ModelState()
        msg.model_name = self.name
        # print(self.state)
        #msg.twist.linear.x = self.state[0]
        #msg.twist.linear.y = self.state[2]
        msg.pose.position.x = self.state[1]
        msg.pose.position.y = self.state[3]
        self.pub.publish(msg)

        
if __name__ == '__main__':
    try:
        rospy.init_node('driver', anonymous=True)
        driver = Driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass