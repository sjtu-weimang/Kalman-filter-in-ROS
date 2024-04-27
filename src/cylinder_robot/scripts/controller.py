#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
from gazebo_msgs.msg import ModelState

x, y, vx, vy = 0, 0, 0, 0 # initial

idx = 1 # user input
pos = [[1.5,2.5],[3,2.5],[4.5,2.5],[1.5,0.5],[3,0.5],[4.5,0.5]]
order = pos[idx-1]

if idx <=3:
    X = [0,order[0],order[0],order[0],0,0]
    Y = [2,2,order[1],2,2,0]
else:
    X = [order[0]-0.5,order[0],order[0],order[0],order[0]-0.5,0]
    Y = [0,0,order[1],0,0,0]
print("Table ",idx, "needs order!")

class PID_Controller:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0

    def constrain(self, output):
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output

    def get_output(self, error):
        self.error = error
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error
        output = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.output = self.constrain(output)
        return self.output


def PoseCallback(state):
    global x, y, vx, vy
    x = state.pose.position.x  
    y = state.pose.position.y
    vx = state.twist.linear.x
    vy = state.twist.linear.y

def controller():
    pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)
    # rospy.Subscriber('gazebo/set_model_state', ModelState, PoseCallback)
    rospy.Subscriber('/robot/esti_model_state', ModelState, PoseCallback)
    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(10)

    cmd_vel = Twist()
    move_er_threshold = 0.01
    pos_controller = PID_Controller(0.1, 0.000, 0, -5, 5)
    vel_controller = PID_Controller(3, 0.000, 0, -5, 5)

    i = 0

    while not rospy.is_shutdown():
        if i <= 5:

            outter_err_x = X[i] - x
            outter_err_y = Y[i] - y

            if abs(outter_err_x) < move_er_threshold and abs(outter_err_y) < move_er_threshold:
                i = i+ 1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                pos_controller.error = 0
                pos_controller.error_sum = 0   
                vel_controller.error = 0
                vel_controller.error_sum = 0
            else:
                inner_input_x = pos_controller.get_output(outter_err_x)
                inner_err_x = inner_input_x- vx
                cmd_vel.linear.x = vel_controller.get_output(inner_err_x)
                
                inner_input_y = pos_controller.get_output(outter_err_y)
                inner_err_y = inner_input_y- vy
                cmd_vel.angular.z = vel_controller.get_output(inner_err_y)

        pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
