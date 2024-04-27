#!/usr/bin/env python

import rospy
import rospkg
from scipy import linalg as lnr
from matplotlib import pyplot as plt
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
import os
import sys

# import the Kalman filter we finished last week
# from KalmanFilter import KalmanFilter

class KalmanFilter(object):
    # initialization the kalman filter. 
    #   x'(t) = Ax(t) + Bu(t) + w(t)
    #   y(t) = Cx(t) + v(t)
    #   x(0) ~ N(x_0, P_0)
    def __init__(self, mass, C, Sigma_w, Sigma_v, x_0, P_0):
        self.mass = mass
        self.C = C
    
        self.n = 4
        self.m = 2

        self.Sigma_w = Sigma_w
        self.Sigma_v = Sigma_v
        
        self.t = 0
        self.x = x_0
        self.P = P_0
        self.u = np.zeros([self.m, 1])

    # Given duration dt, return the discretization of A, B, Sigma_w. Just like what we do last week.
    def _discretization_Func(self, dt):
        Atilde = np.array([
            [1, 0,  0,  0],
            [dt,1,  0,  0],
            [0, 0,  1,  0],
            [0, 0,  dt, 1]
        ])
        Btilde = np.array([
            [dt/self.mass,      0],
            [dt*dt/2/self.mass, 0],
            [0,      dt/self.mass],
            [0, dt*dt/2/self.mass]
        ])
        q1 = self.Sigma_w[0,0]
        q2 = self.Sigma_w[1,1]
        q3 = self.Sigma_w[2,2]
        q4 = self.Sigma_w[3,3]
        Sigma_w_tilde = np.array([
            [dt*q1,         dt*dt/2*q1,                 0,          0],
            [dt*dt/2*q1,    (dt*q2)+(dt*dt*dt/3*q1),    0,          0],
            [0,             0,                          dt*q3,      dt*dt/2*q3],
            [0,             0,                          dt*dt/2*q3, (dt*q4)+(dt*dt*dt/3*q3)],
        ])

        return Atilde, Btilde, Sigma_w_tilde

    ################################################################################################
    ## ================================ Edit below here ================================ vvvvvvvvvvv
    # predict step   
    def _predict_Step(self, ctrl_time):
        dt = ctrl_time - self.t
        self.t = ctrl_time

        A, B, Sigma_w_tilde = self._discretization_Func(dt)

        self.x = np.dot(A, self.x) + np.dot(B, self.u)
        self.P = np.dot(np.dot(A, self.P), A.T) + Sigma_w_tilde



    # correction step
    def _correction_Step(self, y):
        K = np.dot(np.dot(self.P, self.C.T), lnr.inv(np.dot(np.dot(self.C, self.P), self.C.T) + self.Sigma_v))
        self.x = self.x + np.dot(K, (y - np.dot(self.C, self.x)))
        self.P = np.dot((np.eye(self.n) - np.dot(K, self.C)), self.P)
    



    # when getting the control signal, execution the predict step, update the control signal      
    def control_moment(self, u_new, time_now):
        self._predict_Step(time_now)
        self.u = u_new


    # when getting the observe info, execution the predict step, and then execution the correction step

    def observe_moment(self, y_new, time_now):
        if self.t < time_now:
            self._predict_Step(time_now)
        self._correction_Step(y_new)
  

    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================
class Localization(object):
    def __init__(self):
        # config the subscribe information
        rospy.Subscriber('/robot/control', Twist, self.callback_control)
        rospy.Subscriber('/robot/observe', LaserScan, self.callback_observe)
        rospy.Subscriber('gazebo/set_model_state', ModelState, self.callback_state)
        self.pub = rospy.Publisher("/robot/esti_model_state", ModelState, queue_size=10)
        # catch Ctrl+C. When you press Ctrl+C, call self.visualzation()
        rospy.on_shutdown(self.visualization)
        self.last_x = 0
        self.last_y = 0
        self.real_x = 0
        self.real_y = 0

        # initialize Kalman filter. 
        self.kf = KalmanFilter(
            mass = 10, 
            C = np.array([
                [0, 1, 0, 0],
                [0, 0, 0, 1]
            ]),
            Sigma_w = np.eye(4)*0.00001,
            Sigma_v = np.array([[0.02**2, 0],[0, 0.02**2]]),
            x_0 = np.zeros([4,1]),
            P_0 = np.eye(4)/1000
            )

        # list to save data for visualization
        self.x_esti_save = []
        self.x_esti_time = []
        self.x_true_save = []
        self.x_true_time = []
        self.p_obsv_save = []
        self.p_obsv_time = []

    ################################################################################################
    ## ================================ Edit below here ================================ vvvvvvvvvvv
    def callback_control(self, twist):
        current_time = rospy.get_time()
        linear_velocity = twist.linear.x
        angular_velocity = twist.angular.z
        u_new = np.array([[linear_velocity], [angular_velocity]])
        self.kf.control_moment(u_new, current_time)
        self.x_esti_save.append(self.kf.x.flatten().tolist())
        self.x_esti_time.append(current_time)


    def callback_observe(self, laserscan):
        current_time = rospy.get_time()

        ranges = np.array(laserscan.ranges)
        wall_thickness = 0.2
        wall_length = 10.4
        ranges=np.array(laserscan.ranges)

        position_x = wall_length/2-wall_thickness-ranges[0]
        position_y = wall_length/2-wall_thickness-ranges[-1]
        y_new = np.array([[position_x], [position_y]])
        y_new = y_new.reshape(2,1)   
        ob = y_new

        threshold_lower = 0.0005
        threshold_upper = 0.1

        if (abs(position_x - self.last_x) < threshold_upper and abs(position_x - self.last_x) > threshold_lower): self.real_x = position_x-self.last_x+self.real_x
        if (abs(position_y - self.last_y) < threshold_upper and abs(position_y - self.last_y) > threshold_lower): self.real_y = position_y-self.last_y+self.real_y
        y_new = np.array([[self.real_x], [self.real_y]])
        y_new = y_new.reshape(2,1)

        self.kf.observe_moment(y_new, current_time)
        self.x_esti_save.append(self.kf.x.flatten().tolist())
        self.x_esti_time.append(current_time)
        self.p_obsv_save.append(ob.flatten().tolist())
        self.p_obsv_time.append(current_time)
     
        self.last_x = position_x
        self.last_y = position_y

        self.sendStateMsg()


    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================

    # restore the true state of robot for visualization. You CAN NOT get them in real world.
    def callback_state(self, state):
        current_time = rospy.get_time()
        x = np.zeros([4,1])
        x[0,0] = state.twist.linear.x
        x[1,0] = state.pose.position.x
        x[2,0] = state.twist.linear.y
        x[3,0] = state.pose.position.y
        self.x_true_save.append(x)
        self.x_true_time.append(current_time)

    def sendStateMsg(self):
        msg = ModelState()
        msg.model_name = 'cylinder_robot'
        msg.pose.position.x = self.kf.x[1]
        msg.pose.position.y = self.kf.x[3]
        msg.twist.linear.x = self.kf.x[0]
        msg.twist.linear.y = self.kf.x[2]
        self.pub.publish(msg)

    # visualzation
    def visualization(self):
        print("Visualizing......")
        
        if self.x_esti_save:
            x_esti = np.transpose(self.x_esti_save)
        else:
            x_esti = np.array([[]])  


        if self.p_obsv_save:
            p_obsv = np.transpose(self.p_obsv_save)
        else:
            p_obsv = np.array([[]])

        if self.x_true_save:
            x_true = np.column_stack([np.atleast_2d(x) for x in self.x_true_save])
        else:
            x_true = np.array([[]])

        if x_esti.shape[0] > 1 and x_true.shape[0] > 1:
            t_esti = np.array(self.x_esti_time)
            t_true = np.array(self.x_true_time)
            t_obsv = np.array(self.p_obsv_time)
            
            plt.figure(figsize=(16,9))
            plt.subplot(2,2,1)
            plt.plot(t_esti, x_esti[1,:], label="Estimation")
            plt.plot(t_true, x_true[1,:], label="True")
            plt.plot(t_obsv, p_obsv[0,:], label='obsv')
            plt.legend(loc='upper left')
            plt.title('px')

            plt.subplot(2,2,2)
            plt.plot(t_esti, x_esti[3,:], label="Estimation")
            plt.plot(t_true, x_true[3,:], label="True")
            plt.plot(t_obsv, p_obsv[1,:], label='obsv')
            plt.legend(loc='upper right')
            plt.title('py')

            plt.subplot(2,2,3)
            plt.plot(x_esti[1,:], x_esti[3,:], label="Estimation Trace")
            plt.plot(x_true[1,:], x_true[3,:], label="True Trace")
            plt.legend(loc='upper left')
            plt.title('Trace: Estimation vs True')

            plt.subplot(2,2,4)
            plt.plot(x_esti[1,:], x_esti[3,:], label="Estimation Trace")
            plt.plot(p_obsv[0,:], p_obsv[1,:], label='obsv')
            plt.legend(loc='upper left')
            plt.title('Trace: Estimation vs Observations')

            plt.tight_layout()
            fig_path = rospkg.RosPack().get_path('cylinder_robot') + "/"
            plt.savefig(fig_path + 'visualization.png', dpi=120)
            plt.show()
        else:
            print("Not enough data to visualize.")

        print("Visualization Complete.")




if __name__ == '__main__':
    try:
        rospy.init_node('perception', anonymous=True)
        obs = Localization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

