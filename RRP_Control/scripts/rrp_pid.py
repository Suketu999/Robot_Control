#!/usr/bin/env python3

from cgi import print_form
from cmath import cos, sin
from logging import shutdown
from turtle import pd
import pandas as pd
import matplotlib.pyplot as plt
import rospy
import csv
from rbe500_project.srv import *
import numpy as np
from numpy import real
import rospy
import tf
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import JointState
from time import sleep
import time

#Given Target Points
p1 = [0, 0.77, 0.34]
p2 = [-0.345, 0.425, 0.24]
p3 = [-0.67, -0.245, 0.14]
p4 = [0.77, 0.0, 0.39]
points = list([p1,p2,p3,p4])

def client_one(ii):
    rospy.wait_for_service('serv_1')
    serv_1 = rospy.ServiceProxy('serv_1',rrpIK)
    resp1 = serv_1(points[ii][0], points[ii][1], points[ii][2])
    return [resp1.theta_one, resp1.theta_two, resp1.d_three]
    print(strg)
    p1_joint_variables = get_joint_variables(p1[x], p1[y], p1[z])

jnt_angles = list([client_one(0),client_one(1),client_one(2),client_one(3)])
print(jnt_angles)
global x 

class RRP():
    def __init__(self):
        rospy.init_node("rrp_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        
        self.jnt1_effrt = Float64()
        self.jnt1_effrt.data = 0
        self.jnt1_effrt_pub = rospy.Publisher("/rrp/joint1_effort_controller/command", Float64,queue_size=10)

        self.jnt2_effrt = Float64()
        self.jnt2_effrt.data = 0
        self.jnt2_effrt_pub = rospy.Publisher("/rrp/joint2_effort_controller/command", Float64,queue_size=10)

        self.jnt3_effrt = Float64()
        self.jnt3_effrt.data = 0
        self.jnt3_effrt_pub = rospy.Publisher("/rrp/joint3_effort_controller/command", Float64,queue_size=10)

        self.i = 0
        self.j = 0
        self.k = 0

        self.rate = rospy.Rate(100)

##...............................................................

        self.poses_j_1 = 0
        self.poses_j_2 = 0
        self.poses_j_3 = 0
        self.vel_j_1 = 0
        self.vel_j_2 = 0
        self.vel_j_3 = 0
        self.js_sub = rospy.Subscriber("/rrp/joint_states", JointState, self.js_callback) 
        #how to call this continuously?
        
        self.logging_counter = 0
        self.trajectory = np.array([[0,0,0]])
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            print('saving file...')
            np.savetxt('trajectory.csv', np.array(self.trajectory,dtype='float64'), fmt='%f', delimiter=',')
		    
        #self.run()
		
    def run(self): #for all the joints

        start_time = time.time()
        jj = 0
        
        while jj <= 3:

            set_point_one = jnt_angles[jj][0] #set point for joint 1
            set_point_two = jnt_angles[jj][1] #set point for joint 2
            set_point_three = jnt_angles[jj][2] #set point for joint 3
            
            error1 = set_point_one - self.poses_j_1
            E1 = error1
            # error_prev1 = error1
            # error_dot1 = error1 - error_prev1
            Kp_1 = 15
            Kd_1 = 70
            Ki_1 = 0
            # Ki_1 = 0.00008
            #5 0.06 # 0.000005

            error2 = set_point_two - self.poses_j_2
            E2 = error2
            # error_prev2 = error2
            # error_dot2 = error2 - error_prev2
            Kp_2 = 5
            Kd_2 = 5
            Ki_2 = 0.00001
            #5 #0.01 #0.0000001

            error3 = set_point_three - self.poses_j_3
            E3 = error3
            # error_prev3 = error3
            # error_dot3 = error3 - error_prev3
            Kp_3 = 8
            Kd_3 = 0.9
            Ki_3 = 0.005

            # initializing effort publication
            self.jnt1_effrt_pub.publish(0)
            self.jnt2_effrt_pub.publish(0)
            self.jnt3_effrt_pub.publish(0)        
            
            while rospy is not shutdown():

                while abs(error1) > 0.1 or abs(error2) > 0.1 or abs(error3) > 0.002:
                    
                    # error_prev1 = error1
                    E1 = E1 + error1
                    error1 = set_point_one - self.poses_j_1
                    # print(error1,error_prev1)
                    # error_dot1 = error1-error_prev1
                    # print(error_dot1)
                    # u1 = Kp_1*error1 + Kd_1*100*error_dot1 + Ki_1*E1
                    # print(u1)
                    # print(error1)
                    # print(self.vel_j_1)
                    u1 = Kp_1*error1 - Kd_1*self.vel_j_1 + Ki_1*E1
                    # print(u1 - Kp_1*error1 - Ki_1*E1)


                    # error_prev2 = error2
                    E2 = E2 + error2
                    error2 = set_point_two - self.poses_j_2
                    # error_dot2 = error2 - error_prev2
                    # u2 = Kp_2*error2 + Kd_2*100*error_dot2 + Ki_2*E2
                    u2 = Kp_2*error2 - Kd_2*self.vel_j_2 + Ki_2*E2

                    # error_prev3 = error3
                    E3 = E3 + error3
                    error3 = set_point_three - self.poses_j_3
                    # error_dot3 = error3 - error_prev3
                    # u3 = Kp_3*error3 + Kd_3*100*error_dot3 + Ki_3*E3
                    u3 = Kp_3*error3 - Kd_3*self.vel_j_3 + Ki_3*E3

                    # if u3 > 0 and u3 < 2.001:
                    #     u3 = 2.001
                    # if u3 < 0 and abs(u3) < 2.001:
                    #     u3 = -2.001
                    # print(u3) 
                    
                    self.jnt1_effrt_pub.publish(u1)
                    self.jnt2_effrt_pub.publish(u2)
                    self.jnt3_effrt_pub.publish(u3)

                    # error1 = set_point_one - self.poses_j_1
                    # error2 = set_point_two - self.poses_j_2
                    # error3 = set_point_three - self.poses_j_3

                    if abs(error1) < 0.1:
                        self.jnt1_effrt_pub.publish(0)
                    if abs(error2) < 0.1:
                        self.jnt2_effrt_pub.publish(0)
                    if abs(error3) < 0.002:
                        self.jnt3_effrt_pub.publish(0)
                    while abs(error1) < 0.1 and abs(error2) < 0.1 and abs(error3) < 0.002:
                        # print('Errors below Specified values!!!')
                        kkk = 0
                        while kkk<100:
                            kkk = kkk+1
                        
                        error1 = set_point_one - self.poses_j_1
                        error2 = set_point_two - self.poses_j_2
                        error3 = set_point_three - self.poses_j_3
                        if abs(error1) < 0.1 and abs(error2) < 0.1 and abs(error3) < 0.002:
                            break

                self.jnt1_effrt_pub.publish(0)
                self.jnt2_effrt_pub.publish(0)
                self.jnt3_effrt_pub.publish(0)
                break

            sleep(1)

            error1 = 1
            error2 = 1
            error3 = 1
            
            print ('Reached point:',jj+1)
            jj = jj+1
            if jj>=4:
                end_time = time.time()
                print(end_time - start_time)
                #print(self.trajectory)
                _, ax = plt.subplots(1)
                ax.set_aspect('equal')
                # self.trajectory = np.loadtxt("trajectory.csv", delimiter=',')
                plt.plot(np.arange(1,len(self.trajectory)+1),self.trajectory[:, 0], label="Joint 1", linewidth=2)
                plt.plot(np.arange(1,len(self.trajectory)+1),self.trajectory[:, 1], label="Joint 2", linewidth=2) 
                plt.plot(np.arange(1,len(self.trajectory)+1),self.trajectory[:, 2], label="Joint 3", linewidth=2)
                plt.legend(loc="upper right")
                plt.title("Evolution of Joint Variables")
                plt.xlabel("Length of list (number)")
                plt.ylabel("Time taken (seconds)")
                plt.xlim(0, 40)
                plt.ylim(0, 5)
                plt.minorticks_on()
                plt.grid()
                plt.xlabel('Iteration Number(*100)')
                plt.ylabel('Joint Value')

                # remove the comment to see the plotting
                #plt.show()
                
                break
            # print ('Moving to point:',jj+1)
    
    '''''
    def visvualization(self):
            _, ax = plt.subplots(1)
            ax.set_aspect('equal')
            # self.trajectory = np.loadtxt("trajectory.csv", delimiter=',')
            plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], self.trajectory[:, 2], linewidth=2)
            plt.xlim(-1, 7)
            plt.ylim(-1, 7)
            #plt.zlim(-1, 7)
            plt.minorticks_on()
            plt.grid()
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            #plt.zlabel('z (m)')
            
            plt.show()
    m = visvualization() 
    '''
#################################################################################################

    def js_callback(self,msg):
        
        l0 = 0.45
        l1 = 0.425
        l2 = 0.345
        l3 = 0.11

        self.poses_j_1 = msg.position[0]
        #print(self.poses_j_1)
        self.vel_j_1 = msg.velocity[0]
        self.poses_j_2 = msg.position[1]
        self.vel_j_2 = msg.velocity[1]
        self.poses_j_3 = msg.position[2]
        self.vel_j_3 = msg.velocity[2]

        self.logging_counter += 1
        df = pd.DataFrame(columns = ['end_x','end_y','end_z'])
        with open('trajectory.csv', 'w', newline='') as f:
                
            if self.logging_counter == 100:
                self.logging_counter = 0

                # save trajectory
                # end_x = l1*cos(self.poses_j_1) + l2*cos(self.poses_j_1 + self.poses_j_2)
                # end_y = l1*sin(self.poses_j_1) + l2*sin(self.poses_j_1 + self.poses_j_2)
                # end_z = real(0.05 + l0 - l3 - self.poses_j_3)
                
                self.trajectory = np.append(self.trajectory,[[self.poses_j_1, self.poses_j_2, self.poses_j_3]],axis=0)
                
                #x = np.append(self.trajectory,[[self.poses_j_1, self.poses_j_2, self.poses_j_3]],axis=0)
                # print(len(self.trajectory))
                # self.trajectory.append([end_x, end_y, end_z])
                #writer = csv.writer(f)
                #writer.writerow([end_x, end_y, end_z]
                # for i in range(50):
                #     df = df.append({'end_x':self.trajectory[i][0],
                #                     'end_y':self.trajectory[i][1],
                #                     'end_z':self.trajectory[i][2] },ignore_index=True)
                #     df.to_csv("prar.csv",index = False)
    
    '''''
    def visualization(self):
    # load csv file and plot trajectory
        _, ax = plt.subplots(1)
        ax.set_aspect('equal')
        # self.trajectory = np.loadtxt("trajectory.csv", delimiter=',')
        plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], self.trajectory[:, 2], linewidth=2)
        plt.xlim(-1, 7)
        plt.ylim(-1, 7)
        #plt.zlim(-1, 7)
        plt.minorticks_on()
        plt.grid()
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        #plt.zlabel('z (m)')
        plt.show()
    '''

if __name__ == '__main__':
   
    whatever = RRP()
    #whatever.visualization()
    
	