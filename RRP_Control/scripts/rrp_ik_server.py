#!/usr/bin/env python3

from cmath import pi
from random import betavariate
from rbe500_project.srv import rrpIK,rrpIKResponse
import rospy
from math import acos, atan2, sqrt
import numpy as np

def find_joint_angles(req): #req is an instance of the class rrpIK request class
    
    m = sqrt(req.x*req.x + req.y*req.y)
    l0 = 0.45
    l1 = 0.425
    l2 = 0.345
    l3 = 0.11

    alpha = atan2(req.y,req.x) # have to acess x and y from client
   
    ki = (-(l2**2 - l1**2 - m**2)/(2*l1*m))
    beta = acos(ki)

    kl = round(((m**2 - l1**2 - l2**2)/(2*l1*l2)),4)
    
    theta_one = alpha - beta

    if theta_one < 0:
        theta_one = theta_one + 2*pi

    theta_two =  acos(kl)
    d_three = 0.05+l0-l3-req.z #have to access from client

    joint1_degree = np.rad2deg(theta_one)
    print("joint one",joint1_degree)
    joint2_degree = np.rad2deg(theta_two)
    print("joint two",joint2_degree)
    print("d_three",d_three)

    #mess ="Hello World"
    return rrpIKResponse(theta_one,theta_two,d_three)

def serv():
    rospy.init_node("find_IK")
    s=rospy.Service('serv_1',rrpIK,find_joint_angles)
    rospy.loginfo("Ready to solve the Inverse Kinematics")
    rospy.spin()

if __name__ == "__main__":
    serv()
