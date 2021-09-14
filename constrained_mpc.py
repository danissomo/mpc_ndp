#!/usr/bin/python3 
import csv
import rtde_control
import rtde_receive
import rtde_io
import time
import math
from copy import deepcopy
from planner import trajectPlanner, rtde_kinematic
import moveJ_from_csv  as csv_ur
import numpy as np
import random
UR_IP = "127.0.0.1"


class Mpc:
    def __init__(self):
        self.a_refab_theta = [] 
        self.a_theta = []
        self.n = 0
        self.a_v = []
        self.a_kg = []
        self.dt = 0.008
        self.dv_max= 0.024
    
    def v_from_theta(self, theta_0, theta, v_0):
        v = []
        for i in range ( len(v_0) ):
            v_i = 2*(theta[i] - theta_0[i])/self.dt - v_0[i]
            v.append(v_i)
        return(v)
    
    def theta_from_v(self, theta_0, v_0, v):
        theta = []
        for i in range ( len(theta_0) ):
            theta_i = theta_0[i] + 0.5*(v_0[i] + v[i])*self.dt
            theta.append(theta_i)
        return(theta)

    def smooth_v(self, begin_smooth =1):
        v = []
        v_i = []
        for i in range(begin_smooth):        
            v.append(deepcopy(self.a_v[i]))
        #n = len(self.a_v)-1
        for i in range(begin_smooth, self.n-begin_smooth):
            v_i = []
            for j in range(len(self.a_v[i])):
                v_ij = 0.25*self.a_v[i-1][j] + 0.5*self.a_v[i][j] + 0.25*self.a_v[i+1][j]
                v_i.append(deepcopy(v_ij))
            v.append(deepcopy(v_i))
        for i in range(self.n-begin_smooth, self.n):
            v_i = []
            for j in range(len(self.a_v[i])):
                v_ij = self.a_v[self.n-i][j]
                v_i.append(deepcopy(v_ij))
            v.append(deepcopy(v_i)) 
        self.a_v =v  
        return self.a_v
    
    def check_constraints(self, v, v_max):
        #flag1 = True
        #key_dim = 0
        key_gain = 1
        for i in range(len(v)):
            if ( abs(v[i]) > v_max[i] ):
                #flag1 = False
                gain = v_max[i] / abs(v[i])
                if ( gain < key_gain ):
                    key_gain = deepcopy(gain)
        return(key_gain)

    
    def get_speedJ(self, a_theta):
        # initialization
        self.a_refab_theta = [] 
        self.a_v = []
        self.a_kg = []

        self.a_theta = a_theta
        self.n = len(a_theta)
        self.a_v.append( [ 0.0 ]*len(a_theta[0]) )
        self.a_refab_theta.append(a_theta[0])
        
        v_max_i = [self.dv_max]*len(a_theta[0])

        constraint_satisfied = False
        dir1 = abs(a_theta[1][0]-a_theta[0][0]) / (a_theta[1][0]-a_theta[0][0])

        # generate velocity list
        for i in range(1, self.n):        
            v_new = self.v_from_theta(a_theta[i-1], a_theta[i], self.a_v[i-1])
            self.a_v.append(deepcopy(v_new))

        self.smooth_v()

        # recalculate angle trajectory from velocities
        for i in range(1, self.n):
            theta_new = self.theta_from_v(self.a_theta[i-1], self.a_v[i-1], self.a_v[i])
            self.a_refab_theta.append(theta_new) 

        # check constraints
        for i in range(len(a_theta[0])):
            v_max_i.append(deepcopy(self.a_v[0][i] + self.dv_max))

        self.a_kg.append( 1 )
        self.a_kg.append( self.check_constraints(self.a_v[1], deepcopy(v_max_i)) )  
        for i in range(2, len(self.a_v)):
            v_max_i = []
            for j in range(len(self.a_v[0])):
                v_max_i.append(deepcopy( abs(self.a_v[i][j]) + self.dv_max))
            self.a_kg.append( self.check_constraints(self.a_v[1], deepcopy(v_max_i)) ) 

        return self.a_v


def print_results(kg, theta, theta_tar):
    n = len(theta_tar)
    #print results
    for i in range(n): 
        toprint = []
        for j in range(6):     
            toprint.append(theta[i][j]-theta_tar[i][j])  
        print(toprint)
        if ( kg[i]<1 ):
            print(kg[i])  


if __name__ == '__main__':
    
    start = time.time()
    rtde_c = rtde_control.RTDEControlInterface(UR_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
    rtde_c.moveJ([1.602055311203003, -2.8690412680255335, 2.6830248832702637, -2.869070831929342, -1.5840452353106897, -0.0009949843036096695], 1)
    #rnd = [-(random.random()*100%20)/100 -0.3, (random.random()*100%20)/100-0.2, (random.random()*100%50)/100+0.2, 0, -math.pi/2, 0]
    
    csv_ur.moveJ_from_scv_f(rtde_c, "ndp.out.txt")
    
     #hardcoded 1 point
    point1 = rtde_r.getActualTCPPose()
    point2 =[0.02779895802090566, -0.6003423098739182, 0.5180805481436014, -0.15059444809490485, -2.217569120591045, 2.069619874084297]

    print("start mpc? y/n")
    ans = input()
    if ans != "y":
        exit()
    tp = trajectPlanner(debug=False)
    cart_a = tp.a_PointToPoint(point1, point2)

    ik = rtde_kinematic(rtde_c, debug=False)
    theta_tar = ik.get_joint_pose(cart_a, point1)
    mpc = Mpc()
    v = mpc.get_speedJ(theta_tar)
        
    #print_results(kg, theta, theta_tar)

    for com in v:
        t_start = time.time()
        rtde_c.speedJ(com, 5, mpc.dt)
        if time.time() - t_start < mpc.dt: 
            time.sleep(mpc.dt - (time.time()-t_start))
    rtde_c.speedStop(10.0)
    
    print()
    print("start pose ", point1)
    print("target pose ", point2)
    print("actual pose ", rtde_r.getActualTCPPose())

    pose_error = []
    for i in range(len(point2)):
        pose_error.append(point2[i]-rtde_r.getActualTCPPose()[i])
    print("pose error ", pose_error)
  
