#!/usr/bin/python3 
import rtde_control
import rtde_receive
import rtde_io
import time
import math
from std_msgs.msg import String
from copy import deepcopy
from planner import trajectPlanner, rtde_kinematic
from ndp.utils.controller import make_controller
import numpy as np
import random
UR_IP = "127.0.0.1"
dt = 0.008
dv_max = 0.024

def v_from_theta(theta_0, theta, v_0):  
    global dt
    v = []
    for i in range ( len(v_0) ):
        v_i = 2*(theta[i] - theta_0[i])/dt - v_0[i]
        v.append(v_i)
    return(v)

def theta_from_v(theta_0, v_0, v):
    global dt
    theta = []
    for i in range ( len(theta_0) ):
        theta_i = theta_0[i] + 0.5*(v_0[i] + v[i])*dt
        theta.append(theta_i)
    return(theta)

def smooth_v(v_0, begin_smooth):
    v = []
    v_i = []
    for i in range(begin_smooth):        
        v.append(deepcopy(v_0[i]))
    #n = len(v_0)-1
    for i in range(begin_smooth, n-begin_smooth):
        v_i = []
        for j in range(6):
            v_ij = 0.25*v_0[i-1][j] + 0.5*v_0[i][j] + 0.25*v_0[i+1][j]
            v_i.append(deepcopy(v_ij))
        v.append(deepcopy(v_i))
    for i in range(n-begin_smooth, n):
        v_i = []
        for j in range(6):
            v_ij = v_0[n-i][j]
            v_i.append(deepcopy(v_ij))
        v.append(deepcopy(v_i))    
    return(v)

def check_constraints(v, v_max):
    #flag1 = True
    #key_dim = 0
    key_gain = 1
    for i in range(6):
        if ( abs(v[i]) > v_max[i] ):
            #flag1 = False
            gain = v_max[i] / abs(v[i])
            if ( gain < key_gain ):
                key_gain = deepcopy(gain)
    return(key_gain)




if __name__ == '__main__':
    
    start = time.time()
    rtde_c = rtde_control.RTDEControlInterface(UR_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
    rtde_c.moveJ([3, -0.7235987755982991, -2.617993877991494, 0, 3.14/2, 0])
    rnd = [-(random.random()*100%20)/100 -0.3, (random.random()*100%20)/100-0.2, (random.random()*100%50)/100+0.2, 0, -math.pi/2, 0]
    print(rnd)
    #rtde_c.moveL([-0.3, 0.2, 0.4, 0, -math.pi/2, 0])
    #rtde_c.moveL(rnd)
    
     #hardcoded 1 point
    point1 = rtde_r.getActualTCPPose()
    point2 =  point1.copy()
    point2 =[-0.5, 0, 0.5]
    point2.append(0)
    point2.append(-math.pi/2)
    point2.append(0)


    #ndp part
    ndp_target = np.array([-0.5, 0, 0.5])
    ref_joints = np.array([
        rtde_c.getInverseKinematics(point2)[0],
        rtde_c.getInverseKinematics(point2)[1],
        rtde_c.getInverseKinematics(point2)[2],
        0,
        0,
        0
    ])
    controller = make_controller()
    for i in range(100000000000):
        obs = np.ndarray((24,))
        obs[0] = rtde_r.getActualQ()[0]
        obs[1] = rtde_r.getActualQ()[1]
        obs[2] = rtde_r.getActualQ()[2]
        obs[3] = rtde_r.getActualQd()[0]
        obs[4] = rtde_r.getActualQd()[1]
        obs[5] = rtde_r.getActualQd()[2]
        obs[6] = ndp_target[0]
        obs[7] = ndp_target[1]
        obs[8] = ndp_target[2]
        obs[9] = rtde_r.getActualTCPPose()[0]
        obs[10] = rtde_r.getActualTCPPose()[1]
        obs[11] = rtde_r.getActualTCPPose()[2]
        obs[12] = obs[6] - obs[9]
        obs[13] = obs[7] - obs[10]
        obs[14] = obs[8] - obs[11]
        obs[15] = ref_joints[0]
        obs[16] = ref_joints[1]
        obs[17] = ref_joints[2]
        obs[18] = obs[15] - obs[0]
        obs[19] = obs[16] - obs[1]
        obs[20] = obs[17] - obs[2]
        obs[21] = 0
        obs[22] = 0
        obs[23] = 0
        u = controller.get_control(obs)
        u = np.clip(u, -1, 1)
        ref_joints[:3] += u
        ref_joints = np.clip(ref_joints, -np.pi, np.pi)
        point  =rtde_r.getActualQ()
        point[:3] += u
        
        rtde_c.moveJ(
            point
        )
        #print(obs)
    rtde_c.speedStop()

    

   
    tp = trajectPlanner(debug=False)
    cart_a = tp.a_PointToPoint(point1, point2)

    ik = rtde_kinematic(rtde_c, debug=False)
    theta_tar = ik.get_joint_pose(cart_a, rtde_r.getActualQ())

    # initialization
    v = []
    #theta = deepcopy (theta_tar)
    j = 1
    constraint_satisfied = False
    dir1 = abs(theta_tar[1][0]-theta_tar[0][0]) / (theta_tar[1][0]-theta_tar[0][0])
    v_max_i = []
    for i in range(6):
        v_max_i.append(dv_max)
    
    # test functions
    v.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] )
    theta = []
    theta.append(theta_tar[0])
    n = len(theta_tar)

    # generate velocity list
    for i in range(1, n):        
        v_new = v_from_theta(theta_tar[i-1], theta_tar[i], v[i-1])
        v.append(deepcopy(v_new))
        #print(v[i])    
    v = smooth_v(v,1)

    # recalculate angle trajectory from velocities
    for i in range(1, n):
        theta_new = theta_from_v(theta[i-1], v[i-1], v[i])
        theta.append(theta_new) 

    # check constraints
    for i in range(6):
        v_max_i.append(deepcopy(v[0][i] + dv_max))
    kg = []
    kg.append( 1 )
    kg.append( check_constraints(v[1], deepcopy(v_max_i)) )  
    for i in range(2, len(v)):
        v_max_i = []
        for j in range(6):
            v_max_i.append(deepcopy( abs(v[i][j]) + dv_max))
        kg.append( check_constraints(v[1], deepcopy(v_max_i)) ) 

    stop = time.time()

    # print results
    # for i in range(n): 
    #     toprint = []
    #     for j in range(6):     
    #         toprint.append(theta[i][j]-theta_tar[i][j])  
    #     print(toprint)
    #     if ( kg[i]<1 ):
    #         print(kg[i])  

    # print(dir1)     
    # print(stop-start)
  
    for com in v:
        t_start = time.time()

        rtde_c.speedJ(com, 5, dt)
        if time.time() - t_start < dt: 
            time.sleep(dt - (time.time()-t_start))
    rtde_c.speedStop()
    
    print()
    print("start pose ", point1)
    print("target pose ", point2)
    print("actual pose ", rtde_r.getActualTCPPose())

    pose_error = []
    for i in range(len(point2)):
        pose_error.append(point2[i]-rtde_r.getActualTCPPose()[i])
    print("pose error ", pose_error)
  
