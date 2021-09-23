#!/usr/bin/python3 
from logging import debug
import logging
import rtde_control
import rtde_receive
import csv
import argparse
import numpy as np
from robot import UR5_robot

UR_IP = "127.0.0.1"

def moveJ_from_scv_f(filename, offset = [1.602055311203003, -2.8690412680255335, 0, -2.869070831929342, 0, 0], dv= 0.5, v = 0.5, mix=0.01):
    f_csv = open(filename, 'r')
    theta_tar_csv = csv.reader(f_csv, delimiter=',', quotechar='\n')
    theta_tar =[]
    for theta in theta_tar_csv:
        theta_tar.append([])
        for i in range(len( theta)):
            theta_tar[-1].append(float(theta[i]))
        #костыль для интерпретации ндп в реальность
        theta_tar[-1] = -np.array(theta_tar[-1])
        theta_tar[-1] +=  np.array(offset)

        theta_tar[-1] = np.concatenate([theta_tar[-1], [dv, v, mix]])
    return theta_tar

def move(rtde_c, theta_tar):
    rtde_c.moveJ(theta_tar)
        


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("N", type=str)
    parser.add_argument("--ip", type=str, default=UR_IP)
    args = parser.parse_args()
    com  = moveJ_from_scv_f( args.N)
    ur5 = UR5_robot(log_level=logging.DEBUG, log_file=False)
    ur5.connect(args.ip)
    ur5.exec_a_moveJ(com)