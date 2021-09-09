#!/usr/bin/python3 
from constrained_mpc import print_results
import rtde_control
import rtde_receive
import csv
import argparse
import numpy as np
UR_IP = "127.0.0.1"

def moveJ_from_scv_f(rtde_c, filename):
    f_csv = open(filename, 'r')
    theta_tar_csv = csv.reader(f_csv, delimiter=',', quotechar='\n')
    theta_tar =[]
    for theta in theta_tar_csv:
        theta_tar.append([])
        for i in range(len( theta)):
            theta_tar[-1].append(float(theta[i]))
        #костыль для интерпретации ндп в реальность
        theta_tar[-1] = -np.array(theta_tar[-1])
        theta_tar[-1] +=  np.array([1.602055311203003, -2.8690412680255335, 0, -2.869070831929342, 0, 0])
    print("start ndp? y/n")
    ans = input()
    if ans !=  "y":
        exit()
    print("step by step? y/n")
    ans = input()
    step = False
    if ans == "y":
        step = True
    for theta in theta_tar:
        if step:
            input()
        rtde_c.moveJ(theta, 0.1)
        


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("N", type=str)
    parser.add_argument("--ip", type=str, default=UR_IP)
    args = parser.parse_args()
    moveJ_from_scv_f(rtde_control.RTDEControlInterface(args.ip), args.N)