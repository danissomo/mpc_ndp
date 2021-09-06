#!/usr/bin/python3 
from constrained_mpc import print_results
import rtde_control
import rtde_receive
import csv
import argparse
UR_IP = "127.0.0.1"

def moveJ_from_scv(rtde_c, filename):
    f_csv = open(filename, 'r')
    theta_tar_csv = csv.reader(f_csv, delimiter=',', quotechar='\n')
    theta_tar =[]
    for theta in theta_tar_csv:
        theta_tar.append([])
        for elem in theta:
            theta_tar[-1].append(float(elem))

    for theta in theta_tar:
        rtde_c.moveJ(theta)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("N", type=str)
    parser.add_argument("--ip", type=str, default=UR_IP)
    args = parser.parse_args()
    moveJ_from_scv(rtde_control.RTDEControlInterface(args.ip), args.N)