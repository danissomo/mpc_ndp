#!/usr/bin/python3 
from io import SEEK_CUR
from sys import stdout
import sys
import rtde_control
import rtde_receive
from enum import Enum
import numpy as np
import time
import logging
import argparse
import planner

class UR5_robot:
    class Facing(Enum):
        plus_yz = None
        plus_xz = None
        plus_yx = None

        minus_yz = None
        minus_xz = None
        minus_yx = None
    
    def __init__(self, log_level=logging.INFO, log_file = True):
        self.facing = self.Facing
        self.rtde_c = None
        self.rtde_r = None
        self.wait_socket = 20.0
        self.fold_pos = [1.602055311203003, -2.8690412680255335, 2.6830248832702637, -2.869070831929342, -1.5840452353106897, -0.0009949843036096695]
        if log_file: 
            logging.basicConfig(filename="ur5.log", level=log_level, format= '[%(levelname)s]:%(message)s')
        else:
            logging.basicConfig(level=log_level, format= '[%(levelname)s]: %(message)s')
        logging.info("--Init robot--")

        
    def connect(self, UR_IP:str = "127.0.0.1"):
        
        t_start = time.time()
        logging.info("Connecting to robot on IP: %s", UR_IP)
        t_last_print = time.time()
        while time.time() - t_start < self.wait_socket :
            try:
                self.rtde_c = rtde_control.RTDEControlInterface(UR_IP)
                self.rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
                break
            except RuntimeError:
                logging.warn("Failed attempt")
                
        if time.time() - t_start > self.wait_socket:
            logging.error("Timeout")
            return
        if self.is_connected():
            logging.info("Connected successfully")
        else:
            logging.error("External control is not running")
            return
    

    def is_connected(self)->bool:
        if self.rtde_c ==None or self.rtde_r == None:
            logging.critical("rtde interface is None")
            return False
        isc = self.rtde_c.isConnected() and self.rtde_r.isConnected()
        if not isc:
            logging.warn("Robot is not connected")
        return isc

    
    def exec_a_speedJ(self, a_speed:np.ndarray, dt:float):
        if not self.is_connected():
            return

        t_update = time.time()
        for speed in a_speed:
            self.rtde_c.speedJ(speed, 5, dt)
            while time.time() - t_update < dt:
                time.sleep(dt- (time.time() - t_update))
            t_update = time.time()
        self.stop()


    def exec_a_moveJ(self, a_theta:np.ndarray, speed:float, acc:float, blend:float):
        if not self.is_connected():
            return
        
        a_theta = np.concatenate(a_theta, [[speed, acc, blend]]*len(a_theta))
        self.rtde_c.moveJ(a_theta)

    def exec_a_moveJ(self, a_theta:np.ndarray):
        self.rtde_c.moveJ(a_theta)

    def fold(self, speed:float, acc:float):
        if not self.is_connected():
            return
        self.rtde_c.moveJ(self.fold_pos, speed, acc)
        logging.debug("Folding robot: %s", self.fold_pos.__str__())
    
    
    def orient_tool(self, facing:Facing, speed:float = 0.1, acc:float=0.1):
        if not self.is_connected():
            return

        actual_cart = self.getCart()
        if actual_cart == None:
            return

        actual_cart[3:]+=facing
        
        self.moveTool(actual_cart, speed, acc)
        logging.debug("Moving eef %%", actual_cart)
        return actual_cart


    def getCart(self)->np.ndarray:
        if not self.is_connected():
            return None
        return np.array(self.rtde_r.getActualTCPPose())
    
    
    def moveJ(self, theta:np.ndarray, speed:float, acc:float):
        if not self.is_connected():
            return
        self.rtde_c.moveJ(theta, speed, acc)
    

    def moveTool(self, cords:np.ndarray,  speed:float, acc:float):
        if not self.is_connected():
            return
        self.rtde_c.moveL(cords, speed, acc)
        pass


    def stop(self):
        if not self.is_connected():
            return
        self.rtde_c.speedStop(5.0)
        

    def emergency_stop(self):
        self.rtde_c.triggerProtectiveStop()
        logging.warn("Emergency protective stop")


    def disconnect(self):
        if not self.is_connected():
            return
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        logging.info("Disconnected")


    def home(self, speed:float, acc:float):
        if not self.is_connected():
            return
        self.moveJ([0.0]*6, 0.1, 0.1)
        pass

    
    def precalc_traject(self, point1, point2):
        tp = planner.trajectPlanner(debug=False)
        cart_a = tp.a_PointToPoint(point1, point2)
        ik = planner.rtde_kinematic(self.rtde_c, debug=False)
        theta_a = ik.get_joint_pose(cart_a, point1)
        return theta_a
    

if __name__ == '__main__':
    ur5 = UR5_robot(log_file=False, log_level=logging.DEBUG)
    ur5.connect()
    ur5.fold(0.5, 0.5)