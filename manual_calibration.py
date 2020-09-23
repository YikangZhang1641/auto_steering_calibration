#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys, getopt
import time
# sys.path.append("../")
import rospy
from udrive_msgs.msg import ControlCmd
# from std_msgs.msg import Int32, Bool
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import multiprocessing
import os

lat_offset = 22.533
lon_offset = 113.938
meter_in_lon = 1.141255544679108e-5
meter_in_lat = 8.993216192195822e-6

Spd = 0.4
OutputFile = "calibration.txt"

def distance(ax, ay, bx, by):
    return np.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

def residuals(p, x, y):
    offset_x, offset_y, R = p
    return (1.0 - np.sqrt((x - offset_x)**2 + (y - offset_y)**2) / R) ** 2

def draw_circle(plt, p0, clr):
    N = 100
    ox, oy, r = p0
    x, y = [], []
    for i in range(N + 1):
        angle = np.pi * 2 * i / N
        x.append(r * np.cos(angle) + ox)
        y.append(r * np.sin(angle) + oy)
    plt.plot(x, y, color=clr, linestyle='-')

def calculate(angle, raw_x, raw_y):
    global OutputFile
    avg_x = np.average(raw_x)
    avg_y = np.average(raw_y)
    r_filter = [distance(avg_x, avg_y, raw_x[i], raw_y[i]) for i in range(len(raw_x))]
    avg_r = np.average(r_filter)
    # avg_r = np.max(raw_x) - np.min(raw_x)

    p0 = [avg_x, avg_y, avg_r]
    plsq = leastsq(residuals, p0, args=(raw_x, raw_y))
    print(plsq[0][0], plsq[0][1], plsq[0][2])

    err_sum = 0
    for i in range(len(raw_x)):
        err_sum += residuals(plsq[0], raw_x[i], raw_y[i])
    err_avg = err_sum / len(raw_x)

    print("Success! Radius = ", plsq[0][2])

    fileObject = open(OutputFile, "a")
    fileObject.write(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
    fileObject.write(" ")
    fileObject.write(str(angle))
    fileObject.write(" ")
    fileObject.write(str(plsq[0][2]))
    fileObject.write(" ")
    fileObject.write(str(err_avg))
    fileObject.write("\n")
    fileObject.close()

    draw_circle(plt, plsq[0], "red")
    plt.title("Radius: " + str(round(plsq[0][2], 2)))
    plt.scatter(raw_x, raw_y)
    plt.grid(True)
    plt.axes().set_aspect('equal')
    # plt.pause(0.001)
    # plt.draw()
    plt.show()

class Calibrator:
    def __init__(self):
        rospy.Subscriber("/novatel718d/pos", NavSatFix, self.data_callback)
        self.steer_pub = rospy.Publisher("auto/cmd_vel", ControlCmd, queue_size=1)

        self.raw_x, self.raw_y = [], []
        self.angle, self.spd = 0, 0
        self.flag_record = False
        self.mp = None
        self.is_finished = False

    def data_callback(self, msg):
        x = (msg.longitude - lon_offset) / meter_in_lon  # x设为经度
        y = (msg.latitude - lat_offset) / meter_in_lat  # y设为纬度

        if self.flag_record is False:
            return

        # if not (msg.status.status >= 1 and msg.position_covariance[0] < 1 and msg.position_covariance[3] < 1 and msg.position_covariance[8] < 1):
        #     print("GPS status error or covariance too large! Data not used! ")
        #     return

        self.raw_x.append(x)
        self.raw_y.append(y)
        # print("raw data length: ", len(self.raw_x))
        # print("x=", x, "y=", y)
        if len(self.raw_x) > 30 and distance(self.raw_x[0], self.raw_y[0], x, y) < 1:
            x_copy, y_copy = self.raw_x.copy(), self.raw_y.copy()
            self.loop_stop()
            self.mp = multiprocessing.Process(target=calculate, args=(self.angle, x_copy, y_copy))
            self.mp.start()

        if len(self.raw_x) > 300:
            self.loop_stop()
            print("Time out! Over 300 sec!")

    def publish_msg(self, angle, spd):
        msg = ControlCmd()
        msg.front_wheel_angle = angle
        msg.speed_mps = spd
        if msg.speed_mps == 0:
           msg.mode = 0
        elif msg.speed_mps > 0:
           msg.mode = 1
        else:
           msg.mode = 2
        self.steer_pub.publish(msg)
        print("command sent:", angle)

    def loop_start(self, angle, spd = 0.4):
        self.angle = angle
        self.spd = spd
        self.flag_record = True
        self.is_finished = False
        self.publish_msg(self.angle, self.spd)

    def loop_stop(self):
        self.flag_record = False
        self.is_finished = True
        self.publish_msg(0, 0)
        self.raw_x.clear()
        self.raw_y.clear()

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        pass

    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass

    return False


def main(input_array):
    global Spd
    calib = Calibrator()
    rospy.sleep(0.5)
    for steer in input_array:
        if not is_number(steer):
            continue

        angle = int(steer)
        if not isinstance(angle, int):
            print(angle, " is not an integer input!!!")
            continue
        if angle < -530:
            print(angle, "out of range: smaller than -530")
        if angle > 530:
            print(angle, "out of range: larger than 530")

        calib.loop_start(angle, Spd)
        while not calib.is_finished and not rospy.is_shutdown():
            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("steering_calibrator")
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hv:o:", ["speed=", "output="])
    except getopt.GetoptError:
        print('auto_calibration.py -v <speed> -o <outputfile> <angle1 angle2 angle3 ...>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('auto_calibration.py -v <speed> -o <outputfile> <angle1 angle2 angle3 ...>')
            sys.exit()
        elif opt in ("-v", "--speed"):
            if is_number(arg):
                Spd = float(arg)
        elif opt in ("-o", "--output"):
            OutputFile = arg

    main(args)
