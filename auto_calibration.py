#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys, getopt
import time
import rospy
from udrive_msgs.msg import ControlCmd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
from sensor_msgs.msg import NavSatFix
import os
from set_boundary import lat_lon_to_x_y

Spd = 0.4
OutputFile = "data/calibration.txt"

class Calibrator:
    def __init__(self, bd):
        rospy.Subscriber("/novatel718d/pos", NavSatFix, self.data_callback)
        self.steer_pub = rospy.Publisher("auto/cmd_vel", ControlCmd, queue_size=1)

        self.raw_x, self.raw_y = [], []
        self.angle, self.spd = 0, 0
        self.flag_record = False
        self.mp = None
        self.is_finished = False

        self.boundarybuilder = bd
        self.OUTSIDE_REGION = False
        self.result = None
        # plt.ion()
        # plt.show()
        if not os.path.exists("data"):
            os.mkdir("data")

    def __del__(self):
        print("jobs done")

    def data_callback(self, msg):
        x, y = lat_lon_to_x_y(msg.longitude, msg.latitude)
        if self.boundarybuilder is not None and self.boundarybuilder.map is not None:
            grid_x, grid_y = self.boundarybuilder.grid_trans(x, y)
            if self.boundarybuilder.not_valid(grid_x, grid_y) or self.boundarybuilder.map[grid_x][grid_y] == self.boundarybuilder.OUTSIDE:
                print("outside region")
                self.OUTSIDE_REGION = True
                self.loop_stop()

        if not (msg.status.status >= 1 and msg.position_covariance[0] < 10 and msg.position_covariance[3] < 10):
            print("GPS status error or covariance too large! Data not used! ")
            return

        if self.flag_record is False:
            return

        self.raw_x.append(x)
        self.raw_y.append(y)
        # print("raw data length: ", len(self.raw_x))
        # print("x=", x, "y=", y)
        if self.at_start_point():
            x_copy, y_copy = self.raw_x.copy(), self.raw_y.copy()
            self.loop_stop()
            self.calculate(self.angle, x_copy, y_copy)
            # self.mp = multiprocessing.Process(target=self.calculate, args=(self.angle, x_copy, y_copy))
            # self.mp.start()

        if len(self.raw_x) > 300:
            self.loop_stop()
            print("Time out! Over 300 sec!")

    def at_start_point(self):
        if len(self.raw_x) > 30 and self.distance(self.raw_x[0], self.raw_y[0], self.raw_x[-1], self.raw_y[-1]) < 1.0:
            return True
        return False

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
        self.OUTSIDE_REGION = False
        self.result = None
        self.raw_x.clear()
        self.raw_y.clear()

    def loop_stop(self):
        self.flag_record = False
        self.is_finished = True
        self.publish_msg(0, 0)
        # if self.mp is not None:
        #     self.mp.join()
        #     self.mp = None

    def distance(self, ax, ay, bx, by):
        return np.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

    def residuals(self, p, x, y):
        offset_x, offset_y, R = p
        return (1.0 - np.sqrt((x - offset_x)**2 + (y - offset_y)**2) / R) ** 2

    def draw_circle(self, p0, clr):
        N = 100
        ox, oy, r = p0
        x, y = [], []
        for i in range(N + 1):
            angle = np.pi * 2 * i / N
            x.append(r * np.cos(angle) + ox)
            y.append(r * np.sin(angle) + oy)
        plt.plot(y, x, color=clr, linestyle='-')

    def calculate(self, angle, raw_x, raw_y):
        global OutputFile
        avg_x = np.average(raw_x)
        avg_y = np.average(raw_y)
        r_filter = [self.distance(avg_x, avg_y, raw_x[i], raw_y[i]) for i in range(len(raw_x))]
        avg_r = np.average(r_filter)
        # avg_r = np.max(raw_x) - np.min(raw_x)

        p0 = [avg_x, avg_y, avg_r]
        plsq = leastsq(self.residuals, p0, args=(raw_x, raw_y))
        print(plsq[0][0], plsq[0][1], plsq[0][2])

        err_sum = 0
        for i in range(len(raw_x)):
            err_sum += self.residuals(plsq[0], raw_x[i], raw_y[i])
        err_avg = err_sum / len(raw_x)

        self.result = [plsq[0][2], err_avg]
        print("Success! Radius = ", plsq[0][2])

        fileObject = open(OutputFile, "a+")
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        fileObject.write(time_str)
        fileObject.write(" ")
        fileObject.write(str(angle))
        fileObject.write(" ")
        fileObject.write(str(plsq[0][2]))
        fileObject.write(" ")
        fileObject.write(str(err_avg))
        fileObject.write("\n")
        fileObject.close()

        # plt.ion()
        # plt.figure(time_str)
        # plt.show()
        # self.draw_circle(plsq[0], "red")

        N = 100
        ox, oy, r = plsq[0]
        x, y = [], []
        for i in range(N + 1):
            angle = np.pi * 2 * i / N
            x.append(r * np.cos(angle) + ox)
            y.append(r * np.sin(angle) + oy)
        plt.plot(y, x, color="red", linestyle='-')

        plt.title("angle: " + str(self.angle) + ", \nRadius: " + str(round(plsq[0][2], 2)))
        plt.scatter(raw_y, raw_x)
        plt.grid(True)
        plt.axes().set_aspect('equal')
        plt.savefig("./data/" + time_str + ".png")

        # plt.draw()
        # plt.show(block=False)
        # plt.pause(0.1)
        # plt.close(time_str)
        plt.clf()

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
    rospy.init_node("steering_calibrator")

    calib = Calibrator(None)
    rospy.sleep(0.5)
    for steer in input_array:
        if is_number(steer):
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
            # if calib.mp is not None:
            #     calib.mp.join()


if __name__ == "__main__":
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
