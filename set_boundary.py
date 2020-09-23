#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys, getopt
import time
import rospy
from udrive_msgs.msg import ControlCmd
# from std_msgs.msg import Int32, Bool
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import multiprocessing
import math
from collections import deque
import tkinter
import hashlib


lat_offset = 22.533
lon_offset = 113.938
meter_in_lon = 1.141255544679108e-5
meter_in_lat = 8.993216192195822e-6
RESOLUTION = 0.2
OFFSET = 5.0


def lat_lon_to_x_y(lat, lon):
    return (lat - lon_offset) / meter_in_lon, (lon - lat_offset) / meter_in_lat  # y设为纬度


class DataRecorder:
    def __init__(self):
        rospy.Subscriber("/novatel718d/pos", NavSatFix, self.data_callback)

        self.raw_x, self.raw_y = [], []
        # self.flag_record = True
        self.is_finished = True

    def data_callback(self, msg):
        x, y = lat_lon_to_x_y(msg.longitude, msg.latitude)

        if self.is_finished is True:
            return

        self.raw_x.append(x)
        self.raw_y.append(y)

        # if len(self.raw_x) > 400:
        #     self.is_finished = True
        #     # self.flag_record = False

    def start(self):
        self.raw_x.clear()
        self.raw_y.clear()
        self.is_finished = False

    def stop(self):
        self.is_finished = True

class BoundaryBuilder:
    def __init__(self, raw_x, raw_y, resolution, offset):
        self.NOT_VISITED = 0
        self.TRAJECTORY = 1
        self.OUTSIDE = -1

        offset = max(offset, 0.1)

        self.resolution = resolution
        self.x_min = np.min(raw_x) - offset
        self.x_max = np.max(raw_x) + offset
        self.y_min = np.min(raw_y) - offset
        self.y_max = np.max(raw_y) + offset

        self.raw_x = raw_x
        self.raw_y = raw_y

        self.map = None
        self.generate_map()

    def generate_map(self):
        self.map = np.zeros(self.grid_trans(self.x_max, self.y_max))
        for i in range(len(self.raw_x)):
            x1 = self.raw_x[i - 1]
            y1 = self.raw_y[i - 1]
            x2 = self.raw_x[i]
            y2 = self.raw_y[i]

            for gx, gy in self.edge_points((x1, y1), (x2, y2)):
                self.map[gx][gy] = self.TRAJECTORY
        self.BFS((0, 0))

    def grid_trans(self, x, y):
        return int(math.floor((x - self.x_min) / self.resolution)), int(math.floor((y - self.y_min) / self.resolution))

    def edge_points(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2

        length = int(max(abs(x2-x1), abs(y2-y1)) / self.resolution) + 1
        dx = (x2 - x1) / length
        dy = (y2 - y1) / length

        x, y = x1, y1
        edge = []
        for i in range(length):
            edge.append(self.grid_trans(x, y))
            x += dx
            y += dy
        return edge

    def BFS(self, start_point):
        if self.map is None:
            print("Map not initialized!")
            return

        print(np.shape(self.map))
        dq = deque()
        dq.append(start_point)

        while len(dq) > 0:
            cur_x, cur_y = dq.popleft()
            if self.map[cur_x][cur_y] != self.NOT_VISITED:
                continue

            self.map[cur_x][cur_y] = self.OUTSIDE
            for next_p in self.surronding(cur_x, cur_y):
                dq.append(next_p)

    def not_valid(self, nx, ny):
        if nx < 0 or nx >= np.shape(self.map)[0] or ny < 0 or ny >= np.shape(self.map)[1]:
            return True
        return False

    def surronding(self, px, py):
        res = []
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx = px + dx
            ny = py + dy
            if self.not_valid(nx, ny):
                continue
            res.append((nx, ny))
        return res

    def plot_raw_traj(self, plt_):
        plt_.scatter(self.raw_x, self.raw_y)
        plt_.grid(True)
        plt_.axes().set_aspect('equal')
        plt_.show()

    def plot_map(self, plt_):
        plt_.imshow(self.map.T)
        plt_.gca().invert_yaxis()
        plt_.show()

if __name__ == "__main__":
    rospy.init_node("boundary_builder")
    record = DataRecorder()

    while record.is_finished is False:
        rospy.sleep(0.5)

    raw_x = record.raw_x.copy()
    raw_y = record.raw_y.copy()

    # boundary_builder = BoundaryBuilder(record.raw_x.copy(), record.raw_y.copy(), RESOLUTION, OFFSET)
    boundary_builder = BoundaryBuilder(raw_x, raw_y, RESOLUTION, OFFSET)

    mp1 = multiprocessing.Process(target=boundary_builder.plot_raw_traj, args=(plt))
    mp1.start()

    mp2 = multiprocessing.Process(target=boundary_builder.plot_map, args=(plt))
    mp2.start()


