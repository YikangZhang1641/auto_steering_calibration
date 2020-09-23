#!/usr/bin/env python3
#coding:utf-8

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
import tkinter.ttk as ttk
import hashlib
import subprocess
import threading

# 创建画布需要的库
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# 创建工具栏需要的库
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

# 快捷键需要的模块
from matplotlib.backend_bases import key_press_handler

# 导入绘图需要的模块
from matplotlib.figure import Figure

from set_boundary import DataRecorder, BoundaryBuilder
from auto_calibration import Calibrator, is_number

lat_offset = 22.533
lon_offset = 113.938
meter_in_lon = 1.141255544679108e-5
meter_in_lat = 8.993216192195822e-6
RESOLUTION = 0.2
OFFSET = 5.0


LOG_LINE_NUM = 0

class MY_GUI():
    def __init__(self, init_window_name):
        self.init_window_name = init_window_name
        rospy.init_node("auto_steering_calibration")
        self.boundary_initialized = False
        self.map = None
        self.recorder = DataRecorder()
        self.boundary_builder = None
        self.calib = Calibrator(self.boundary_builder)
        self.mp = None
        self.calib_is_running = False
        self.t = None

        plt.style.use('seaborn-whitegrid')  # 使用样式
        plt.rcParams['font.sans-serif'] = ['SimHei']  # 显示中文
        plt.rcParams['axes.unicode_minus'] = False  # 显示负号
        self.fig_map = plt.Figure(figsize=(5, 4), dpi=100)
        self.canvas_map = FigureCanvasTkAgg(self.fig_map, master=self.init_window_name)  # A tk.DrawingArea.
        self.canvas_map.get_tk_widget().grid(row=0, column=0, rowspan=4, columnspan=4)
        self.axc_map1 = self.fig_map.add_subplot(121)
        self.axc_map2 = self.fig_map.add_subplot(122)

        self.fig_calib = plt.Figure(figsize=(5, 4), dpi=100)
        self.canvas_calib = FigureCanvasTkAgg(self.fig_calib, master=self.init_window_name)  # A tk.DrawingArea.
        self.canvas_calib.get_tk_widget().grid(row=0, column=6, rowspan=4, columnspan=4)
        self.axc_calib = self.fig_calib.add_subplot(111)

    #设置窗口
    def set_init_window(self):
        my_font = ("song ti", 14)
        self.init_window_name.title("自动转向标定 v1.0")           #窗口名

        #self.init_window_name.geometry('320x160+10+10')                         #290 160为窗口大小，+10 +10 定义窗口弹出时的默认展示位置
        self.init_window_name.geometry('1068x681+10+10')
        #self.init_window_name["bg"] = "pink"                                    #窗口背景色，其他背景色见：blog.csdn.net/chl0000/article/details/7657887
        #self.init_window_name.attributes("-alpha",0.9)                          #虚化，值越小虚化程度越高
        #标签

        self.init_data_label = tkinter.Label(self.init_window_name, text="前轮转角", font=my_font)
        self.init_data_label.grid(row=0, column=0)

        # self.result_data_label = tkinter.Label(self.init_window_name, text="输出结果")
        # self.result_data_label.grid(row=1, column=0)

        # self.log_label = tkinter.Label(self.init_window_name, text="日志")
        # self.log_label.grid(row=12, column=0)

        #文本框
        # self.angle_label = tkinter.Label(self.init_window_name, text="输入一组转角值, 以换行符分隔", font=my_font)
        self.angle_label = tkinter.Label(self.init_window_name, text="输入单个转角值", font=my_font)
        self.angle_label.grid(row=5, column=5, rowspan=1, columnspan=2)
        self.angles_text = tkinter.Text(self.init_window_name, width=30, height=8)  #原始数据录入框
        self.angles_text.grid(row=6, column=5, rowspan=1, columnspan=2)

        self.velocity_label = tkinter.Label(self.init_window_name, text="输入单个速度值", font=my_font)
        self.velocity_label.grid(row=5, column=7, rowspan=1, columnspan=2)
        self.velocity_text= tkinter.Text(self.init_window_name, width=30, height=4)  #原始数据录入框
        self.velocity_text.grid(row=6, column=7, rowspan=1, columnspan=2)

        #
        # self.result_data_Text = tkinter.Text(self.init_window_name, width=70, height=49)  #处理结果展示
        # self.result_data_Text.grid(row=1, column=12, rowspan=15, columnspan=10)
        #
        # self.log_data_Text = tkinter.Text(self.init_window_name, width=66, height=9)  # 日志框
        # self.log_data_Text.grid(row=13, column=0, columnspan=10)

        self.auto_calib = tkinter.Button(self.init_window_name, text="自动标定_开始", font=my_font, bg="lightblue", width=10, command=self.mp_calib_start)
        self.auto_calib.grid(row=8, column=5, rowspan=1, columnspan=2)
        self.stop_calib = tkinter.Button(self.init_window_name, text="手动终止", font=my_font, bg="lightblue", width=10, command=self.calib_stop)
        self.stop_calib.grid(row=8, column=7, rowspan=1, columnspan=2)

        #按钮
        self.record_btn = tkinter.Button(self.init_window_name, text="边界绘制_开始", font=my_font, bg="lightblue", width=10, command=self.record_switch)
        self.record_btn.grid(row=5, column=0, rowspan=1, columnspan=2)
        self.log_label = tkinter.Label(self.init_window_name, text="边界: 未初始化")
        self.log_label.grid(row=6, column=0, rowspan=1, columnspan=2)
        # self.draw_boundary_btn = tkinter.Button(self.init_window_name, text="", font=my_font, bg="lightblue", width=10, command=self.stop_recorder)  # 调用内部方法  加()为直接调用
        # self.draw_boundary_btn.grid(row=3, column=11)

        self.plot_left()
        self.plot_right()

   #功能函数
    def record_switch(self):
        if self.recorder.is_finished:
            self.start_recorder()
        else:
            self.stop_recorder()

    def start_recorder(self):
        self.recorder.start()
        self.record_btn["text"] = "边界绘制_结束"
        self.log_label["text"] = "正在录制边界"
        self.axc_map1.clear()
        self.axc_map2.clear()
        self.boundary_initialized = False


    def stop_recorder(self):
        self.recorder.stop()
        self.record_btn["text"] = "边界绘制_重新开始"
        self.log_label["text"] = "录制已完成"
        self.boundary_initialized = True

    def mp_calib_start(self):
        self.t = threading.Thread(target=self.calib_start)
        self.t.setDaemon(True)
        self.t.start()

    def calib_start(self):
        mp_list = []
        if self.calib_is_running:
            return
        self.calib_is_running = True

        self.auto_calib = tkinter.Button(self.init_window_name, text="自动标定_开始", font=("song ti", 14), bg="lightblue", width=10, command=self.mp_calib_start)
        self.calib = Calibrator(self.boundary_builder)
        rospy.sleep(0.5)

        vel_str = self.velocity_text.get('0.0', tkinter.END)
        if not is_number(vel_str):
            print(vel_str, " is not an number!!!")
            self.calib.loop_stop()
            return

        Spd = float(vel_str)
        if Spd < 0:
            Spd = 0
        if Spd > 2.0:
            Spd = 2.0
        print("Spd: ", Spd)

        lines = self.angles_text.get('0.0', tkinter.END).splitlines()
        print(lines)

        index = 0
        while index < len(lines):
            line = lines[index]
            index += 1

            if not is_number(line):
                print(line, " is not an number!!!")
                self.calib.loop_stop()
                return

            angle = math.floor(float(line))
            if math.fabs((angle - float(line)) > 1e-3):
                print(float(line), " is not an integer input!!!")
                self.calib.loop_stop()
                return

            self.calib.loop_start(angle, Spd)
            while not rospy.is_shutdown() and not self.calib.is_finished and not self.calib.OUTSIDE_REGION:
                rospy.sleep(0.1)

            if self.calib.OUTSIDE_REGION:
                self.calib.loop_stop()

            rospy.sleep(0.5)
            if self.calib.is_finished:
                if self.calib.result is not None:
                    radius, err_avg = self.calib.result

        #     if self.calib.mp is not None:
        #         mp_list.append(self.calib.mp)
        # for mp in mp_list:
        #     mp.join()

    def calib_stop(self):
        self.calib.loop_stop()
        if self.t is not None:
            self.t.join()
            self.t = None
        self.calib_is_running = False
        self.calib.calculate(self.calib.angle, self.calib.raw_x, self.calib.raw_y)

    # def str_trans_to_md5(self):
    #     src = self.init_data_Text.get(1.0,tkinter.END).strip().replace("\n","").encode()
    #     #print("src =",src)
    #     if src:
    #         try:
    #             myMd5 = hashlib.md5()
    #             myMd5.update(src)
    #             myMd5_Digest = myMd5.hexdigest()
    #             #print(myMd5_Digest)
    #             #输出到界面
    #             self.result_data_Text.delete(1.0, tkinter.END)
    #             self.result_data_Text.insert(1.0, myMd5_Digest)
    #             self.write_log_to_Text("INFO:str_trans_to_md5 success")
    #         except:
    #             self.result_data_Text.delete(1.0, tkinter.END)
    #             self.result_data_Text.insert(1.0, "字符串转MD5失败")
    #     else:
    #         self.write_log_to_Text("ERROR:str_trans_to_md5 failed")
    #
    #
    # #获取当前时间
    # def get_current_time(self):
    #     current_time = time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))
    #     return current_time
    #
    #
    # #日志动态打印
    # def write_log_to_Text(self,logmsg):
    #     global LOG_LINE_NUM
    #     current_time = self.get_current_time()
    #     logmsg_in = str(current_time) +" " + str(logmsg) + "\n"      #换行
    #     if LOG_LINE_NUM <= 7:
    #         self.log_data_Text.insert(tkinter.END, logmsg_in)
    #         LOG_LINE_NUM = LOG_LINE_NUM + 1
    #     else:
    #         self.log_data_Text.delete(1.0,2.0)
    #         self.log_data_Text.insert(tkinter.END, logmsg_in)
    #

    def update_map(self):
        if len(self.recorder.raw_x) == 0:
            return

        if not self.boundary_initialized:
            self.boundary_builder = BoundaryBuilder(self.recorder.raw_x.copy(), self.recorder.raw_y.copy(), RESOLUTION, OFFSET)
            self.map = self.boundary_builder.map.T

        self.axc_map1.scatter(self.recorder.raw_x.copy(), self.recorder.raw_y.copy(), color='b')
        self.axc_map1.set_aspect('equal')
        self.axc_map2.imshow(self.map)
        self.axc_map2.invert_yaxis()

    def update_calib_plot(self):
        self.axc_calib.clear()
        if len(self.recorder.raw_x.copy()) > 0:
            self.axc_calib.scatter(self.recorder.raw_x.copy(), self.recorder.raw_y.copy(), color='gray')
        if len(self.calib.raw_x) > 0:
            self.axc_calib.scatter([self.calib.raw_x[-1]], [self.calib.raw_y[-1]], marker="*", s=200, color='r')

        # if self.map is not None:
        #     mark_map = self.map
        #     try:
        #         mark_x, mark_y = self.boundary_builder.grid_trans(self.recorder.raw_x[-1], self.recorder.raw_y[-1])
        #         mark_map[mark_x][mark_y] = 3
        #     except Exception:
        #         print("coord invalid")
        #     self.axc_calib.imshow(mark_map)

    def plot_right(self):
        if not rospy.is_shutdown():
            self.update_calib_plot()
            self.canvas_calib.show()
            self.init_window_name.after(300, self.plot_right)

    def plot_left(self):
        if not rospy.is_shutdown():
            self.update_map()
            self.axc_map1.set_title("Origin Trajectory")
            self.axc_map2.set_title("Drivable Region")

            self.canvas_map.show()
            self.init_window_name.after(300, self.plot_left)

if __name__ == "__main__":
    init_window = tkinter.Tk()              #实例化出一个父窗口
    ZMJ_PORTAL = MY_GUI(init_window)
    # 设置根窗口默认属性
    ZMJ_PORTAL.set_init_window()

    init_window.mainloop()