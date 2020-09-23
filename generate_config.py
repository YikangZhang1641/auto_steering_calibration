import sys, getopt
import time
import numpy as np

InputFile = "data/calibration.txt"
OutputFile = "data/can_steer_table_config.config"
wheel_base = 2.26
width_rtk = 0.72
rear_to_rtk = 0.43

if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hi:o:", ["input=", "output="])
    except getopt.GetoptError:
        print('generate_config.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('generate_config.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--input"):
            InputFile = arg
        elif opt in ("-o", "--output"):
            OutputFile = arg

    data = {}
    inputObject = open(InputFile, "r")
    line = inputObject.readline()
    while line:
        words = line.split()
        angle = float(words[2])
        if angle not in data:
            data[angle] = []
        data[angle].append(float(words[3]))
        line = inputObject.readline()

    fileObject = open(OutputFile, "w")
    for key in sorted(data):
        steering_angle = float(key)
        Rg = np.average(data[key])
        if steering_angle < 0:
            radius = np.sqrt(Rg * Rg - rear_to_rtk * rear_to_rtk) - width_rtk / 2
            wheel_angle = -np.arctan2(wheel_base, radius) * 180 / np.pi
        else:
            radius = np.sqrt(Rg * Rg - rear_to_rtk * rear_to_rtk) + width_rtk / 2
            wheel_angle = np.arctan2(wheel_base, radius) * 180 / np.pi

        print("angle:", steering_angle, " radius:", radius, ", wheel:", wheel_angle)
        fileObject.write("steer_map {\n")
        fileObject.write("  front_wheel_angle: ")
        fileObject.write(str(wheel_angle))
        fileObject.write("\n  steering_wheel_angle: ")
        fileObject.write(str(key))
        fileObject.write("\n}\n\n")
    fileObject.close()