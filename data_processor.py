from __future__ import division
import os
import pandas as pd
import math
import numpy as np
import scipy
import scipy.integrate
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
location = "F:\\Documents\\XV Gimnazija - tasks\\Personal Project\\test data\\"
accel_offset = [-0.163881298971316, 0.36584333045042167, -0.6074492173180897]
gyro_offset = [-3.5269592065422124, 0.3879769369541203, -0.2694927242527558]

def butter_lowpass(cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_highpass(cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_highpass_filter(data, cutoff, fs, order):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def gforce_to_ms2(value):
    return value*9.81

def integrate_points(x, y):
    output = scipy.integrate.cumtrapz(y, x, initial=0)
    """output = []
    for i in range(len(y)):
        if i != 0:
            integrated = scipy.integrate.simps(y[:i], x[:i])
        else:
            integrated = y[i]*x[i]
        output.append(integrated)"""
    return output

def time_interval_finder(times):
    differences = []
    for position in range(1, len(times)):
        difference = times[position] - times[position - 1] 
        differences.append(difference)
    avg_interval = np.mean(differences)
    return avg_interval

def convert_to_global_accel(local_accel, angles):
    ax_global_all = []
    ay_global_all = []
    az_global_all = []
    
    for i in range(len(local_accel[0])):
        ax, ay, az = local_accel[0][i], local_accel[1][i], local_accel[2][i]
        gx, gy, gz = angles[0][i], angles[1][i], angles[2][i]
        rotation_matrix = make_rotation_matrix([gx, gy, gz])
        ax_global, ay_global, az_global = np.dot(rotation_matrix, [ax, ay, az])
        ax_global_all.append(ax_global)
        ay_global_all.append(ay_global)
        az_global_all.append(az_global)
    return [ax_global_all, ay_global_all, az_global_all]

def make_rotation_matrix(angle):
    angle[0] = math.radians(angle[0])
    angle[1] = math.radians(angle[1])
    angle[2] = math.radians(angle[2])
    rot_x = np.array([[1, 0, 0],
                    [0, math.cos(angle[0]), -math.sin(angle[0])],
                    [0, math.sin(angle[0]), math.cos(angle[0])]])
    
    rot_y = np.array([[math.cos(angle[1]), 0, math.sin(angle[1])],
                    [0, 1, 0],
                    [-math.sin(angle[1]), 0, math.cos(angle[1])]])             

    rot_z = np.array([[math.cos(angle[2]), -math.sin(angle[2]), 0],
                    [math.sin(angle[2]),    math.cos(angle[2]), 0],
                    [0, 0, 1]])
    rotation_matrix = np.dot(rot_z, np.dot(rot_y, rot_x))
    rotation_matrix = transpose(rotation_matrix)
    return rotation_matrix

def transpose(input_matrix):
    transposed_matrix = np.transpose(input_matrix)
    return transposed_matrix

def compensate_for_gravity(accel_global_z):
    accel_global_z_no_grav = []
    for i in accel_global_z:
        accel_global_z_no_grav.append(i - 9.81)
    return accel_global_z_no_grav

def mean_of_points(data, interval):
    mean_data =  []
    for i in range(int(round(len(data)/interval))):
        section_mean = np.mean(data[i:i+interval])
        mean_data.append(section_mean)
    return mean_data

def median_of_points(data, interval):
    median_data =  []
    for i in range(int(round(len(data)/interval))):
        section_median = np.median(data[i:i+interval])
        median_data.append(section_median)
    return median_data

def sum_of_points(data, interval):
    summed_data =  []
    for i in range(int(round(len(data)/interval))):
        section_sum = np.sum(data[i:i+interval])
        summed_data.append(section_sum)
    return summed_data

def last_in_points(data, interval):
    last_data =  []
    for i in range(int(round(len(data)/interval))):
        section_last = data[i:i+interval][-1]
        last_data.append(section_last)
    return last_data

lowpass_cutoff = 1 #Hz cutoff frequency for lowpass
highpass_cutoff = 1 #Hz cutoff frequency for highpass
order = 5
interval = 3
starting_time = 30
save_figure = True
show_figure = False

#for every file in directory
for filename in os.listdir(location):
    #make absolute location
    full_name = location + "\\" + filename
    #if the file is not the configuration file
    if filename != "config.txt":
        #read data
        #t, ax, ay, az, gx, gy, gz, mx, my, mz, temp, press, alt
        #time between datapoints 30ms
        data1 = pd.read_csv(full_name, delimiter = "\t", header=4)
        data = pd.DataFrame.to_numpy(data1)
        data = np.delete(data, data.shape[0] - 1, 0)
        
        #read time values
        times_ms = data.T[0]
        times_seconds = []
        for time in times_ms:
            times_seconds.append(time/1000)
        times = times_seconds
        
        #find time interval
        time_interval = round(time_interval_finder(times), 2)
        frequency = 1.00 / time_interval
        print("The sampling frequency is " + str(frequency) + "Hz")
        #values for filters
        fs = frequency #sampling rate, Hz
        sampling_interval = time_interval
        
        
        data = data[int(starting_time * frequency):]
        #read time values
        times_ms = data.T[0]
        times_seconds = []
        for time in times_ms:
            times_seconds.append(time/1000)
        times = times_seconds
        
        #read press, temp, alt
        temp = data.T[10]
        press = data.T[11]
        alt = data.T[12]
        
        accel_cal_from_data = []
        #acceleration data processing
        accel = [data.T[1].copy().astype('float64'), data.T[2].copy().astype('float64'), data.T[3].copy().astype('float64')]
        #for each axis
        for direction in range (len(accel)):
            #for each datapoint in axis data
            for datapoint in range(len(accel[direction])):
                #convert to g-force, add offset
                accel[direction][datapoint] = gforce_to_ms2(accel[direction][datapoint] / 2048) - accel_offset[direction]
            #print calibration values
            accel_cal_from_data.append(np.mean(accel[direction]))
            
        gyro_cal_from_data = []
        #gyroscope data processing
        gyro = [data.T[4].copy().astype('float64'), data.T[5].copy().astype('float64'), data.T[6].copy().astype('float64')]
        for direction in range (len(gyro)):
            #for each datapoint in axis data
            for datapoint in range(len(gyro[direction])):
                #convert to deg/s, add offset
                gyro[direction][datapoint] = gyro[direction][datapoint] / (16.4) - gyro_offset[direction]
            #print calibration values
            gyro_cal_from_data.append(np.mean(gyro[direction]))
        
        #print calibration values if this was calibration data
        print("Accelerometer means X/Y/Z")
        accel_cal_from_data[2] = -(9.81 - accel_cal_from_data[2])
        print(accel_cal_from_data)
        print("Gyroscope means X/Y/Z")
        print(gyro_cal_from_data)
            
        """accel = [[1, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0 , 0], [0, 0, 0, 0, 0, 0, 0 , 0]]
        times = [1, 2, 3, 4, 5, 6, 7, 8]
        gyro = [[0, 0, 0, 0, 0, 0, 0 , 0], [0, 0, 0, 0, 0, 0, 0 , 0], [0, 0, 0, 0, 0, 0, 0 , 0]]"""
        
        #smooth values by taking medians - EXPERIMENTAL - 
        """accel = [median_of_points(accel[0], interval), median_of_points(accel[1], interval), median_of_points(accel[2], interval)]
        gyro =  [sum_of_points(gyro[0], interval), sum_of_points(gyro[1], interval), sum_of_points(gyro[2], interval)]
        interval_times = last_in_points(times, interval)
        times = interval_times"""
        
        #low pass/high pass filters
        #y1 = butter_lowpass_filter(y, lowpass_cutoff, fs, order) #lowpass
        #y2 = butter_highpass_filter(y, highpass_cutoff, fs, order) #highpass
        """accel_x = butter_lowpass_filter(accel[0], lowpass_cutoff, fs, order)
        accel_y = butter_lowpass_filter(accel[1], lowpass_cutoff, fs, order)
        accel_z = butter_lowpass_filter(accel[2], lowpass_cutoff, fs, order)
        accel = [accel_x, accel_y, accel_z]"""
        
            
        #integrate angular momentum for angles
        gyro_angles = [integrate_points(times, gyro[0]), integrate_points(times, gyro[1]), integrate_points(times, gyro[2])]
        
        #convert acceleration to global frame of refrence via gyroscopic data
        accel_global = convert_to_global_accel(accel, gyro_angles)
        
        #compensate for gravity
        accel_global = [accel_global[0], accel_global[1], compensate_for_gravity(accel_global[2])]
        accel = [accel[0], accel[1], compensate_for_gravity(accel[2])]
        
        #integrate velocity and distance
        velocity_local = [integrate_points(times, accel[0]), integrate_points(times, accel[1]), integrate_points(times, accel[2])]
        distances_local = [integrate_points(times, velocity_local[0]), integrate_points(times, velocity_local[1]), integrate_points(times, velocity_local[2])]
        velocity_global = [integrate_points(times, accel_global[0]), integrate_points(times, accel_global[1]), integrate_points(times, accel_global[2])]
        distances_global = [integrate_points(times, velocity_global[0]), integrate_points(times, velocity_global[1]), integrate_points(times, velocity_global[2])]


        #ACCELERATION GRAPH LOCAL
        plt.figure()
        plt.title("3-axis acceleration data depending on time in local frame of reference")
        plt.xlabel("Time/s")
        plt.ylabel("acceleration/ms2")
        plt.plot(times, accel[0], label = "accelerometer x")
        plt.plot(times, accel[1], label = "accelerometer y")
        plt.plot(times, accel[2], label = "accelerometer z")
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/accel_local.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #ACCELERATION GRAPH GLOBAL
        plt.figure()
        plt.title("3-axis acceleration data depending on time in global frame of reference")
        plt.xlabel("Time/s")
        plt.ylabel("acceleration/ms2")
        plt.plot(times, accel_global[0], label = "accelerometer x")
        plt.plot(times, accel_global[1], label = "accelerometer y")
        plt.plot(times, accel_global[2], label = "accelerometer z")
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/accel_global.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #ANGUALAR VELOCITY GRAPH
        plt.figure()
        plt.title("3-axis angular velocity depending on time")
        plt.xlabel("Time/s")
        plt.ylabel("Anglular velocity / deg/s")
        plt.plot(times, gyro[0], label = "gyroscope x")
        plt.plot(times, gyro[1], label = "gyroscope y")
        plt.plot(times, gyro[2], label = "gyroscope z")
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/angular_velocity.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #ROTATION GRAPH
        plt.figure()
        plt.title("3-axis rotation depending on time")
        plt.xlabel("Time/s")
        plt.ylabel("rotation/deg")
        plt.plot(times, gyro_angles[0], label = "gyroscope x")
        plt.plot(times, gyro_angles[1], label = "gyroscope y")
        plt.plot(times, gyro_angles[2], label = "gyroscope z")
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/rotation.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #VELOCITY GRAPH LOCAL
        plt.figure()
        plt.title("3-axis velocity from single-integration depending on time in local frame of reference")
        plt.xlabel("Time/s")
        plt.ylabel("velocity")
        plt.plot(times, velocity_local[0], label = "velocity x")
        plt.plot(times, velocity_local[1], label = "velocity y")
        plt.plot(times, velocity_local[2], label = "velocity z")
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/velocity_local.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #VELOCITY GRAPH LOCAL
        plt.figure()
        plt.title("3-axis velocity from single-integration depending on time in global frame of reference")
        plt.xlabel("Time/s")
        plt.ylabel("velocity")
        plt.plot(times, velocity_global[0], label = "velocity x")
        plt.plot(times, velocity_global[1], label = "velocity y")
        plt.plot(times, velocity_global[2], label = "velocity z")
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/velocity_global.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #DISTANCES GRAPH 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plt.title("Rocket path from acceleration data in global frame of reference")
        ax.set_xlabel('X distance')
        ax.set_ylabel('Y distance')
        ax.set_zlabel('Z distance')
        ax.plot(distances_global[0], distances_global[1], distances_global[2])
        if save_figure == True: 
            plt.savefig('output/rocket_path_global.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #DISTANCES GRAPH 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plt.title("Rocket path from acceleration data in local frame of reference")
        ax.set_xlabel('X distance')
        ax.set_ylabel('Y distance')
        ax.set_zlabel('Z distance')
        ax.plot(distances_local[0], distances_local[1], distances_local[2])
        if save_figure == True: 
            plt.savefig('output/rocket_path_local.png', bbox_inches='tight')
        if show_figure == True:    
            plt.show()
        
        #temp/press
        fig, ax1 = plt.subplots()
        plt.title("Temeprature and pressure depending on time")
        ax1.set_xlabel('Time/s')
        ax1.set_ylabel('Pressure/Pa', color = 'tab:blue')
        ax1.plot(times, press, color = 'tab:blue')
        ax1.tick_params(axis='y', labelcolor = 'tab:blue')
        ax2 = ax1.twinx()
        ax2.set_ylabel('Temperature/degrees Celsius', color = 'tab:red')
        ax2.plot(times, temp, color = 'tab:red')
        ax2.tick_params(axis='y', labelcolor = 'tab:red')
        fig.tight_layout()
        if save_figure == True: 
            plt.savefig('output/temp_press.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #DISTANCES GRAPH 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plt.title("Rocket path from acceleration data in local frame of reference")
        ax.set_xlabel('X distance')
        ax.set_ylabel('Y distance')
        ax.set_zlabel('Z distance')
        ax.plot(distances_local[0], distances_local[1], distances_local[2])
        if save_figure == True: 
            plt.savefig('output/distances_local.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #ALTITUDE
        plt.figure()
        plt.title("Altitude based on pressure")
        plt.xlabel("Time/s")
        plt.ylabel("Altitude/m")
        plt.plot(times, alt)
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/altitude.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #local Z acceleration
        plt.figure()
        plt.title("Vertical acceleration in local frame of reference")
        plt.xlabel("Time/s")
        plt.ylabel("Acceleration/m")
        plt.plot(times, accel[2])
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/accel_vertical_local.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        #global Z acceleration
        plt.figure()
        plt.title("Vertical acceleration in global frame of reference")
        plt.xlabel("Time/s")
        plt.ylabel("Acceleration/m")
        plt.plot(times, accel_global[2])
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/accel_vertical_global.png', bbox_inches='tight')
        if show_figure == True:    
            plt.show()
        
        #global Z displacement
        plt.figure()
        plt.title("Vertical displacement in global frame of reference")
        plt.xlabel("Time/s")
        plt.ylabel("Displacement/m")
        plt.plot(times, distances_global[2])
        plt.legend(loc="upper left")
        if save_figure == True: 
            plt.savefig('output/vertical_displacement_global.png', bbox_inches='tight')
        if show_figure == True:
            plt.show()
        
        """
        #read magnetometer
        mag = [data.T[7].copy().astype('float64'), data.T[8].copy().astype('float64'), data.T[9].copy().astype('float64')]
        plt.figure()
        plt.title("Magnetometer data depending on time")
        plt.xlabel("Time/ms")
        plt.ylabel("???")
        plt.plot(times, mag[0], label = "magnetometer x")
        plt.plot(times, mag[1], label = "magnetometer y")
        plt.plot(times, mag[2], label = "magnetometer z")
        plt.legend(loc="upper left")
        plt.show()
        """
        break