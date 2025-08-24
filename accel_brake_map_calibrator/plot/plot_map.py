#! /usr/bin/python3
from mpl_toolkits import mplot3d

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def read_csv(file_path: str):
    data_frame = pd.read_csv(file_path)
    velocity = list(data_frame.columns)
    pedal = list(data_frame[velocity[0]])
    velocity = velocity[1:]
    for i in range(len(velocity)):
        velocity[i] = float(velocity[i])
    x = np.array(velocity)
    y = np.array(pedal)
    z = []
    for i in range(len(velocity)):
        for j in range(len(pedal)):
            z.append(data_frame.loc[i][j])
    z = np.array(z)
    x, y = np.meshgrid(x, y)
    z = z.reshape((len(velocity), len(pedal)))
    return x, y, z


def plot_calibration(x, y, z, map_title):

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.contour3D(x, y, z, 50, cmap='binary')
    ax.set_xlabel('velocity(m/s)')
    ax.set_ylabel('pedal')
    ax.set_zlabel('accel(m/s^s)')
    ax.plot_surface(x, y, z, rstride=1, cstride=1,
                cmap='viridis', edgecolor='none')
    ax.set_title(map_title+'_calibration')



if __name__ == "__main__":
    accel_map_csv_path = "/home/ahua/mark_dir/bag/dynamics_calib/accel_map.csv"
    brake_map_csv_path = "/home/ahua/mark_dir/bag/dynamics_calib/brake_map.csv"
    X_a, Y_a, Z_a = read_csv(accel_map_csv_path)
    plot_calibration(X_a, Y_a, Z_a, 'accel')
    X_b, Y_b, Z_b = read_csv(brake_map_csv_path)
    plot_calibration(X_b, Y_b, Z_b, 'brake')

    plt.show()