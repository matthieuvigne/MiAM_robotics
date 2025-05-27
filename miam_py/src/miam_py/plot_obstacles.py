#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Display plots for helping with the analysis of trajectory tracking.
    See inline help for more info.
'''

import argparse

import os
import miam_py

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from matplotlib.widgets import Slider

def get_data_resampled(logfile: miam_py.LogLoader, var_name: str, time: np.array):
    '''
    Get data at specified resampling time
    '''
    t, d = logfile.data[var_name]
    return np.interp(time, t, d)

def main():
    description_str = "Extract and plot trajectory tracking information from a robot log file."

    parser = argparse.ArgumentParser(description = description_str, formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument("input", help = "Input csv log file.")
    args = parser.parse_args()

    # Load log file.
    logname = args.input
    if logname == 'latest':
        logname = get_most_recent_log()
        print("Loading log: {}".format(logname))

    logfile = miam_py.LogLoader(logname)

    # Get robot parameters from file.
    time = logfile.data['MotionController.currentPositionX'][0]
    time = time[np.searchsorted(time, logfile.start_time):]

    # Load robot x, y, theta position.
    trajectory_x = get_data_resampled(logfile, 'MotionController.currentPositionX', time)
    trajectory_y = get_data_resampled(logfile, 'MotionController.currentPositionY', time)
    trajectory_theta = get_data_resampled(logfile, 'MotionController.currentPositionTheta', time)

    # Load obstacle data.
    obstacles = [[] for _ in range(len(time))]

    i = 0
    done = False
    while not done:
        try:
            t, x = logfile.data[f"ObstacleX_{i}"]
            _, y = logfile.data[f"ObstacleY_{i}"]
            try:
                _, n = logfile.data[f"ObstacleN_{i}"]
            except KeyError:
                n = y * 0
            for k in range(len(t)):
                try:
                    j = np.where(time == t[k])[0][0]
                except:
                    print("error")
                obstacles[j].append((x[k], y[k], n[k]))
        except KeyError:
            done = True
        i += 1


    # Create table plot, used for changing axis ratio.
    fig, ax = plt.subplots()
    fig.subplots_adjust(bottom=0.25)

    def redo_table_plot():
        ax.plot(trajectory_x, trajectory_y, color='w', linewidth=5.0)
        ax.plot(trajectory_x, trajectory_y, label="Target Position", linewidth=3.0)
        ax.grid()
        ax.axis('equal')
        ax.set_xlim(0, 3000)
        ax.set_ylim(0, 2000)
        # Show table image as background.
        # table = mpimg.imread(os.path.join(os.path.dirname(miam_py.__file__),'../images/table.png'))
        # image = ax.imshow(table, extent=[0, 3000, 0, 2000], alpha=0.6, aspect='auto')
        # plt.legend(bbox_to_anchor=(1.0, 1.0), loc = 1)

    axfreq = fig.add_axes([0.25, 0.1, 0.65, 0.03])
    freq_slider = Slider(ax =  axfreq, label="Time (s)", valmin = time[0], valmax = time[-1], valinit = time[0])

    def update(idx):
        idx = np.searchsorted(time, idx)
        ax.clear()
        redo_table_plot()
        ax.scatter([o[0] for o in obstacles[idx]], [o[1] for o in obstacles[idx]], 1000)
        for obs in obstacles[idx]:
            ax.text(obs[0], obs[1], str(obs[2]))

        x, y, theta = trajectory_x[idx], trajectory_y[idx], trajectory_theta[idx]
        c = np.cos(theta)
        s = np.sin(theta)
        W = 100
        H = 100
        print([x - H * c + W * s, x - H * c + W * s, x + H * c],
              [y - H * s + W * c, y - H * s - W * c, y + H * s], theta)

        ax.fill([x - H * c - W * s, x - H * c + W * s, x + H * c],
                [y - H * s + W * c, y - H * s - W * c, y + H * s], color="red")
        # ax.scatter([trajectory_x[idx]], [trajectory_y[idx]], 1000, color='red')
        ax.text(0.01, 0.01, f"Time: {time[idx]}", transform=fig.transFigure)
        fig.canvas.draw_idle()

    freq_slider.on_changed(update)
    update(0.0)
    plt.show()


if __name__ == "__main__":
    main()