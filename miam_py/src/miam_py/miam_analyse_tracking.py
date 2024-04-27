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
from matplotlib.backend_bases import NavigationToolbar2

from .tabbed_figure import TabbedFigure
from .utils import get_most_recent_log

# Custom home button to fix home view.
def new_home(self, *args, **kwargs):
    fig = plt.figure(1)
    plt.xlim(-500, 3500)
    plt.ylim(-500, 2500)
    fig.canvas.draw()
    fig.canvas.flush_events()

NavigationToolbar2.home = new_home


# Key press callback
def press(event):
    global current_aspect_ratio
    if event.key == 'a':
        if current_aspect_ratio == 'auto':
            current_aspect_ratio = 'equal'
        else:
            current_aspect_ratio = 'auto'
        redo_table_plot()


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
    window = TabbedFigure(logname)

    # Get robot parameters from file.
    time = logfile.data['MotionController.currentPositionX'][0]
    # Load robot x, y, theta position.
    x = get_data_resampled(logfile, 'MotionController.currentPositionX', time)
    y = get_data_resampled(logfile, 'MotionController.currentPositionY', time)
    theta = get_data_resampled(logfile, 'MotionController.currentPositionTheta', time)

    v_right = get_data_resampled(logfile, 'MotionController.commandVelocityRight', time)
    v_left = get_data_resampled(logfile, 'MotionController.commandVelocityLeft', time)

    v = get_data_resampled(logfile, 'MotionController.currentVelocityLinear', time)
    omega = get_data_resampled(logfile, 'MotionController.currentVelocityAngular', time)

    # Load trajectory x, y, theta, velocities.
    trajectory_x = get_data_resampled(logfile, 'MotionController.targetPositionX', time)
    trajectory_y = get_data_resampled(logfile, 'MotionController.targetPositionY', time)
    trajectory_theta = get_data_resampled(logfile, 'MotionController.targetPositionTheta', time)
    trajectory_v = get_data_resampled(logfile, 'MotionController.targetVelocityLinear', time)
    trajectory_omega = get_data_resampled(logfile, 'MotionController.targetVelocityAngular', time)


    # Create table plot, used for changing axis ratio.
    map_fig = plt.figure(1)
    def redo_table_plot():
        xlim = map_fig.gca().get_xlim()
        ylim = map_fig.gca().get_ylim()
        plt.clf()
        plt.text(0.01, 0.01, "Press a to change aspect ratio", transform=plt.gcf().transFigure)
        plt.plot(x, y, color='w', linewidth=5.0)
        plt.plot(trajectory_x, trajectory_y, color='w', linewidth=5.0)
        plt.plot(x, y, label="Current Position", linewidth=3.0)
        plt.plot(trajectory_x, trajectory_y, label="Target Position", linewidth=3.0)
        plt.xlim(xlim)
        plt.ylim(ylim)
        plt.grid()
        # Show table image as background.
        table = mpimg.imread(os.path.join(os.path.dirname(miam_py.__file__),'../images/table.png'))
        image = plt.imshow(table, extent=[-500, 3500, -500, 2500], alpha=0.6, aspect=current_aspect_ratio)
        plt.legend(bbox_to_anchor=(1.0, 1.0), loc = 1)
        map_fig.canvas.draw()
        map_fig.canvas.flush_events()

    # First plot: plot (x,y) position on playing field.
    # Callback on letter a to change image aspect ratio
    map_fig.canvas.mpl_connect('key_press_event', press)
    current_aspect_ratio = 'equal'
    redo_table_plot()
    plt.xlim(-500, 3500)
    plt.ylim(-500, 2500)

    window.add_plot("map", map_fig)

    fig, axs = plt.subplots(2, 3, sharex = True)
    fig.tight_layout()

    xc =  [x, y, theta]
    xt = [trajectory_x, trajectory_y, trajectory_theta]
    label = ["x", "y", "angle"]
    for i in range(len(xc)):
        axs[0][i].plot(time, xc[i], label=f"Current {label[i]}")
        axs[0][i].plot(time, xt[i], label=f"Target {label[i]}")
        axs[1][i].plot(time, xc[i] - xt[i], label=f"Error {label[i]}")

    for ax in axs:
        for a in ax:
            a.legend(bbox_to_anchor=(1.0, 1.0), loc = 1)
            a.grid()
    fig.suptitle(logname)

    window.add_plot("Position error", fig)

    # Error in each component, plus velocity
    fig, axs = plt.subplots(nrows=2, ncols=2, sharex = True)
    fig.tight_layout()

    axs[0][0].plot(time, theta, label="Current angle")
    axs[0][0].plot(time, trajectory_theta, label="Target angle")
    axs[0][0].plot(time, [theta[i] - trajectory_theta[i] for i in range(len(theta))], label="Error")

    axs[0][1].plot(time, get_data_resampled(logfile, 'trackingLongitudinalError', time), label="Longitudinal error")
    axs[0][1].plot(time, get_data_resampled(logfile, 'trackingTransverseError', time), label="Transverse error")

    axs[1][0].plot(time, v, label="Linear velocity")
    axs[1][0].plot(time, trajectory_v, label="Trajectory linear velocity")
    axs[1][0].plot(time, get_data_resampled(logfile, 'MotionController.linearPIDCorrection', time), label="PID correction")

    axs[1][1].plot(time, omega, label="Angular velocity")
    axs[1][1].plot(time, trajectory_omega, label="Trajectory angular velocity")
    axs[1][1].plot(time, get_data_resampled(logfile, 'MotionController.angularPIDCorrection', time), label="PID correction")

    axs[0][0].set_title("Angle (rad)")
    axs[0][1].set_title("Position error (mm)")
    axs[1][0].set_title("Linear velocity (mm/s)")
    axs[1][1].set_title("Angular velocity (rad/s)")
    for ax in axs:
        for a in ax:
            a.legend(bbox_to_anchor=(1.0, 1.0), loc = 1)
            a.grid()
    fig.suptitle(logname)
    window.add_plot("Tracking", fig)
    window.show()

if __name__ == "__main__":
    main()