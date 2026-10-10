#!/usr/bin/env python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Plot a MiAM log file using matplotlib.
    See inline help for more info.
'''

from miam_py import LogLoader
import argparse
import os

import numpy as np
import matplotlib.pyplot as plt

from scipy.fftpack import fft

from .utils import expand_plot_specs, get_most_recent_log, plot_layout


def my_fft(data, dt):
    # Number of sample points
    N = len(data)

    data_fft = fft(np.matrix(data).A1)
    frequencies = np.linspace(0.0, 1.0/(2.0*dt), N//2)

    return frequencies, 2.0/N * np.abs(data_fft[0:N//2])


def main():
    description_str = "Plot data from a robot log file.\n" + \
                      "Specify a list of headers, separated by a colon for plotting on the same subplot.\n" + \
                      "Example: h1 h2:h3 generates two subplots, one with h1, one with h2 and h3.\n" + \
                      "Extra values: timeDiffs plots dt between two log lines."

    parser = argparse.ArgumentParser(description = description_str, formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument("input", help = "Input HDF5 or .miam log file, or keyword 'latest'.")
    parser.add_argument("-fft", required = False, default = False, action = "store_true",
                        help = "If set, plot Fourier transform instead.")
    main_arguments, plotting_commands = parser.parse_known_args()

    # Load log file.
    # Process latest argument.
    logname = main_arguments.input
    if logname == 'latest':
        logname = get_most_recent_log()
        print("Loading log: {}".format(logname))
    logfile = LogLoader(logname)

    if len(plotting_commands) == 0:
        print("Available data:")
        for h in logfile.variables:
            print(" - {}".format(h))
        exit(0)

    # Parse plotting arguments.
    plotted_elements = expand_plot_specs(plotting_commands, logfile.variables, strict=False)
    if not plotted_elements:
        parser.error("No channels match the plotting expressions.")


    # Create figure.
    n_plot = len(plotted_elements)

    n_rows, n_cols = plot_layout(n_plot)
    _, axs = plt.subplots(nrows=n_rows, ncols=n_cols, sharex = True)

    if n_plot == 1:
        axs = np.array([axs])
    axs = axs.flatten()

    plt.suptitle(logfile.filename)

    # Plot each element.
    for i in range(n_plot):
        for name in plotted_elements[i]:
            if main_arguments.fft:
                f, v = my_fft(logfile.data[name][1], np.average(np.diff(logfile.data[name][0])))
                axs[i].plot(f, v, label = name)
            else:
                axs[i].plot(logfile.data[name][0], logfile.data[name][1], label = name)
    # Add legend to upper left corner.
    for ax in axs:
        ax.legend(bbox_to_anchor=(1.0, 1.0), loc = 1)
        ax.grid()
    plt.subplots_adjust(bottom=0.05, top=0.92, left=0.06, right=0.98, wspace=0.1, hspace=0.05)
    plt.show()

if __name__ == "__main__":
    main()
