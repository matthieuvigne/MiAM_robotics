# miam_py python package

This package implements python tools developed for MiAM robotics team. These tools are mostly conserned with
plotting and analysing telemetry logs created by the robot C++ code.

Aside from the package (miam_py) itself, the following script are provided:

 - miam_plot: a generic plotter script. Specify a space-separated list of headers to plot them. Separate headers by
 colons for plotting on the same axes. See script for full help.
 - miam_analyse_match: a multi-tab viewer with synchronized map, tracking plots, custom plots, and text.
 - miam_analyse_tracking: performs specific plots aiming at helping understanding robot servoing on the table. This script
 plots the robot position and target position on the table (with table image as background), as well as position, angle,
 velocity and angular velocity values or errors.

# Installing

Simply run from the package main directory:

```
python -m pip install -e .
```

# MiAM log format

The robot records telemetry in binary `.miam` files containing named numeric time series and timestamped text messages.
`miam_log_converter` converts these files to `.hdf5`; the Python loader also runs this converter automatically when
given a `.miam` file, provided the converter is available on `PATH`.

In the HDF5 file, each numeric channel is a dataset named after the variable (for example,
`MotionController.currentPositionX`). Its shape is `(2, N)`: the first row contains timestamps in seconds,
and the second contains values. Channels can have different sampling times and lengths. Text messages are stored
in the string dataset `textLog/log`, with each timestamped message prefixed by `[time]`. Numeric and text timestamps
use the same elapsed-time clock, including in simulation with the corrected logger.

Legacy CSV logs are also supported. They begin with an info line containing `Robot Log: <log_id>` and optional
comma-separated metadata, followed by a header line naming the numeric columns. Each subsequent row contains one
value per column. A `time` column, when present, supplies timestamps for all channels; otherwise, the loader uses
sample indices. CSV logs do not contain the HDF5 text-log dataset.

## Images

```miam_analyse_plot``` uses an image of the table as background for showing the robot trajectory.
The wheel includes `src/miam_py/images/table.png`, loaded with `importlib.resources` rather than a path relative
to `__file__`. The background is displayed at full opacity. This file must contain an image such that 50cm of whitespace are added
around the table (i.e. overage image size is 3x4m, with the tabled centered).
