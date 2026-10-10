# miam_py python package

This package implements python tools developed for MiAM robotics team. These tools are mostly conserned with
plotting and analysing telemetry logs created by the robot C++ code.

Aside from the package (miam_py) itself, the following script are provided:

 - miam_plot: a generic plotter script. Specify a space-separated list of headers to plot them. Separate headers by
 colons for plotting on the same axes. See script for full help.
 - miam_analyse_match: a multi-tab viewer with synchronized map, tracking plots, custom plots, and text.
   Plot tabs have a collapsible **Text log** panel, folded by default. **Play/Pause** advances the shared
   time at real-time speed and pauses at the end of the log; pressing Play at the end restarts from the beginning.
   **Replay speed** selects 2x, 1x, 0.5x, 0.25x, or 0.1x and can change while playing without jumping in time.
   **Space** toggles replay regardless of which widget has focus.
   Clicking or editing the time selection controls, or seeking through plots or text, pauses playback.
   Playback targets 30 FPS by caching static drawings and redrawing only the moving markers and time cursors.
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
