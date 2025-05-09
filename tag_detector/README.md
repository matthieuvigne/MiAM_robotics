# MiAM_robotics

This repo contains information and software for robotic-related projects done with the [MiAM Eurobot team](https://www.miam-robotique.fr/).

Our robot architecture are based off an embedded linux on an ARM platform, Raspberry Pi or Beaglebone Black, programmed
in C++. We also use Arduinos as slaves for low-level tasks such as motor control or sensor handling - using serial communication through
USB.

## Folder structure

 - [miam_utils](./miam_utils) : a very generic C++ library for handling all low-level activities. This ranges from sensor drivers to
 basic robot functions such as PID or basic data logging. This is the base code from which other projects spawn.
 Feel free to use this code in other projects !

 - [vacuum_cleaning](./vacuum_cleaning) : a tutorial project: building a vacuum-cleaning omnidirectional robot.

 - [miam_py](./miam_py) : a python package for reading the robot's logs

 - [eurobot2019](./eurobot2019) : main robot code for the Eurobot 2019 competition, to act as reference.
 See original repo at [https://github.com/matthieuvigne/MiAM_eurobot2019](https://github.com/matthieuvigne/MiAM_eurobot2019).
