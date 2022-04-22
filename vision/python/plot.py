#!/usr/bin/env python

import argparse
import csv
from os.path import exists
import numpy as np
import matplotlib.pyplot as plt

# Parse arguments
parser = argparse.ArgumentParser();
parser.add_argument("csv_file", type=str, default="");
args = parser.parse_args();
assert(exists(args.csv_file));

# Open the csv file
wx, wy, wz, tx, ty, tz = [], [], [], [], [], [];
csv_file = open(args.csv_file,'r');
for line in csv_file.readlines():
  line = line.replace('\n','');
  if line[0] == '#': continue;
  values = line.split(';');
  wx.append(float(values[0])); 
  wy.append(float(values[1]));
  wz.append(float(values[2]));
  tx.append(float(values[3]));
  ty.append(float(values[4]));
  tz.append(float(values[5]));

# Print the rotation errors
plt.plot(wx);
plt.plot(wy);
plt.plot(wz);
plt.show();

# Print the translation errors
plt.plot(tx);
plt.plot(ty);
plt.plot(tz);
plt.show();
