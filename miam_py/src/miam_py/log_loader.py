'''
    Defines class LogLoader, a class for helping loading of the log CSV.
'''
import numpy as np
import csv
from pathlib import Path

import h5py


class LogLoader:
    def __init__(self, filename):
        ''' Load a log file.
        @param filename Log file name
        '''
        p = Path(filename)
        self.filename = p.stem
        if p.suffix == ".csv":
            self._from_csv(p)
        else:
            self._from_hdf5(p)

    def _from_hdf5(self, filepath):
        with (h5py.File(filepath)) as f:
            self.data = {}
            # self.time = {}
            for n in f:
                val = np.array(f[n])
                self.data[n] = (val[0], val[1])
        self.variables = list(self.data.keys())


    def _from_csv(self, filepath):
        # Load log file
        file_data = np.genfromtxt(filepath, delimiter=',', skip_header = 2)
        f = open(filepath, "r")
        reader = csv.reader(f)
        self.log_info = reader.__next__()
        self.log_name = self.log_info[0].replace('Robot Log: ', '')
        # Get header list - keep it to have an ordered list along with the keys.
        self.variables = reader.__next__()
        f.close()

        # Format data in a dictionnary.
        self.data = {}
        for i in range(len(self.headers)):
            self.data[self.variables[i]] = (range(len(file_data[:, i])), file_data[:, i])
        self.log_length = len(self.data[self.headers[0]])

        # If time is present, use it for all variables
        if 'time' in self.variables:
            for v in self.variables:
                self.data[v] = (self.data["time"][1], self.data[v][1])