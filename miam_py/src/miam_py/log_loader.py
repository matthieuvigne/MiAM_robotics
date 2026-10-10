'''
    Defines class LogLoader, a class for helping loading of the log CSV.
'''
import numpy as np
import csv
from pathlib import Path
from subprocess import run

import h5py


class LogLoader:
    def __init__(self, filename):
        ''' Load a log file.
        @param filename Log file name
        '''
        p = Path(filename)
        self.filename = p.stem
        self.text_log = []
        self.match_start_time = 0.0
        if p.suffix == ".csv":
            self._from_csv(p)
        elif p.suffix == ".miam":
            run(["miam_log_converter", str(p)], check=True)
            self._from_hdf5(p.with_suffix(".hdf5"))
        else:
            self._from_hdf5(p)

        for message in self.text_log:
            if "Starting match" in message and message.startswith("["):
                try:
                    timestamp = float(message[1:].split("]", 1)[0])
                except ValueError:
                    continue
                if np.isfinite(timestamp):
                    self.match_start_time = timestamp

    def _from_hdf5(self, filepath):
        with (h5py.File(filepath)) as f:
            self.data = {}
            if "textLog/log" in f:
                self.text_log = f["textLog/log"].asstr(errors="replace")[:].tolist()
            for n in f:
                if n == "textLog":
                    continue
                val = np.array(f[n])
                self.data[n] = (val[0], val[1])

        self.variables = list(self.data.keys())

    def _from_csv(self, filepath):
        # Load log file
        file_data = np.genfromtxt(filepath, delimiter=',', skip_header = 2, ndmin=2)
        f = open(filepath, "r")
        reader = csv.reader(f)
        self.log_info = reader.__next__()
        self.log_name = self.log_info[0].replace('Robot Log: ', '')
        # Get header list - keep it to have an ordered list along with the keys.
        self.variables = reader.__next__()
        f.close()

        # Format data in a dictionnary.
        self.data = {}
        for i in range(len(self.variables)):
            self.data[self.variables[i]] = (range(len(file_data[:, i])), file_data[:, i])
        self.log_length = len(file_data)

        # If time is present, use it for all variables
        if 'time' in self.variables:
            for v in self.variables:
                self.data[v] = (self.data["time"][1], self.data[v][1])
