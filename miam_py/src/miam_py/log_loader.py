'''
    Defines class LogLoader, a class for helping loading of the log CSV.
'''
import numpy as np
import csv
import os

class LogLoader:
    def __init__(self, filename):
        ''' Load a log file.
        @param filename Log file name
        '''
        # Load log file
        file_data = np.genfromtxt(filename, delimiter=',', skip_header = 2)
        f = open(filename, "r")
        reader = csv.reader(f)
        self.log_info = reader.__next__()
        self.log_name = self.log_info[0].replace('Robot Log: ', '')
        self.filename = os.path.basename(filename)
        # Get header list - keep it to have an ordered list along with the keys.
        self.headers = reader.__next__()
        f.close()

        # Format data in a dictionnary.
        self.data = {}
        for i in range(len(self.headers)):
            self.data[self.headers[i]] = file_data[:, i]
        self.log_length = len(self.data[self.headers[0]])

        # If time is present, create a special variable time and dt (time increment):
        if 'time' in self.headers:
            self.time = self.data['time']
            self.dt = np.concatenate((np.array([0]), np.diff(self.time)))
            self.dt[0] = self.dt[1]
            if 'timeDiffs' not in self.headers:
                self.data['timeDiffs'] = self.dt
                self.headers.append('timeDiffs')
