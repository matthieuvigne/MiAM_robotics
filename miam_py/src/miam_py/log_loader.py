"""Load robot telemetry from HDF5 or converted .miam logs."""
import numpy as np
from pathlib import Path
from subprocess import run

import h5py

class LogLoader:
    def __init__(self, filename):
        ''' Load a log file.
        @param filename Log file name
        '''
        p = Path(filename)
        if p.suffix == ".miam":
            run(["miam_log_converter", str(p)], check=True)
            p = p.with_suffix(".hdf5")
        self.filename = p.stem
        self.text_log = []
        self._load_hdf5(p)

        self.match_start_time = 0.0
        for message in self.text_log:
            if "Starting match" in message and message.startswith("["):
                try:
                    timestamp = float(message[1:].split("]", 1)[0])
                except ValueError:
                    continue
                if np.isfinite(timestamp):
                    self.match_start_time = timestamp

    def _load_hdf5(self, filepath):
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
