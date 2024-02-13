#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Plot a MiAM log file using matplotlib.
    See inline help for more info.
'''

import argparse
import h5py
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description = "Extract text log from h5df file.")
    parser.add_argument("input", type = Path, help = "Input HDF5 log file.")
    # parser.add_argument("-fft", required = False, default = False, action = "store_true",
    #                     help = "If set, plot Fourier transform instead.")
    parser.add_argument("--file", nargs='?', type=Path, default=argparse.SUPPRESS, help = "If set, output to text file (flag alone = log_name.txt). If unset, content is printed to the terminal")
    args = parser.parse_args()
    output_to_file = hasattr(args, 'file')

    with (h5py.File(args.input)) as f:
        data = f["textLog/log"]
        string_data  = '\n'.join([s.decode() for s in f["textLog/log"]])

    if output_to_file:
        filename = args.file
        if filename is None:
            filename = args.input.with_suffix(".txt")
        with open(filename, "w") as f:
            f.write(string_data + "\n")
        print(f"Content saved to file '{filename}'")
    else:
        print(string_data)

if __name__ == "__main__":
    main()