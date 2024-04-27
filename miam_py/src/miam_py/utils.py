import re
import os

def sorted_nicely(l):
    """ Sort the given iterable in the way that humans expect."""
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)

def get_most_recent_log():
    ''' Return the most recent log of the current folder'''
    logfiles = [i for i in os.listdir('.') if i.startswith('log') and i.endswith("hdf5")]
    return sorted_nicely(logfiles)[-1]