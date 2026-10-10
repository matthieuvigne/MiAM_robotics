import re
import os
import shlex
from math import ceil
from fnmatch import filter as matching
from typing import Collection, Union

def sorted_nicely(l):
    """ Sort the given iterable in the way that humans expect."""
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)

def get_most_recent_log():
    ''' Return the most recent log of the current folder'''
    logfiles = [i for i in os.listdir('.') if i.startswith('log') and i.endswith("hdf5")]
    return sorted_nicely(logfiles)[-1]


def plot_layout(n_plots: int) -> tuple[int, int]:
    """Return (rows, columns), keeping columns at most two greater than rows."""
    rows, columns = 1, n_plots
    while columns > rows + 2:
        rows += 1
        columns = ceil(n_plots / rows)
    return rows, columns


def expand_plot_specs(
    commands: Union[str, list[str]], variables: Collection[str], strict: bool = True,
) -> list[list[str]]:
    """Expand the wildcard/colon syntax used by miam_plot."""
    if isinstance(commands, str):
        commands = shlex.split(commands)
    plots = []
    for command in commands:
        patterns = command.split(":")
        matches = [sorted(matching(variables, pattern)) for pattern in patterns]
        if strict and any(not group for group in matches):
            missing = [pattern for pattern, group in zip(patterns, matches) if not group]
            raise ValueError("No channels match: " + ", ".join(missing))
        if strict and len({len(group) for group in matches}) > 1:
            raise ValueError(f"Unequal channel counts in '{command}': "
                             + ", ".join(f"{p}: {len(g)}" for p, g in zip(patterns, matches)))
        plots.extend([group[i] for group in matches] for i in range(min(map(len, matches))))
    if strict and not plots:
        raise ValueError("Enter at least one channel or wildcard expression.")
    return plots
