"""Launch the synchronized map, plot, and text-log viewer."""

import argparse
import sys
from subprocess import CalledProcessError

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

from .log_loader import LogLoader
from .match_analyzer.data import MatchData
from .match_analyzer.window import MatchWindow
from .utils import expand_plot_specs, get_most_recent_log


def main() -> int:
    """Load a match and run its synchronized analysis window."""
    parser = argparse.ArgumentParser(description="Analyse a match with synchronized map, plots, and text.")
    parser.add_argument("input", help="HDF5 or .miam log; 'latest' selects the latest HDF5 in this directory.")
    parser.add_argument("--plot", action="append", default=[], help="Add a custom tab using miam_plot wildcard/colon syntax.")
    args = parser.parse_args()
    try:
        filename = get_most_recent_log() if args.input == "latest" else args.input
        data = MatchData(LogLoader(filename))
    except KeyError as error:
        parser.exit(2, f"Missing required telemetry channel: {error.args[0]}\n")
    except (OSError, ValueError, IndexError, RuntimeError, CalledProcessError) as error:
        parser.exit(2, f"Cannot load log: {error}\n")
    try:
        initial_plots = [expand_plot_specs(expression, data.series) for expression in args.plot]
    except ValueError as error:
        parser.exit(2, f"Invalid plot expression: {error}\n")
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    app = QApplication(sys.argv)
    try:
        window = MatchWindow(data, initial_plots)
    except KeyError as error:
        parser.exit(2, f"Missing required telemetry channel: {error.args[0]}\n")
    except (OSError, ValueError) as error:
        parser.exit(2, f"Cannot initialise match viewer: {error}\n")
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
