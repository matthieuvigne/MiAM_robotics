"""Plot definitions for the default position and velocity tabs."""

import numpy as np

from .data import MatchData
from .plot_view import Plot


PREFIX = "MotionController."


def configure_default_plots(data: MatchData) -> dict[str, list[Plot]]:
    """Build the titled position and velocity plot tabs from recorded channels."""
    output: dict[str, list[Plot]] = {}
    positions, errors = [], []
    for suffix, unit in (("X", "mm"), ("Y", "mm"), ("Theta", "rad")):
        current = data.series[PREFIX + "currentPosition" + suffix]
        target = data.series[PREFIX + "targetPosition" + suffix]
        times, values = current.plot_data()
        positions.append((f"{suffix} ({unit})", [("Current", times, values),
                                                ("Target", *target.plot_data())]))
        difference = values - target.sample(times, angle=suffix == "Theta")
        if suffix == "Theta":
            difference = (difference + np.pi) % (2 * np.pi) - np.pi
        error = ("Current − target", times, difference)
        if suffix == "Theta":
            angular_error = error
        errors.append((f"{suffix} error", [error]))
    output["Robot position"] = positions + errors

    groups = [
        ("Linear velocity (mm/s)", [("currentVelocityLinear", "Current"),
                                   ("targetVelocityLinear", "Target"),
                                   ("linearPIDCorrection", "PID correction")]),
        ("Angular velocity (rad/s)", [("currentVelocityAngular", "Current"),
                                     ("targetVelocityAngular", "Target"),
                                     ("angularPIDCorrection", "PID correction")]),
        ("Tracking error (mm)", [("trackingLongitudinalError", "Longitudinal"),
                                 ("trackingTransverseError", "Transverse")]),
    ]
    plots = [(title, [(label, *data.series[PREFIX + suffix].plot_data())
                     for suffix, label in channels]) for title, channels in groups]
    plots.append(("Angular error (rad)", [angular_error]))
    output["Velocity tracking"] = plots
    return output
