"""Recorded state intervals and match-only duration totals."""

from .data import MatchData


# Values follow StrategyState and MotionControllerState in the C++ headers.
STATE_STYLES: dict[str, tuple[str, dict[int, tuple[str, str]]]] = {
    "Strategy.state": ("Strategy", {
        0: ("Startup", "#a0a0a0"),
        1: ("Choosing action", "#e6ab02"),
        2: ("Moving to action start", "#377eb8"),
        3: ("Performing action", "#4daf4a"),
        4: ("End-of-match action", "#984ea3"),
        5: ("Match done", "#505050"),
    }),
    "MotionController.state": ("Motion", {
        0: ("Stopped", "#e41a1c"),
        1: ("Trajectory tracking", "#4daf4a"),
        2: ("Waiting for avoidance", "#ff7f00"),
        3: ("Waiting for trajectory", "#377eb8"),
    }),
}
UNRECORDED_COLOR = "#000000"
StateInterval = tuple[float, float, int]


def state_durations(intervals: list[StateInterval], start: float, end: float) -> dict[int, float]:
    """Sum time in each state inside [start, end), omitting zero-duration states."""
    totals: dict[int, float] = {}
    for first, last, state in intervals:
        duration = min(last, end) - max(first, start)
        if duration > 0:
            totals[state] = totals.get(state, 0.0) + duration
    return totals


class MatchStates:
    """Shared state intervals for navigation and duration totals inside the recorded match."""

    def __init__(self, data: MatchData) -> None:
        self.start = data.match_start_time
        self.end = self.start + 100.0
        self.intervals: dict[str, list[StateInterval]] = {}
        for name, (_, styles) in STATE_STYLES.items():
            series = data.series[name]
            for state in series.values:
                if state not in styles:
                    raise ValueError(f"Unsupported state value {state} in {name}")
            self.intervals[name] = [
                (float(start), float(end), int(state))
                for start, end, state in zip(series.times, [*series.times[1:], data.maximum], series.values)
                if end > start
            ]
        self.durations = {name: state_durations(intervals, self.start, self.end)
                          for name, intervals in self.intervals.items()}
