"""Telemetry preparation and lookup, independent of the GUI."""

from dataclasses import dataclass
import re
from typing import Optional, Union

import numpy as np

from ..log_loader import LogLoader


# Seconds: timestamps within 10 microseconds refer to the same system instant.
TIME_TOLERANCE = 10e-6
MIN_GAP_SECONDS = 0.05
GAP_INTERVAL_FACTOR = 5


@dataclass(frozen=True)
class TextEntry:
    """A native text message and its log timestamp."""

    time: float
    message: str
    raw: str


class Series:
    """Read-only time series, with optional (opt-out) gap handling and duplicate removal"""

    def __init__(self, times: np.ndarray, values: np.ndarray, sanitize_gaps: bool = True) -> None:
        self._angle_values = None
        if sanitize_gaps:
            # Retain the last value when a numeric channel repeats a timestamp.
            keep = np.r_[np.diff(times) != 0, True] if len(times) else np.array([], dtype=bool)
            self.times, self.values = (times, values) if keep.all() else (times[keep], values[keep])
            intervals = np.diff(self.times)
            # Permit scheduling jitter, but break unusually long sampling intervals.
            self.gap_limit = (max(MIN_GAP_SECONDS, GAP_INTERVAL_FACTOR * np.median(intervals))
                              if len(intervals) else MIN_GAP_SECONDS)
        else:
            self.times, self.values = times, values
            # State holds are not recording gaps, however long they last.
            self.gap_limit = float("inf")

    def sample(self, times: Union[float, np.ndarray], angle: bool = False) -> np.ndarray:
        """Interpolate values, unwrapping angles if requested; gaps return NaN."""
        query = times
        if not len(self.times):
            return np.full(np.shape(query), np.nan)
        values = self.values
        if angle:
            if self._angle_values is None:
                self._angle_values = np.unwrap(self.values)
            values = self._angle_values
        right = np.searchsorted(self.times, query, side="left")
        bounded = np.clip(right, 0, len(self.times) - 1)
        left = np.clip(right - 1, 0, len(self.times) - 1)
        query = np.where(np.isclose(query, self.times[bounded], rtol=0, atol=TIME_TOLERANCE),
                         self.times[bounded], query)
        query = np.where(np.isclose(query, self.times[left], rtol=0, atol=TIME_TOLERANCE),
                         self.times[left], query)
        result = np.interp(query, self.times, values, left=np.nan, right=np.nan)
        in_gap = ((right > 0) & (right < len(self.times))
                  & (self.times[bounded] - self.times[left] > self.gap_limit)
                  & (query != self.times[bounded]) & (query != self.times[left]))
        return np.where(in_gap, np.nan, result)

    def plot_data(self) -> tuple[np.ndarray, np.ndarray]:
        """Return timestamps and values with breaks at recording gaps."""
        breaks = np.flatnonzero(np.diff(self.times) > self.gap_limit) + 1
        if not len(breaks):
            return self.times, self.values
        return (np.insert(self.times, breaks, self.times[breaks]),
                np.insert(self.values, breaks, np.nan))

    def sample_exact(self, time: float) -> float:
        """Return a recorded value at this time, or NaN if absent."""
        right = int(np.searchsorted(self.times, time))
        for index in (right, right - 1):
            if 0 <= index < len(self.times) and abs(self.times[index] - time) <= TIME_TOLERANCE:
                return float(self.values[index])
        return float("nan")


class MatchData:
    """Prepared telemetry and text from one robot log."""

    def __init__(self, logfile: LogLoader) -> None:
        self.filename = logfile.filename
        self.match_start_time = logfile.match_start_time
        self.series = {
            name: Series(times, values, sanitize_gaps=name not in ("Strategy.state", "MotionController.state"))
            for name, (times, values) in logfile.data.items()
        }
        self.obstacle_count = self.series["lidarNumberOfObstacles"]
        slots = sorted(int(match.group(1)) for name in self.series
                       if (match := re.fullmatch(r"ObstacleX_(\d+)", name)))
        self.obstacle_channels = {slot: (self.series[f"ObstacleX_{slot}"], self.series[f"ObstacleY_{slot}"])
                                  for slot in slots}
        self.entries = []
        for raw in logfile.text_log:
            match = re.match(r"^\[(-?\d+\.\d{6})\] ", raw)
            if match is None:
                raise ValueError(f"Invalid native text log timestamp: {raw!r}")
            self.entries.append(TextEntry(float(match.group(1)), raw[match.end():], raw))
        # Deterministic nearest lookup; an explicit text click can select any duplicate.
        self._text_times, self._text_rows = np.unique([entry.time for entry in self.entries],
                                                    return_index=True)
        bounds = [(s.times[0], s.times[-1]) for s in self.series.values() if len(s.times)]
        if len(self._text_times):
            bounds.append((self._text_times[0], self._text_times[-1]))
        self.minimum = min((lo for lo, _ in bounds), default=0.0)
        self.maximum = max((hi for _, hi in bounds), default=self.minimum)

    def nearest_entry(self, time: float) -> Optional[int]:
        """Return the nearest timed text row, choosing the first row on ties."""
        if not len(self._text_times):
            return None
        right = int(np.searchsorted(self._text_times, time))
        candidates = [i for i in (right - 1, right) if 0 <= i < len(self._text_times)]
        closest = min(candidates, key=lambda i: (abs(self._text_times[i] - time), i))
        return int(self._text_rows[closest])

    def pose(self, time: float, target: bool = False) -> Optional[tuple[float, float, float]]:
        """Return actual or target (x, y, angle), or None where pose data is missing."""
        prefix = "MotionController." + ("targetPosition" if target else "currentPosition")
        names = [prefix + component for component in ("X", "Y", "Theta")]
        pose = tuple(float(self.series[name].sample(time, angle=name.endswith("Theta"))) for name in names)
        return pose if np.isfinite(pose).all() else None

    def obstacles(self, time: float) -> list[tuple[float, float]]:
        """Return obstacle positions from the latest recorded count frame."""
        frames = self.obstacle_count
        if not len(frames.times) or not self.obstacle_channels:
            return []
        index = int(np.searchsorted(frames.times, time + TIME_TOLERANCE, side="right")) - 1
        if (index < 0 or time > frames.times[-1] + TIME_TOLERANCE
                or time - frames.times[index] > frames.gap_limit):
            return []
        frame_time = frames.times[index]
        count = frames.values[index]
        result = []
        for slot, (x_channel, y_channel) in self.obstacle_channels.items():
            if slot >= count:
                continue
            x = x_channel.sample_exact(frame_time)
            y = y_channel.sample_exact(frame_time)
            if not np.isfinite([x, y]).all():
                continue
            result.append((x, y))
        return result
