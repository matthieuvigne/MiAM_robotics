"""One selected timestamp shared by every match view."""

from typing import Optional

from PyQt5.QtCore import QElapsedTimer, QObject, Qt, QTimer, pyqtSignal

from .data import MatchData


REPLAY_SPEEDS = (2.0, 1.0, 0.5, 0.25, 0.1)
FRAME_INTERVAL_MS = 33  # Approximately 30 frames per second.


class TimeController(QObject):
    """Shared time selection and elapsed-time replay for all match views."""

    changed = pyqtSignal(float, object)
    playback_changed = pyqtSignal(bool)
    speed_changed = pyqtSignal(float)

    def __init__(self, data: MatchData, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._data = data
        self.time = data.minimum
        self.entry = data.nearest_entry(self.time)
        self.playing = False
        self.speed = 1.0
        self._clock = QElapsedTimer()
        self._play_start_time = self.time
        self._timer = QTimer(self)
        self._timer.setInterval(FRAME_INTERVAL_MS)
        self._timer.setTimerType(Qt.PreciseTimer)
        self._timer.timeout.connect(self._advance_playback)

    def set_time(self, time: float, entry: Optional[int] = None) -> None:
        """Pause, seek and reveal the selected time; an explicit row preserves text selection."""
        self.pause()
        self._set_time(time, entry)

    def _set_time(self, time: float, entry: Optional[int] = None) -> None:
        time = min(self._data.maximum, max(self._data.minimum, time))
        if entry is None:
            entry = self._data.nearest_entry(time)
        self.time, self.entry = time, entry
        self.changed.emit(time, entry)

    def toggle_playback(self) -> None:
        """Toggle between playing and paused."""
        if self.playing:
            self.pause()
        else:
            self.play()

    def play(self) -> None:
        """Start replay, restarting from the beginning if already at the end."""
        if self.playing or self._data.maximum <= self._data.minimum:
            return
        if self.time >= self._data.maximum:
            self.set_time(self._data.minimum)
        self._play_start_time = self.time
        self._clock.start()
        self.playing = True
        self._timer.start()
        self.playback_changed.emit(True)

    def pause(self) -> None:
        """Stop replay at the current selected time."""
        if self.playing:
            self._timer.stop()
            self.playing = False
            self.playback_changed.emit(False)

    def set_speed(self, speed: float) -> None:
        """Set the replay multiplier without jumping or pausing."""
        if speed == self.speed:
            return
        if self.playing:
            self._advance_playback()
            self._play_start_time = self.time
            self._clock.restart()
        self.speed = speed
        self.speed_changed.emit(self.speed)

    def _advance_playback(self) -> None:
        if not self.playing:
            return
        time = self._play_start_time + self.speed * self._clock.nsecsElapsed() / 1e9
        self._set_time(time)
        if time >= self._data.maximum:
            self.pause()
