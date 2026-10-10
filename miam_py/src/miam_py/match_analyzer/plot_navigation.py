"""Shared cursor, seeking, and viewport navigation for time plots."""

from typing import Optional

from matplotlib.axes import Axes
from matplotlib.backend_bases import MouseEvent
from matplotlib.lines import Line2D
from PyQt5.QtCore import QEvent, QObject, QTimer
from PyQt5.QtWidgets import QWidget

from .figure_view import FigurePane
from .timeline import TimeController


class PlotNavigation(QObject):
    """Own time-plot interaction while the view owns its figure and axes."""

    def __init__(self, view: QWidget, pane: FigurePane, axes: list[Axes],
                 cursors: list[Line2D], controller: TimeController) -> None:
        super().__init__(view)
        self._view = view
        self._pane = pane
        self._axes = axes
        self._cursors = cursors
        self._controller = controller
        self._reveal_pending = False
        self._dragging = False
        self._pending_time = None
        self._seek_timer = QTimer(self)
        self._seek_timer.setSingleShot(True)
        self._seek_timer.setInterval(30)
        self._seek_timer.timeout.connect(self._flush_seek)
        self._pane.canvas.mpl_connect("button_press_event", self._press)
        self._pane.canvas.mpl_connect("motion_notify_event", self._motion)
        self._pane.canvas.mpl_connect("button_release_event", self._release)
        pane.animate(cursors)
        controller.changed.connect(self._update_time)
        view.installEventFilter(self)

    def _press(self, event: MouseEvent) -> None:
        if event.button == 1 and event.inaxes in self._axes and not self._pane.toolbar.mode:
            self._dragging = True
            self._controller.set_time(event.xdata)

    def _motion(self, event: MouseEvent) -> None:
        if self._dragging and not self._pane.toolbar.mode and event.inaxes in self._axes:
            self._pending_time = event.xdata
            if not self._seek_timer.isActive():
                self._seek_timer.start()

    def _release(self, event: MouseEvent) -> None:
        if self._dragging:
            if event.inaxes in self._axes and event.xdata is not None:
                self._pending_time = event.xdata
            self._flush_seek()
        self._dragging = False
        self._seek_timer.stop()

    def _flush_seek(self) -> None:
        if self._pending_time is not None:
            self._controller.set_time(self._pending_time)
            self._pending_time = None
        self._seek_timer.stop()

    def _update_time(self, _time: float, _entry: Optional[int]) -> None:
        self._reveal_pending = True
        self._refresh()

    def _refresh(self) -> None:
        if not self._view.isVisible():
            return
        time = self._controller.time
        for cursor in self._cursors:
            cursor.set_xdata([time, time])
        if self._reveal_pending and self._axes:
            lo, hi = self._axes[0].get_xlim()
            if not lo <= time <= hi:
                width = hi - lo
                self._axes[0].set_xlim(time - width / 2, time + width / 2)
        self._reveal_pending = False
        self._pane.render()

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Show:
            self._refresh()
        elif event.type() == QEvent.Hide:
            self._seek_timer.stop()
            self._pending_time = None
            self._dragging = False
        return super().eventFilter(watched, event)
