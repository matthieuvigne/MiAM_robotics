"""Matplotlib canvas, toolbar, and cached rendering shared by map and plots."""

from typing import Iterable

from matplotlib.artist import Artist
from matplotlib.axes import Axes
from matplotlib.backend_bases import DrawEvent
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt5.QtGui import QShowEvent
from PyQt5.QtWidgets import QScrollArea, QVBoxLayout, QWidget


class FigurePane(QWidget):
    """A canvas and toolbar with cached rendering for moving artists."""

    def __init__(self, parent: QWidget, scrollable: bool = False) -> None:
        super().__init__(parent)
        self.figure = Figure(layout="constrained")
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.toolbar)
        if scrollable:
            area = QScrollArea()
            area.setWidgetResizable(True)
            area.setWidget(self.canvas)
            layout.addWidget(area, 1)
        else:
            layout.addWidget(self.canvas, 1)
        self._artists = []
        self._background = None
        self.canvas.mpl_connect("draw_event", self._cache_background)
        self.canvas.mpl_connect("resize_event", self.invalidate)

    def watch_axes(self, axes: Axes) -> None:
        """Invalidate the drawing cache when these axes change limits."""
        axes.callbacks.connect("xlim_changed", self.invalidate)
        axes.callbacks.connect("ylim_changed", self.invalidate)

    def animate(self, artists: Iterable[Artist]) -> None:
        """Register the moving artists to redraw over the cached background."""
        self._artists = sorted(artists, key=lambda artist: artist.get_zorder())
        for artist in self._artists:
            artist.set_animated(True)
        self.invalidate()

    def invalidate(self, *_: object) -> None:
        """Discard the cached background before the next frame."""
        self._background = None

    def _draw_artists(self) -> None:
        for artist in self._artists:
            if artist.get_visible():
                self.figure.draw_artist(artist)

    def _cache_background(self, event: DrawEvent) -> None:
        # Exports include animated artists in the normal draw; don't draw them twice.
        if event.canvas.is_saving():
            self.invalidate()
        else:
            self._background = self.canvas.copy_from_bbox(self.figure.bbox)
            self._draw_artists()

    def render(self) -> None:
        """Draw the visible figure, reusing its static background when possible."""
        if not self.isVisible():
            return
        if self._background is None or self.figure.stale:
            self.canvas.draw_idle()
        else:
            self.canvas.restore_region(self._background)
            self._draw_artists()
            self.canvas.blit(self.figure.bbox)

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        self.invalidate()
        self.canvas.draw_idle()
