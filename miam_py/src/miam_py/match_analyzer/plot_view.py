"""Time plots and their tab-owned, foldable text panel."""

from typing import Optional

import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QSplitter, QToolButton, QVBoxLayout, QWidget

from ..utils import plot_layout
from .data import MatchData
from .figure_view import FigurePane
from .text_view import TextLogModel, TextLogPanel
from .timeline import TimeController
from .plot_navigation import PlotNavigation


PlotSeries = tuple[str, np.ndarray, np.ndarray]
Plot = tuple[str, list[PlotSeries]]


def channel_plots(data: MatchData, groups: list[list[str]]) -> list[Plot]:
    """Build plot definitions from resolved channel groups."""
    return [(" / ".join(name.rsplit(".", 1)[-1] for name in group),
             [(name, *data.series[name].plot_data()) for name in group])
            for group in groups]


class PlotView(QWidget):
    """Time plots with shared-time cursors and mouse seeking."""

    def __init__(
        self,
        data: MatchData,
        controller: TimeController,
        plots: list[Plot],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        pane = FigurePane(self, scrollable=True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(pane)
        axes = []
        cursors = []
        figure = pane.figure
        rows, columns = plot_layout(len(plots))
        limits = (data.minimum, data.maximum)
        pane.canvas.setMinimumWidth(400)
        pane.canvas.setMinimumHeight(rows * 180)
        for i, (title, series) in enumerate(plots):
            ax = figure.add_subplot(rows, columns, i + 1,
                                    sharex=axes[0] if axes else None)
            for label, times, values in series:
                ax.plot(times, values, label=label, linewidth=1)
            ax.set_title(title)
            ax.set_xlabel("Time (s)")
            ax.grid(alpha=0.3)
            ax.legend(fontsize="small")
            if limits[1] > limits[0]:
                ax.set_xlim(*limits)
            axes.append(ax)
            cursors.append(ax.axvline(controller.time, color="crimson", linewidth=1.3))
            pane.watch_axes(ax)
        PlotNavigation(self, pane, axes, cursors, controller)


class PlotTab(QSplitter):
    """A plot and synchronized text panel whose splitter owns folding."""

    def __init__(
        self,
        view: QWidget,
        model: TextLogModel,
        controller: TimeController,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(Qt.Vertical, parent)
        self._expanded_height = 220
        self.addWidget(view)
        self._text_area = QWidget()
        layout = QVBoxLayout(self._text_area)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self._toggle = QToolButton()
        self._toggle.setText("Text log")
        self._toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self._toggle.setCheckable(True)
        self._toggle.setToolTip("Expand or fold the synchronized text log")
        layout.addWidget(self._toggle)
        self._text_panel = TextLogPanel(model, controller, self._text_area)
        layout.addWidget(self._text_panel, 1)
        self.addWidget(self._text_area)
        self.setCollapsible(1, False)
        self.setStretchFactor(0, 3)
        self.setStretchFactor(1, 1)
        self._toggle.toggled.connect(self._set_expanded)
        self._set_expanded(False)

    def _set_expanded(self, expanded: bool) -> None:
        if not expanded and self._text_panel.isVisible():
            self._expanded_height = self.sizes()[1]
        self._toggle.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self._text_panel.setVisible(expanded)
        folded_height = self._toggle.sizeHint().height()
        self._text_area.setMaximumHeight(16777215 if expanded else folded_height)
        height = self._expanded_height if expanded else folded_height
        self.setSizes([max(0, self.height() - self.handleWidth() - height), height])
