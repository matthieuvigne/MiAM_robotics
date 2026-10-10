"""Complete map tab: field, moving markers, readouts, and synchronized text."""

from typing import Optional

import numpy as np
from matplotlib.patches import Circle, Polygon
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QShowEvent
from PyQt5.QtWidgets import QFormLayout, QLabel, QSplitter, QVBoxLayout, QWidget

from ..resources import load_table_image
from .data import MatchData
from .figure_view import FigurePane
from .text_view import TextLogModel, TextLogPanel
from .timeline import TimeController


OBSTACLE_RADIUS_MM = 200  # Display radius used by the C++ simulation viewer.
ROBOT_SHAPE = np.array([[150, 0], [-100, 100], [-100, -100]])


class MapView(QWidget):
    """The field, robot and obstacle markers, readouts, and text log."""

    def __init__(
        self,
        data: MatchData,
        controller: TimeController,
        model: TextLogModel,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._data = data
        self._controller = controller
        self._pane = FigurePane(self)
        self._pane.canvas.setMinimumHeight(300)
        info_panel = QWidget()
        info_panel.setMinimumWidth(260)
        info = QVBoxLayout(info_panel)
        readouts = QFormLayout()
        self._time_label = QLabel()
        self._match_time_label = QLabel()
        self._match_time_label.setToolTip("Elapsed seconds since match start; zero before the match starts.")
        self._obstacle_count_label = QLabel()
        readouts.addRow("Log time", self._time_label)
        readouts.addRow("Match time", self._match_time_label)
        readouts.addRow("Displayed obstacles", self._obstacle_count_label)
        info.addLayout(readouts)
        self._pose_status = QLabel()
        info.addWidget(self._pose_status)
        info.addWidget(TextLogPanel(model, controller, info_panel), 1)
        content = QSplitter(Qt.Horizontal)
        content.addWidget(self._pane)
        content.addWidget(info_panel)
        content.setSizes([800, 550])
        content.setStretchFactor(0, 3)
        content.setStretchFactor(1, 2)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(content)

        ax = self._pane.figure.add_subplot(111)
        image = ax.imshow(load_table_image(), extent=[0, 3000, 0, 2000], origin="upper", aspect="equal")
        image.set_mouseover(False)
        self._robots = []
        for target, color in ((False, "tab:blue"), (True, "tab:orange")):
            prefix = "MotionController." + ("targetPosition" if target else "currentPosition")
            x = data.series[prefix + "X"]
            y = data.series[prefix + "Y"]
            times, values = x.plot_data()
            ax.plot(values, y.sample(times), color=color, linewidth=1.2, alpha=0.7)
            marker = Polygon(ROBOT_SHAPE, closed=True, color=color, alpha=0.5 if target else 0.8)
            marker.set_visible(False)
            ax.add_patch(marker)
            self._robots.append((target, marker))
        self._obstacle_markers = []
        for _ in data.obstacle_channels:
            marker = Circle((0, 0), radius=OBSTACLE_RADIUS_MM, facecolor="red",
                            edgecolor="none", zorder=4, visible=False)
            ax.add_patch(marker)
            self._obstacle_markers.append(marker)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(0, 3000)
        ax.set_ylim(0, 2000)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.grid(alpha=0.3)
        self._pane.watch_axes(ax)
        self._pane.animate([marker for _, marker in self._robots] + self._obstacle_markers)
        controller.changed.connect(self._update_time)

    def _update_time(self, time: float, _entry: Optional[int]) -> None:
        if not self.isVisible():
            return
        obstacles = self._data.obstacles(time)
        for index, marker in enumerate(self._obstacle_markers):
            marker.set_visible(index < len(obstacles))
            if index < len(obstacles):
                marker.center = obstacles[index]
        self._obstacle_count_label.setText(str(len(obstacles)))
        available = False
        for target, marker in self._robots:
            pose = self._data.pose(time, target=target)
            marker.set_visible(pose is not None)
            if pose is not None:
                x, y, angle = pose
                c, s = np.cos(angle), np.sin(angle)
                marker.set_xy(ROBOT_SHAPE @ np.array([[c, s], [-s, c]]) + [x, y])
                if not target:
                    available = True
        self._time_label.setText(f"{time:.3f} s")
        self._match_time_label.setText(f"{max(0.0, time - self._data.match_start_time):.3f} s")
        self._pose_status.setText("" if available else "No recorded robot pose")
        self._pane.render()

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        self._update_time(self._controller.time, self._controller.entry)
