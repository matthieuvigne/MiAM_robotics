"""Match-only state durations and control-loop timing with synchronized text."""

import numpy as np
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget

from .data import MatchData
from .figure_view import FigurePane
from .plot_view import PlotTab
from .plot_navigation import PlotNavigation
from .state_timeline import StateTimeline
from .states import MatchStates, STATE_STYLES, UNRECORDED_COLOR
from .text_view import TextLogModel
from .timeline import TimeController


def build_timing_tab(data: MatchData, states: MatchStates, controller: TimeController,
                     model: TextLogModel, parent: QWidget) -> PlotTab:
    """Build the state timeline, duration charts and loop timing with foldable text."""
    content = QWidget()
    pane = FigurePane(content)
    layout = QVBoxLayout(content)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.addWidget(QLabel("State timeline — wheel: zoom · Shift-wheel: pan · double-click: reset"))
    layout.addWidget(StateTimeline(data, states, controller, content, zoomable=True))
    layout.addWidget(pane)
    pane.canvas.setMinimumSize(700, 500)
    figure = pane.figure
    grid = figure.add_gridspec(2, 2)
    tables = grid[1, 0].subgridspec(2, 1)
    duration = states.end - states.start
    figure.suptitle(f"Match: {states.start:.3f}–{states.end:.3f} s  |  Duration: {duration:.3f} s")
    for index, (name, (title, styles)) in enumerate(STATE_STYLES.items()):
        rows = [(label, states.durations[name].get(state, 0.0), color)
                for state, (label, color) in styles.items()
                if states.durations[name].get(state, 0.0) > 0]
        unrecorded = duration - sum(seconds for _, seconds, _ in rows)
        if unrecorded > 1e-7:
            rows.append(("Not recorded", unrecorded, UNRECORDED_COLOR))
        ax = figure.add_subplot(grid[0, index])
        ax.barh([label for label, _, _ in rows], [seconds for _, seconds, _ in rows],
                color=[color for _, _, color in rows])
        ax.invert_yaxis()
        ax.set_title(title)
        ax.set_xlabel("Time in state (s)")
        ax.grid(axis="x", alpha=0.3)
        table_ax = figure.add_subplot(tables[index])
        table_ax.set_axis_off()
        table = table_ax.table(
            cellText=[[label, f"{seconds:.3f}"] for label, seconds, _ in rows],
            colLabels=[title + " state", "Duration (s)"],
            colWidths=[0.75, 0.25], bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        for (row, column), cell in table.get_celld().items():
            cell.set_edgecolor("#d0d7de")
            cell.set_linewidth(0.5)
            if row == 0:
                cell.set_facecolor("#dce6f0")
                cell.get_text().set_weight("bold")
            else:
                cell.set_facecolor("#f5f7fa" if row % 2 == 0 else "white")
    increment_axes = figure.add_subplot(grid[1, 1])
    times, values = data.series["timeIncrement"].plot_data()
    inside_match = (times >= states.start) & (times <= states.end)
    milliseconds = values[inside_match] * 1000
    increment_axes.plot(times[inside_match], milliseconds, linewidth=1)
    increment_axes.set_title("Control-loop period (ms)")
    increment_axes.set_xlabel("Time (s)")
    increment_axes.set_ylabel("ms")
    increment_axes.set_xlim(states.start, states.end)
    increment_axes.set_ylim(0, float(np.nanmax(milliseconds)) * 1.1)
    increment_axes.grid(alpha=0.3)
    cursor = increment_axes.axvline(controller.time, color="crimson", linewidth=1.3)
    pane.watch_axes(increment_axes)
    PlotNavigation(content, pane, [increment_axes], [cursor], controller)
    return PlotTab(content, model, controller, parent)
