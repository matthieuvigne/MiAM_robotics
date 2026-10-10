"""Application composition, time controls, and custom-tab lifecycle."""

from typing import Iterable, Optional, Sequence

from PyQt5.QtCore import QEvent, QObject, Qt
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QHBoxLayout,
    QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow, QPushButton,
    QStyle, QTabBar, QTabWidget, QTextEdit, QToolButton, QVBoxLayout, QWidget,
)

from ..utils import expand_plot_specs
from .data import MatchData
from .map_view import MapView
from .plot_view import Plot, PlotTab, PlotView, channel_plots
from .text_view import TextLogModel, TextLogPanel
from .timeline import REPLAY_SPEEDS, TimeController
from .default_plot_tabs import configure_default_plots
from .states import MatchStates
from .state_timeline import StateTimeline
from .timing_view import build_timing_tab


class PlotDialog(QDialog):
    """Choose a custom plot title and resolve its channel expression."""

    def __init__(self, variables: Iterable[str], parent: QWidget) -> None:
        super().__init__(parent)
        self.setWindowTitle("Create plot tab")
        self.resize(760, 600)
        self._variables = sorted(variables)
        self._groups = []
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Tab name"))
        self._name = QLineEdit("Custom plots")
        layout.addWidget(self._name)
        help_text = QLabel("Separate subplots with spaces; join channels with colons. "
                           "Wildcards: *, ?, [abc]. Double-click a channel to append it.")
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        self._expression = QLineEdit()
        self._expression.setPlaceholderText("MotionController.currentPositionX:MotionController.targetPositionX")
        layout.addWidget(self._expression)
        search = QLineEdit()
        search.setPlaceholderText("Filter available channels…")
        layout.addWidget(search)
        self._channels = QListWidget()
        layout.addWidget(self._channels, 1)
        self._preview = QTextEdit()
        self._preview.setReadOnly(True)
        layout.addWidget(self._preview, 1)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self._create_button = buttons.button(QDialogButtonBox.Ok)
        self._create_button.setText("Create tab")
        layout.addWidget(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self._expression.textChanged.connect(self._update_preview)
        search.textChanged.connect(self._filter_channels)
        self._channels.itemDoubleClicked.connect(self._append_channel)
        self._filter_channels("")
        self._update_preview()

    def selected_plot(self) -> tuple[str, list[list[str]]]:
        """Return the accepted tab title and resolved channel groups."""
        return self._name.text().strip() or "Custom plots", self._groups

    def _filter_channels(self, text: str) -> None:
        self._channels.clear()
        self._channels.addItems([name for name in self._variables if text.lower() in name.lower()])

    def _append_channel(self, item: QListWidgetItem) -> None:
        self._expression.setText((self._expression.text() + " " + item.text()).strip())

    def _update_preview(self, *_: object) -> None:
        try:
            self._groups = expand_plot_specs(self._expression.text(), self._variables)
            self._preview.setPlainText("\n".join(f"Plot {i + 1}: " + " + ".join(group)
                                                 for i, group in enumerate(self._groups)))
            self._create_button.setEnabled(True)
        except ValueError as error:
            self._groups = []
            self._preview.setPlainText(str(error))
            self._create_button.setEnabled(False)


class MatchWindow(QMainWindow):
    """Compose complete match tabs around shared navigation and text data."""

    def __init__(self, data: MatchData, initial_plots: Sequence[list[list[str]]] = ()) -> None:
        super().__init__()
        self._data = data
        self.setWindowTitle("MiAM match analysis — " + data.filename)
        self.resize(1400, 950)
        self._controller = TimeController(data, self)
        self._text_model = TextLogModel(data, self)
        self._states = MatchStates(data)
        central = QWidget()
        layout = QVBoxLayout(central)
        self.setCentralWidget(central)
        self._time_bar = self._build_time_bar()
        layout.addWidget(self._time_bar)
        layout.addWidget(self._build_tabs(initial_plots), 1)
        QApplication.instance().installEventFilter(self)

    def _build_tabs(self, initial_plots: Sequence[list[list[str]]]) -> QTabWidget:
        self._tabs = QTabWidget()
        add = QPushButton("+")
        add.setToolTip("Create a custom plot tab")
        add.clicked.connect(self._create_plot_tab)
        self._tabs.setCornerWidget(add, Qt.TopRightCorner)
        self._tabs.addTab(MapView(self._data, self._controller, self._text_model), "Map")
        for title, plots in configure_default_plots(self._data).items():
            self._add_plot_tab(title, plots)
        self._tabs.addTab(build_timing_tab(self._data, self._states, self._controller, self._text_model, self._tabs), "Timing")
        self._tabs.addTab(TextLogPanel(self._text_model, self._controller), "Text")
        for groups in initial_plots:
            self._add_custom_plot("Custom plots", groups)
        return self._tabs

    def _build_time_bar(self) -> QWidget:
        bar = QWidget()
        controls = QHBoxLayout(bar)
        controls.setContentsMargins(0, 0, 0, 0)
        controls.addWidget(QLabel("Time (s)"))
        self._time_input = QDoubleSpinBox()
        self._time_input.setDecimals(3)
        self._time_input.setRange(self._data.minimum, self._data.maximum)
        self._time_input.setSingleStep(0.005)
        self._time_input.setKeyboardTracking(False)
        self._time_input.valueChanged.connect(self._controller.set_time)
        controls.addWidget(self._time_input)
        slider = StateTimeline(self._data, self._states, self._controller, bar)
        controls.addWidget(slider, 1)
        controls.addWidget(QLabel(f"{self._data.maximum:.3f} s"))
        controls.addWidget(QLabel("Replay speed"))
        self._replay_speed = QComboBox()
        for speed in REPLAY_SPEEDS:
            self._replay_speed.addItem(f"{speed:g}x", speed)
        self._update_replay_speed(self._controller.speed)
        self._replay_speed.currentIndexChanged.connect(
            lambda index: self._controller.set_speed(self._replay_speed.itemData(index)))
        self._controller.speed_changed.connect(self._update_replay_speed)
        controls.addWidget(self._replay_speed)
        self._play_button = QPushButton("Play")
        self._play_button.setEnabled(self._data.maximum > self._data.minimum)
        self._play_button.clicked.connect(self._controller.toggle_playback)
        self._controller.playback_changed.connect(self._update_playback)
        controls.addWidget(self._play_button)
        self._controller.changed.connect(self._update_controls)
        self._update_controls(self._controller.time, self._controller.entry)
        return bar

    def _add_plot_tab(self, title: str, plots: list[Plot]) -> PlotTab:
        view = PlotView(self._data, self._controller, plots)
        tab = PlotTab(view, self._text_model, self._controller)
        self._tabs.addTab(tab, title)
        return tab

    def _add_custom_plot(self, title: str, groups: list[list[str]]) -> PlotTab:
        tab = self._add_plot_tab(title, channel_plots(self._data, groups))
        bar = self._tabs.tabBar()
        close = QToolButton(bar)
        close.setIcon(close.style().standardIcon(QStyle.SP_TitleBarCloseButton))
        close.setAutoRaise(True)
        close.setToolTip("Close tab")
        close.clicked.connect(lambda _checked: self._close_tab(tab))
        bar.setTabButton(self._tabs.indexOf(tab), QTabBar.RightSide, close)
        return tab

    def _update_playback(self, playing: bool) -> None:
        self._play_button.setText("Pause" if playing else "Play")

    def _update_replay_speed(self, speed: float) -> None:
        self._replay_speed.blockSignals(True)
        self._replay_speed.setCurrentIndex(self._replay_speed.findData(speed))
        self._replay_speed.blockSignals(False)

    def _update_controls(self, time: float, _entry: Optional[int]) -> None:
        self._time_input.blockSignals(True)
        self._time_input.setValue(time)
        self._time_input.blockSignals(False)

    def _create_plot_tab(self) -> None:
        dialog = PlotDialog(self._data.series, self)
        try:
            if dialog.exec_() == QDialog.Accepted:
                title, groups = dialog.selected_plot()
                tab = self._add_custom_plot(title, groups)
                self._tabs.setCurrentWidget(tab)
        finally:
            dialog.deleteLater()

    def _close_tab(self, tab: PlotTab) -> None:
        self._tabs.removeTab(self._tabs.indexOf(tab))
        tab.deleteLater()

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if not isinstance(watched, QWidget) or watched.window() is not self:
            return super().eventFilter(watched, event)
        if (event.type() in (QEvent.ShortcutOverride, QEvent.KeyPress)
                and event.key() == Qt.Key_Space and event.modifiers() == Qt.NoModifier):
            focus = QApplication.focusWidget()
            editing = isinstance(focus, QLineEdit) and not focus.isReadOnly()
            if not editing:
                if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
                    self._controller.toggle_playback()
                event.accept()
                return True
        if (event.type() in (QEvent.MouseButtonPress, QEvent.Wheel, QEvent.KeyPress)
                and (watched is self._time_bar or self._time_bar.isAncestorOf(watched))
                and not any(watched is control or control.isAncestorOf(watched)
                            for control in (self._play_button, self._replay_speed))):
            self._controller.pause()
        return super().eventFilter(watched, event)

    def closeEvent(self, event: QCloseEvent) -> None:
        self._controller.pause()
        super().closeEvent(event)
