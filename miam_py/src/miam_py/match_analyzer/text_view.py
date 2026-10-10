"""Reusable searchable text panels backed by one shared model."""

from typing import Optional

from PyQt5.QtCore import QAbstractTableModel, QModelIndex, QObject, QSortFilterProxyModel, Qt, QTimer
from PyQt5.QtGui import QShowEvent
from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QHeaderView, QHBoxLayout, QLabel,
    QLineEdit, QTableView, QVBoxLayout, QWidget,
)

from .data import MatchData
from .timeline import TimeController


class TextLogModel(QAbstractTableModel):
    """A shared, read-only table of timestamped log messages."""

    def __init__(self, data: MatchData, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self.entries = data.entries

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        """Return the number of log messages for the root table."""
        return 0 if parent.isValid() else len(self.entries)

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        """Return the two table columns: time and message."""
        return 0 if parent.isValid() else 2

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> Optional[str]:
        """Return the displayed text or original message tooltip."""
        if not index.isValid():
            return None
        entry = self.entries[index.row()]
        if role == Qt.DisplayRole:
            return f"{entry.time:.6f}" if index.column() == 0 else entry.message
        if role == Qt.ToolTipRole:
            return entry.raw
        return None

    def headerData(self, section: int, orientation: Qt.Orientation, role: int = Qt.DisplayRole) -> Optional[str]:
        """Return the time or message column heading."""
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return ("Time (s)", "Message")[section]
        return None


class TextLogPanel(QWidget):
    """Searchable log messages synchronized with the shared timeline."""

    def __init__(self, model: TextLogModel, controller: TimeController, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._controller = controller
        self._proxy = QSortFilterProxyModel(self)
        self._proxy.setSourceModel(model)
        self._proxy.setFilterKeyColumn(-1)
        self._proxy.setFilterCaseSensitivity(Qt.CaseInsensitive)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        controls = QHBoxLayout()
        search = QLineEdit()
        search.setPlaceholderText("Search messages…")
        search.setClearButtonEnabled(True)
        self._follow = QCheckBox("Follow time")
        self._follow.setToolTip("Scroll to the highlighted message when time changes. "
                               "Turn off to keep your reading position; time synchronization stays active.")
        self._follow.setChecked(True)
        self._status = QLabel()
        controls.addWidget(search, 1)
        controls.addWidget(self._follow)
        controls.addWidget(self._status)
        layout.addLayout(controls)
        self._table = QTableView()
        self._table.setModel(self._proxy)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setWordWrap(False)
        self._table.verticalHeader().hide()
        self._table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        layout.addWidget(self._table)
        self._table.clicked.connect(self._seek)
        self._table.activated.connect(self._seek)
        search.textChanged.connect(self._filter_changed)
        self._follow.toggled.connect(self._refresh)
        controller.changed.connect(self._update_time)

    def _seek(self, index: QModelIndex) -> None:
        row = self._proxy.mapToSource(index).row()
        entry = self._proxy.sourceModel().entries[row]
        self._controller.set_time(entry.time, entry=row)

    def _filter_changed(self, text: str) -> None:
        self._proxy.setFilterFixedString(text)
        self._refresh()

    def _refresh(self, *_: object) -> None:
        self._update_time(self._controller.time, self._controller.entry)

    def _update_time(self, _time: float, row: Optional[int]) -> None:
        if not self.isVisible():
            return
        index = (self._proxy.mapFromSource(self._proxy.sourceModel().index(row, 0))
                 if row is not None else QModelIndex())
        selection = self._table.selectionModel()
        if not index.isValid():
            selection.clear()
            self._status.setText("No messages" if row is None
                                 else "Selected message filtered out")
            return
        self._status.setText("")
        changed = (self._table.currentIndex().row() != index.row()
                   or not selection.isRowSelected(index.row(), QModelIndex()))
        if changed:
            vertical = self._table.verticalScrollBar().value()
            horizontal = self._table.horizontalScrollBar().value()
            selection.setCurrentIndex(index, selection.ClearAndSelect | selection.Rows)
            if not self._follow.isChecked():
                # Changing the current index can implicitly scroll QTableView.
                self._table.verticalScrollBar().setValue(vertical)
                self._table.horizontalScrollBar().setValue(horizontal)
        if self._follow.isChecked() and (changed or not self._table.visualRect(index).intersects(
                self._table.viewport().rect())):
            self._table.scrollTo(index, QAbstractItemView.PositionAtCenter)

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        # Wait for the enclosing splitter to assign the visible panel's geometry.
        QTimer.singleShot(0, self._refresh)
