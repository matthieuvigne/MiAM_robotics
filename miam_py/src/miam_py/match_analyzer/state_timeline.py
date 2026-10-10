"""Two state-colored rows drawn over the existing slider navigation model."""

from PyQt5.QtCore import QEvent, QRectF, Qt
from PyQt5.QtGui import QColor, QMouseEvent, QPainter, QPaintEvent, QPen, QPixmap, QResizeEvent, QWheelEvent
from PyQt5.QtWidgets import QSlider, QToolTip, QWidget

from .data import MatchData
from .timeline import TimeController
from .states import MatchStates, STATE_STYLES, UNRECORDED_COLOR


class StateTimeline(QSlider):
    """Seek through the log using aligned strategy and motion-state rows."""

    def __init__(self, data: MatchData, states: MatchStates, controller: TimeController,
                 parent: QWidget, zoomable: bool = False) -> None:
        super().__init__(Qt.Horizontal, parent)
        self._controller = controller
        self._zoomable = zoomable
        self._data = data
        self._states = states
        self._view_start = data.minimum
        self._view_end = data.maximum
        self._background = None
        self.setMouseTracking(True)
        self.setMinimumHeight(2 * self.fontMetrics().height() + 16)
        self.setMinimumWidth(300)
        if zoomable:
            self.setMinimumHeight(3 * self.fontMetrics().height() + 20)
        self.setRange(0, min(2_000_000_000, max(1, round((data.maximum - data.minimum) * 1000))))
        self.valueChanged.connect(self._select_time)
        controller.changed.connect(self._update_time)
        self._update_time(controller.time, controller.entry)

    def _plot_left(self) -> int:
        return max(self.fontMetrics().horizontalAdvance(title) for title, _ in STATE_STYLES.values()) + 12

    def _plot_width(self) -> int:
        return self.width() - self._plot_left() - 5

    def _row_height(self) -> float:
        label_height = self.fontMetrics().height() + 4 if self._zoomable else 0
        return (self.height() - label_height) / 2

    def _x(self, time: float) -> float:
        return self._plot_left() + ((time - self._view_start)
                                   / (self._view_end - self._view_start)) * self._plot_width()

    def _draw_background(self) -> QPixmap:
        ratio = self.devicePixelRatioF()
        image = QPixmap(round(self.width() * ratio), round(self.height() * ratio))
        image.setDevicePixelRatio(ratio)
        image.fill(self.palette().window().color())
        painter = QPainter(image)
        painter.setFont(self.font())
        row_height = self._row_height()
        for row, (name, (title, styles)) in enumerate(STATE_STYLES.items()):
            top = row * row_height + 2
            painter.setPen(self.palette().windowText().color())
            painter.drawText(QRectF(0, top, self._plot_left() - 6, row_height - 4),
                             Qt.AlignVCenter | Qt.AlignLeft, title)
            painter.fillRect(QRectF(self._plot_left(), top, self._plot_width(), row_height - 4),
                             QColor(UNRECORDED_COLOR))
            for start, end, state in self._states.intervals[name]:
                label, color = styles[state]
                left = self._x(max(start, self._view_start))
                right = self._x(min(end, self._view_end))
                if right <= left:
                    continue
                rect = QRectF(left, top, right - left, row_height - 4)
                shade = QColor(color)
                painter.fillRect(rect, shade)
                if rect.width() > self.fontMetrics().horizontalAdvance(label) + 8:
                    painter.setPen(Qt.black if shade.lightness() > 140 else Qt.white)
                    painter.drawText(rect.adjusted(4, 0, -4, 0), Qt.AlignVCenter | Qt.AlignLeft, label)
        painter.end()
        return image

    def paintEvent(self, event: QPaintEvent) -> None:
        if self._background is None:
            self._background = self._draw_background()
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._background)
        x = round(self._x(self._controller.time))
        painter.setPen(QPen(Qt.white, 4))
        painter.drawLine(x, 0, x, self.height())
        painter.setPen(QPen(Qt.black, 2))
        painter.drawLine(x, 0, x, self.height())
        if self._zoomable:
            painter.setPen(self.palette().windowText().color())
            top = 2 * self._row_height()
            rect = QRectF(self._plot_left(), top, self._plot_width(), self.height() - top)
            painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, f"{self._view_start:.3f} s")
            painter.drawText(rect, Qt.AlignRight | Qt.AlignVCenter, f"{self._view_end:.3f} s")

    def resizeEvent(self, event: QResizeEvent) -> None:
        self._background = None
        super().resizeEvent(event)

    def _seek(self, x: int) -> None:
        fraction = max(0.0, min(1.0, (x - self._plot_left()) / self._plot_width()))
        time = self._view_start + fraction * (self._view_end - self._view_start)
        self._controller.set_time(time)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self.setFocus(Qt.MouseFocusReason)
            self._controller.pause()
            if (self._plot_left() <= event.x() <= self._plot_left() + self._plot_width()
                    and 0 <= event.y() < 2 * self._row_height()):
                self.setSliderDown(True)
                self._seek(event.x())
            event.accept()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self.isSliderDown():
            self._seek(event.x())
            event.accept()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton and self.isSliderDown():
            self._seek(event.x())
            self.setSliderDown(False)
            event.accept()

    def event(self, event: QEvent) -> bool:
        if event.type() == QEvent.ToolTip:
            position = event.pos()
            fraction = (position.x() - self._plot_left()) / self._plot_width()
            if 0 <= fraction <= 1:
                row = min(1, int(position.y() / self._row_height()))
                name = tuple(STATE_STYLES)[row]
                time = self._view_start + fraction * (self._view_end - self._view_start)
                for start, end, state in self._states.intervals[name]:
                    if start <= time < end:
                        label, _ = STATE_STYLES[name][1][state]
                        QToolTip.showText(event.globalPos(),
                                          f"{label}\n{start:.3f}–{end:.3f} s\nDuration: {end - start:.3f} s", self)
                        return True
                QToolTip.showText(event.globalPos(), "Not recorded", self)
            else:
                QToolTip.hideText()
            return True
        return super().event(event)

    def _select_time(self, value: int) -> None:
        fraction = value / self.maximum()
        self._controller.set_time(self._data.minimum + fraction * (self._data.maximum - self._data.minimum))

    def _update_time(self, time: float, _entry: object) -> None:
        if self._zoomable and not self._view_start <= time <= self._view_end:
            width = self._view_end - self._view_start
            self._view_start = max(self._data.minimum,
                                   min(time - width / 2, self._data.maximum - width))
            self._view_end = self._view_start + width
            self._background = None
        fraction = (time - self._data.minimum) / (self._data.maximum - self._data.minimum)
        self.blockSignals(True)
        self.setValue(round(fraction * self.maximum()))
        self.blockSignals(False)
        self.update()

    def wheelEvent(self, event: QWheelEvent) -> None:
        if not self._zoomable:
            super().wheelEvent(event)
            return
        steps = event.angleDelta().y() / 120
        width = self._view_end - self._view_start
        full_width = self._data.maximum - self._data.minimum
        if event.modifiers() & Qt.ShiftModifier:
            start = self._view_start - steps * width * 0.1
        else:
            fraction = max(0.0, min(1.0, (event.pos().x() - self._plot_left()) / self._plot_width()))
            anchor = self._view_start + fraction * width
            width = min(full_width, max(0.01, width * 0.8 ** steps))
            start = anchor - fraction * width
        self._view_start = max(self._data.minimum, min(start, self._data.maximum - width))
        self._view_end = self._view_start + width
        self._background = None
        self.update()
        event.accept()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        if self._zoomable and event.button() == Qt.LeftButton:
            self._view_start = self._data.minimum
            self._view_end = self._data.maximum
            self._background = None
            self.update()
            event.accept()
