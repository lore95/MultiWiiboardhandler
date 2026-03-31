import os
import signal

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from matplotlib.gridspec import GridSpec

from controllers.wiiboard_serial_controller import WiiBoardController

PLOT_HISTORY = 300


class WiiBoardView:
    def __init__(self, controller: WiiBoardController):
        self.controller = controller

        self.fig = None
        self.center_ax = None
        self.center_line = None
        self.record_btn = None
        self.record_btn_ax = None
        self.stop_all_btn = None
        self.stop_all_btn_ax = None
        self.device_axes = []
        self.device_lines = []
        self.finalized = False

    def build(self):
        self.fig = plt.figure(figsize=(14, 9))
        gs = GridSpec(3, 3, figure=self.fig, width_ratios=[1, 1.4, 1], height_ratios=[1, 1.2, 0.4])

        self.center_ax = self.fig.add_subplot(gs[0:2, 1])
        (self.center_line,) = self.center_ax.plot([], [], label="Sum of Total Forces")
        self.center_ax.set_title("TOTAL FORCE — ALL DEVICES (SUM)")
        self.center_ax.set_xlabel("Sample Index (aligned min length)")
        self.center_ax.set_ylabel("Force (N)")
        self.center_ax.grid(True)
        self.center_ax.legend(loc="upper right")

        corner_slots = [(0, 0), (0, 2), (2, 0), (2, 2)]

        for i, dev in enumerate(self.controller.devices):
            if i >= len(corner_slots):
                break
            r, c = corner_slots[i]
            ax = self.fig.add_subplot(gs[r, c])
            (line,) = ax.plot([], [], label=f"{dev.board_name} Total Force")
            ax.set_title(f"{dev.board_name} — Total Force")
            ax.set_xlabel("Host Time (s)")
            ax.set_ylabel("Force (N)")
            ax.grid(True)
            ax.legend(loc="upper right")

            self.device_axes.append(ax)
            self.device_lines.append(line)

        self.status_text = self.fig.text(
            0.5, 0.14, "", 
            ha="center", va="center",
            fontsize=11, color="green"
        )

        self.record_btn_ax = self.fig.add_axes([0.18, 0.02, 0.22, 0.06])
        self.record_btn = Button(self.record_btn_ax, "Start Recording")
        self.record_btn.on_clicked(self._on_toggle_recording)

        self.stop_all_btn_ax = self.fig.add_axes([0.60, 0.02, 0.20, 0.06])
        self.stop_all_btn = Button(self.stop_all_btn_ax, "Stop All")
        self.stop_all_btn.on_clicked(self._on_stop_all)

        # Add fields to input session name
        self.textbox_ax = self.fig.add_axes([0.18, 0.08, 0.4, 0.05])
        self.textbox = TextBox(self.textbox_ax, "Enter session name: ")

        self.save_text_btn_ax = self.fig.add_axes([0.60, 0.08, 0.20, 0.05])
        self.save_text_btn = Button(self.save_text_btn_ax, "Save session name")
        self.save_text_btn.on_clicked(self._on_save_session_name)

        self.fig.canvas.mpl_connect(
            "key_press_event",
            lambda e: self.finalize() if e.key == "escape" else None
        )
        self.fig.canvas.mpl_connect("close_event", lambda _evt: self.finalize())

    def _on_toggle_recording(self, _evt=None):
        if not self.controller.is_recording:
            self.controller.start_recording()
            self.record_btn.label.set_text("Stop Recording")
            self._lock_session_name()
        else:
            self.controller.stop_recording()
            self.record_btn.label.set_text("Start Recording")
            self._unlock_session_name()
        self.fig.canvas.draw_idle()

    def _on_stop_all(self, _evt=None):
        self.finalize()

    def _on_save_session_name(self, event):
        if self.controller.is_recording:
            return
        
        text = self.textbox.text.strip()
        self.controller.session_name = text
        self.status_text.set_text(f"Session name set to: '{text}'")
        self.fig.canvas.draw_idle()

    def _lock_session_name(self):
        self.textbox.set_active(False)
        self.textbox.ax.set_alpha(0.5)

        self.save_text_btn.label.set_text("No renaming while recording")
        self.save_text_btn.ax.set_alpha(0.5)

        self.fig.canvas.draw_idle()

    def _unlock_session_name(self):
        self.textbox.set_active(True)
        self.textbox.ax.set_alpha(1.0)

        self.save_text_btn.label.set_text("Save session name")
        self.save_text_btn.ax.set_alpha(1.0)

        self.fig.canvas.draw_idle()

    def update(self, _frame):
        for i, dev in enumerate(self.controller.devices):
            x, y = dev.get_plot_data(PLOT_HISTORY)
            if x and y:
                self.device_lines[i].set_data(x, y)
                self.device_axes[i].relim()
                self.device_axes[i].autoscale_view()

        series_list = [dev.get_total_series() for dev in self.controller.devices if dev.get_total_series()]
        if series_list:
            min_len = min(len(s) for s in series_list)
            if min_len >= 2:
                summed = [float(np.sum([s[i] for s in series_list])) for i in range(min_len)]
                x = list(range(min_len))
                self.center_line.set_data(x[-PLOT_HISTORY:], summed[-PLOT_HISTORY:])
                self.center_ax.relim()
                self.center_ax.autoscale_view()

        return ()

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True

        if self.controller.is_recording:
            self.controller.stop_recording()

        self.controller.close_all()

        try:
            plt.close(self.fig)
        except Exception:
            pass
        try:
            plt.close("all")
        except Exception:
            pass


def register_signal_handlers(view: WiiBoardView):
    def _handle_term(_signum, _frame):
        try:
            view.finalize()
        finally:
            os._exit(0)

    signal.signal(getattr(signal, "SIGTERM", signal.SIGINT), _handle_term)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _handle_term)