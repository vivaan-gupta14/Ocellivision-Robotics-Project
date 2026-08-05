
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import time
import matplotlib.pyplot as plt
print("Telemetry Imports Complete\n")

#-----------------------------------------------------------TELEMETRY CLASS---------------------------------------------------------
# Live current-vs-time plot with vertical event markers (phase boundaries). Data lives only
# in memory/on-screen for the run - no CSV is written. Redraws happen incrementally as
# sample()/markEvent() are called from the polling loop, rather than being plotted once at
# the end like torque_characterization_test.py's plot_telemetry().

CURRENT_COLOR = "#eb6834"   # categorical slot 2 (orange) - matches torque_characterization_test.py
EVENT_COLOR   = "#52514e"   # muted ink, text/lines only - never a series color


class Telemetry:

    def __init__(self, title, ylabel="Current (mA)", xlabel="Elapsed Time (s)", thresholdMa=None):
        self.times    = []
        self.currents = []
        self.events   = []   # (elapsedSeconds, label)
        self.t0       = time.time()

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(11, 5))
        self.line, = self.ax.plot([], [], color=CURRENT_COLOR, linewidth=1.5)
        self.ax.set_title(title, pad=14)
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        self.ax.axhline(0, color=EVENT_COLOR, linewidth=0.75, alpha=0.4)

        # Dotted reference lines at +/- the obstacle-detection threshold, so a spike is
        # visually obvious against the live trace as soon as it crosses them.
        if thresholdMa is not None:
            self.ax.axhline(thresholdMa, color=CURRENT_COLOR, linewidth=1, linestyle=":", alpha=0.5)
            self.ax.axhline(-thresholdMa, color=CURRENT_COLOR, linewidth=1, linestyle=":", alpha=0.5)

        self.fig.tight_layout()
        plt.show(block=False)
        plt.pause(0.001)

    #-----------------------------------------------------SAMPLING-----------------------------------------------------------

    def sample(self, currentMa):
        elapsed = time.time() - self.t0
        self.times.append(elapsed)
        self.currents.append(currentMa)
        self._redraw()
        return elapsed

    #-----------------------------------------------------EVENT MARKERS-----------------------------------------------------------
    # Vertical dashed line + rotated label at "now" - same visual style as
    # torque_characterization_test.py's event markers, but drawn live as each phase begins.

    def markEvent(self, label):
        elapsed = time.time() - self.t0
        self.events.append((elapsed, label))
        self.ax.axvline(elapsed, color=EVENT_COLOR, linewidth=1, linestyle="--", alpha=0.6)
        self.ax.annotate(label, xy=(elapsed, 0.98), xycoords=("data", "axes fraction"),
                          xytext=(3, -3), textcoords="offset points",
                          fontsize=8, color=EVENT_COLOR, rotation=90, va="top", ha="left")
        self._redraw()
        return elapsed

    #-----------------------------------------------------REDRAW-----------------------------------------------------------

    def _redraw(self):
        self.line.set_data(self.times, self.currents)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw_idle()
        plt.pause(0.001)   # pumps the GUI event loop so the window actually repaints live

    #-----------------------------------------------------SHUTDOWN-----------------------------------------------------------

    def savefig(self, path):
        self.fig.savefig(path, dpi=150)

    def freeze(self):
        """Stops interactive mode. Call once the run is complete, before the final blocking show()."""
        plt.ioff()

    def show(self):
        plt.show()
