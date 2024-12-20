import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from dji_mavic_io import read_flightdata_csv
from matplotlib import patches as mpl_patches
from matplotlib import lines as mpl_lines

# Remote Control Display Code (simplified version)

class RemoteControlDisplay:
    def __init__(self, flightdata_df):
        # Initialize remote control values from flight data
        self.rc_climb = np.array(flightdata_df['rc_throttle'].to_list(), dtype=np.float64)
        self.rc_yaw = np.array(flightdata_df['rc_rudder'].to_list(), dtype=np.float64)
        self.rc_pitch = np.array(flightdata_df['rc_elevator'].to_list(), dtype=np.float64)
        self.rc_roll = np.array(flightdata_df['rc_aileron'].to_list(), dtype=np.float64)
        
        # Matplotlib figure setup
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvas(self.fig)

        self.ax.set_xlim(-120, 120)
        self.ax.set_ylim(-120, 120)

        # Add the stick and bars to the plot
        self.stick = mpl_lines.Line2D([0, 0], [0, 0], linewidth=2, color='blue', animated=True)
        self.ax.add_line(self.stick)
        
        self.bar_x = mpl_lines.Line2D([0, 0], [0, 0], linewidth=5, color='orange', solid_capstyle='butt', animated=True)
        self.bar_y = mpl_lines.Line2D([0, 0], [0, 0], linewidth=5, color='orange', solid_capstyle='butt', animated=True)
        self.ax.add_line(self.bar_x)
        self.ax.add_line(self.bar_y)

    def update(self, index):
        val1 = self.rc_yaw[index]
        val2 = self.rc_climb[index]
        self.stick.set_data([0, val1], [0, val2])
        self.bar_x.set_data([0, val1], [0, 0])
        self.bar_y.set_data([0, 0], [0, val2])
        self.canvas.draw()

    def get_canvas(self):
        return self.canvas


class RemoteControlWindow(QMainWindow):
    def __init__(self, flightdata_df):
        super().__init__()

        self.setWindowTitle('Remote Control Display')
        self.setGeometry(100, 100, 800, 600)

        # Create a layout and widget to hold the canvas
        layout = QVBoxLayout()

        # Initialize RemoteControlDisplay
        self.rc_display = RemoteControlDisplay(flightdata_df)

        # Get the Matplotlib canvas and add it to the layout
        layout.addWidget(self.rc_display.get_canvas())

        # Set up the main widget and layout
        main_widget = QWidget(self)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

    def update_display(self, index):
        # Update remote control display with new index
        self.rc_display.update(index)


if __name__ == '__main__':
    # Load flight data
    flightdata_df = read_flightdata_csv('dji_mavic_test_data.csv')

    app = QApplication(sys.argv)

    # Create RemoteControlWindow
    window = RemoteControlWindow(flightdata_df)
    window.show()

    # Update display periodically or based on user input
    samplerate = 1  # Adjust this for update frequency
    for i in range(0, len(flightdata_df), samplerate):
        window.update_display(i)

    sys.exit(app.exec_())
