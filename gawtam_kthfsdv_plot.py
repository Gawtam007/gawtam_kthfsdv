import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation
from PyQt5.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QWidget, QSlider, QPushButton, QLineEdit, QLabel, QHBoxLayout, QFileDialog
from PyQt5.QtCore import Qt
import csv
from datetime import datetime
import tikzplotlib

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, t_end=10, num_points=1000):
        fig, self.ax = plt.subplots()
        super().__init__(fig)
        self.setParent(parent)
        self.t = np.linspace(0, t_end, num_points)
        self.num_points = num_points
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlim(0, t_end)
        self.ax.set_ylim(0, 10)
        self.is_running = False

    def h(self, t):
        lambda_t = 5 * np.sin(2 * np.pi * t)
        return 3 * np.pi * np.exp(-lambda_t)

    def init_plot(self):
        self.line.set_data([], [])
        return self.line,

    def update_plot(self, frame):
        if self.is_running:
            t_values = self.t[:frame]
            h_values = self.h(t_values)
            self.line.set_data(t_values, h_values)
        return self.line,

    def start_animation(self):
        self.is_running = True
        self.anim = FuncAnimation(self.figure, self.update_plot, frames=len(self.t), init_func=self.init_plot, 
                                  interval=100, blit=True)
        self.draw()

    def stop_animation(self):
        """Stop the animation"""
        self.is_running = False

    def reset_animation(self):
        """Reset the plot"""
        self.stop_animation()
        self.line.set_data([], [])
        self.ax.set_xlim(0, max(self.t))
        self.ax.set_ylim(0, 10)
        self.draw()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Exercise 2")
        self.plot_canvas = PlotCanvas(self, t_end=10)
        self.init_ui()

    def init_ui(self):
        widget = QWidget()
        layout = QVBoxLayout()

        # Plot Canvas
        layout.addWidget(self.plot_canvas)

        # Sliders for adjusting axis limits
        self.x_slider = QSlider(Qt.Horizontal)
        self.x_slider.setMinimum(1)
        self.x_slider.setMaximum(20)
        self.x_slider.setValue(10)
        self.x_slider.valueChanged.connect(self.adjust_x_axis)

        self.y_slider = QSlider(Qt.Horizontal)
        self.y_slider.setMinimum(5)
        self.y_slider.setMaximum(20)
        self.y_slider.setValue(10)
        self.y_slider.valueChanged.connect(self.adjust_y_axis)

        # Buttons for control
        start_button = QPushButton('Start')
        start_button.clicked.connect(self.plot_canvas.start_animation)

        stop_button = QPushButton('Stop')
        stop_button.clicked.connect(self.plot_canvas.stop_animation)

        reset_button = QPushButton('Reset')
        reset_button.clicked.connect(self.plot_canvas.reset_animation)

        # Experiment name input
        self.experiment_name = QLineEdit()
        self.experiment_name.setPlaceholderText("Enter experiment name")

        # Save button
        save_button = QPushButton('Save Data')
        save_button.clicked.connect(self.save_data)

        # Grid toggle
        grid_button = QPushButton('Toggle Grid')
        grid_button.clicked.connect(self.toggle_grid)

        # Export TikZ button
        export_button = QPushButton('Export to TikZ')
        export_button.clicked.connect(self.export_to_tikz)

        # Layout for control buttons and input fields
        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("X-Axis Zoom"))
        control_layout.addWidget(self.x_slider)
        control_layout.addWidget(QLabel("Y-Axis Zoom"))
        control_layout.addWidget(self.y_slider)
        control_layout.addWidget(start_button)
        control_layout.addWidget(stop_button)
        control_layout.addWidget(reset_button)
        control_layout.addWidget(self.experiment_name)
        control_layout.addWidget(save_button)
        control_layout.addWidget(grid_button)
        control_layout.addWidget(export_button)  # Add the export button

        layout.addLayout(control_layout)
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def adjust_x_axis(self, value):
        """Adjust the X-axis based on slider value"""
        self.plot_canvas.ax.set_xlim(0, value)
        self.plot_canvas.draw()

    def adjust_y_axis(self, value):
        """Adjust the Y-axis based on slider value"""
        self.plot_canvas.ax.set_ylim(0, value)
        self.plot_canvas.draw()

    def toggle_grid(self):
        self.plot_canvas.ax.grid()
        self.plot_canvas.draw()

    def save_data(self):
        experiment_name = self.experiment_name.text() or "experiment"
        file_name = f"{experiment_name}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        options = QFileDialog.Options()
        save_path, _ = QFileDialog.getSaveFileName(self, "Save CSV", file_name, "CSV Files (*.csv);;All Files (*)", options=options)

        if save_path:
            t_values = self.plot_canvas.t
            h_values = self.plot_canvas.h(t_values)
            with open(save_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time (t)", "h(t)"])
                writer.writerows(zip(t_values, h_values))
            print(f"Data saved to {save_path}")

    def export_to_tikz(self):
        experiment_name = self.experiment_name.text() or "experiment"
        file_name = f"{experiment_name}_tikz_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.tex"
        options = QFileDialog.Options()
        save_path, _ = QFileDialog.getSaveFileName(self, "Export to TikZ", file_name, "TeX Files (*.tex);;All Files (*)", options=options)

        if save_path:
            tikzplotlib.save(save_path) 
            print(f"Plot exported to {save_path}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
