import tkinter as tk
from tkinter import ttk
import numpy as np
from extended_kinematics import ExtendedKinematics
from transformation import Transformation2D

class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Kinematics Visualization')
        self.geometry('1400x1000')
        self.resizable(False, True)

        self.setup_ui()
        self.setup_kinematics()
        self.last_angles = {'left': {}, 'right': {}}
        self.update_plot()

    def setup_ui(self):
        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        self.graph_frame = ttk.Frame(self.main_frame)
        self.graph_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas1 = tk.Canvas(self.graph_frame, width=600, height=600)
        self.canvas1.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas2 = tk.Canvas(self.graph_frame, width=600, height=600)
        self.canvas2.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.slider_frame = ttk.Frame(self.main_frame)
        self.slider_frame.pack(side=tk.RIGHT, fill=tk.Y)

        self.iders = {}
        for leg in ['left', 'right']:
            leg_frame = ttk.LabelFrame(self.slider_frame, text=f"{leg.capitalize()} Leg")
            leg_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
            self.sliders[leg] = {}
            for angle in ['theta1', 'theta2', 'thetaF']:
                self.sliders[leg][angle] = self.create_slider_with_label(
                    f"{leg.capitalize()} {angle}", 
                    -45 if angle == 'theta1' else -135 if angle == 'theta2' else -50, 
                    leg_frame
                )

    def create_slider_with_label(self, label_text, initial_value, parent_frame):
        frame = ttk.Frame(parent_frame)
        frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        label = ttk.Label(frame, text=label_text)
        label.pack(side=tk.LEFT)

        value_label = ttk.Label(frame, text=str(initial_value))
        value_label.pack(side=tk.RIGHT)

        slider = ttk.Scale(frame, from_=-180, to=180, orient=tk.HORIZONTAL, 
                           command=lambda v: self.update_slider_value(v, value_label),
                           length=200)
        slider.set(initial_value)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        return slider

    def update_slider_value(self, value, value_label):
        value_label.config(text=str(int(float(value))))
        self.update_plot()

    def setup_kinematics(self):
        self.ek_left = ExtendedKinematics(100, 200, 200, 400, 200, 150)
        self.ek_right = ExtendedKinematics(100, 200, 200, 400, 200, 150)

    def update_plot(self):
        for leg, ek in [('left', self.ek_left), ('right', self.ek_right)]:
            current_angles = {
                'theta1': self.sliders[leg]['theta1'].get(),
                'theta2': self.sliders[leg]['theta2'].get(),
                'thetaF': self.sliders[leg]['thetaF'].get()
            }

            if current_angles != self.last_angles.get(leg, {}):
                ek.set_angles(**current_angles)
                ek.compute_forward_kinematics()
                self.last_angles[leg] = current_angles.copy()

        self.draw_extended_kinematics()
        self.draw_transformed_kinematics()

    def draw_extended_kinematics(self):
        self.canvas1.delete("all")
        self.draw_kinematics(self.canvas1, False)

    def draw_transformed_kinematics(self):
        self.canvas2.delete("all")
        self.draw_kinematics(self.canvas2, True)

    def draw_kinematics(self, canvas, transform):
        canvas_width = canvas.winfo_width()
        canvas_height = canvas.winfo_height()
        scale = min(canvas_width / 1200, canvas_height / 1200)
        offset_x = canvas_width / 2
        offset_y = canvas_height / 2 if not transform else canvas_height * 0.8

        for leg, ek in [('left', self.ek_left), ('right', self.ek_right)]:
            points = ek.format_result()
            if transform:
                angle_flower = self.angle_between_vectors((0, 0), (1, 0), points['E'], points['F'])
                transformer = Transformation2D(origin=points['E'], angle=-angle_flower)
                points = {key: transformer.transform(value) for key, value in points.items()}

            for start, end, color in [('B1', 'M1', 'red'), ('M1', 'X', 'blue'), ('X', 'M2', 'green'),
                                      ('M2', 'B2', 'yellow'), ('X', 'E', 'magenta'), ('E', 'F', 'cyan')]:
                if points[start] is not None and points[end] is not None:
                    x1, y1 = self.transform_point(points[start], scale, offset_x, offset_y)
                    x2, y2 = self.transform_point(points[end], scale, offset_x, offset_y)
                    canvas.create_line(x1, y1, x2, y2, fill=color, width=2)

            for point, color in zip(['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F'],
                                    ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan']):
                if points[point] is not None:
                    x, y = self.transform_point(points[point], scale, offset_x, offset_y)
                    canvas.create_oval(x-3, y-3, x+3, y+3, fill=color)
                    canvas.create_text(x+10, y+10, text=f'{leg} {point}', anchor='sw')

    def transform_point(self, point, scale, offset_x, offset_y):
        return point[0] * scale + offset_x, -point[1] * scale + offset_y

    @staticmethod
    def angle_between_vectors(v1_start, v1_end, v2_start, v2_end):
        v1 = np.array(v1_end) - np.array(v1_start)
        v2 = np.array(v2_end) - np.array(v2_start)
        angle_rad = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
        angle_deg = np.degrees(angle_rad)
        return angle_deg if -180 <= angle_deg <= 180 else angle_deg - 360 * np.sign(angle_deg)

if __name__ == '__main__':
    app = KinematicsApp()
    app.mainloop()
