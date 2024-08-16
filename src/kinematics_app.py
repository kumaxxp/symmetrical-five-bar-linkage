import tkinter as tk
from gui import GUI
from kinematics.hip import Hip
from visualization import Visualization

class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Kinematics Visualization')
        self.geometry('1600x1000')
        self.resizable(False, True)

        self.gui = GUI(self)
        self.hip = Hip()
        self.visualization = Visualization(self.gui.canvas)

        self.setup_kinematics()
        self.set_initial_angles()
        self.initialize_kinematics()
        
        self.after(100, self.update_plot)

    def setup_kinematics(self):
        B1 = (100, -100 + 200)
        B2 = (-100, -100 + 200)
        self.hip.setup(B1, B2)

        self.initial_angles = {
            'left': {'theta1': -50, 'theta2': -120, 'thetaF': -60},
            'right': {'theta1': -50, 'theta2': -120, 'thetaF': -60}
        }

        for leg in ['left', 'right']:
            self.hip.set_leg_angles(leg, **self.initial_angles[leg])

    def set_initial_angles(self):
        for leg in ['left', 'right']:
            for angle, value in self.initial_angles[leg].items():
                self.gui.set(leg, angle, value)

    def initialize_kinematics(self):
        for leg in ['left', 'right']:
            initial_angles = {
                'theta1': self.gui.get_slider_value(leg, 'theta1'),
                'theta2': self.gui.get_slider_value(leg, 'theta2'),
                'thetaF': self.gui.get_slider_value(leg, 'thetaF')
            }
            self.hip.set_leg_angles(leg, **initial_angles)
        self.hip.compute_forward_kinematics()

    def update_plot(self):
        for leg in ['left', 'right']:
            current_angles = {
                'theta1': self.gui.get_slider_value(leg, 'theta1'),
                'theta2': self.gui.get_slider_value(leg, 'theta2'),
                'thetaF': self.gui.get_slider_value(leg, 'thetaF')
            }
            self.hip.set_leg_angles(leg, **current_angles)

        self.hip.compute_forward_kinematics()
        self.visualization.draw_transformed_kinematics(self.hip)
        self.gui.canvas.update()
        self.after(100, self.update_plot)
