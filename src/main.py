import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from extended_kinematics import ExtendedKinematics

class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title('Kinematics Visualization')
        self.geometry('800x600')

        # メインフレームの作成
        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # Matplotlib Figureの作成
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.figure, self.main_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # スライダフレームの作成
        self.slider_frame = ttk.Frame(self.main_frame)
        self.slider_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # スライダの作成
        self.slider_theta1 = ttk.Scale(self.slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=self.update_plot)
        self.slider_theta1.set(-45)
        self.slider_theta1.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.slider_theta2 = ttk.Scale(self.slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=self.update_plot)
        self.slider_theta2.set(-135)
        self.slider_theta2.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.slider_thetaF = ttk.Scale(self.slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=self.update_plot)
        self.slider_thetaF.set(-60)
        self.slider_thetaF.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.ek = self.initialize_kinematics()
        self.update_plot()

    def initialize_kinematics(self):
        # 順運動学の初期化
        Yb = 100
        l = 200
        b = 200
        m = 400
        e = 200
        f = 150
        ek = ExtendedKinematics(Yb, l, b, m, e, f)
        ek.set_angles(-45, -135, -60)
        ek.compute_forward_kinematics()
        return ek

    def update_plot(self, event=None):
        try:
            theta1 = self.slider_theta1.get()
            theta2 = self.slider_theta2.get()
            thetaF = self.slider_thetaF.get()

            self.ek.set_angles(theta1, theta2, thetaF)
            self.ek.compute_forward_kinematics()

            self.plot_extended_kinematics()
        except Exception as e:
            print(f"エラーが発生しました: {e}")

    def plot_extended_kinematics(self):
        self.ax.clear()

        result = self.ek.calculate()
        B1, M1, X, M2, B2, E, F = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["F"]

        if B1 is not None and M1 is not None:
            self.ax.plot([B1[0], M1[0]], [B1[1], M1[1]], 'r-o', label='B1-M1')
        if X is not None and M1 is not None:
            self.ax.plot([M1[0], X[0]], [M1[1], X[1]], 'b-o', label='M1-X')
        if X is not None and M2 is not None:
            self.ax.plot([X[0], M2[0]], [X[1], M2[1]], 'g-o', label='X-M2')
        if M2 is not None and B2 is not None:
            self.ax.plot([M2[0], B2[0]], [M2[1], B2[1]], 'y-o', label='M2-B2')
        if X is not None and E is not None:
            self.ax.plot([X[0], E[0]], [X[1], E[1]], 'mo-', label='X-E')
        if E is not None and F is not None:
            self.ax.plot([E[0], F[0]], [E[1], F[1]], 'c-o', label='E-F')

        for point, color, label in zip([B1, M1, X, M2, B2, E, F],
                                       ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan'],
                                       ['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F']):
            if point is not None:
                self.ax.scatter(*point, color=color, zorder=5)
                self.ax.text(point[0], point[1], label, fontsize=12, ha='right')

        self.ax.set_xlim(-600, 600)
        self.ax.set_ylim(-1000, 200)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Extended Kinematics Visualization')
        self.ax.grid(True)
        self.ax.legend()
        self.canvas.draw()

if __name__ == '__main__':
    app = KinematicsApp()
    app.mainloop()
