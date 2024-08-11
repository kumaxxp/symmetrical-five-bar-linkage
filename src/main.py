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
        self.geometry('1000x600')

        # 初期角度の設定
        self.initial_theta1 = -45
        self.initial_theta2 = -135
        self.initial_thetaF = -60

        # メインフレームの作成
        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # グラフ用フレームの作成（左側に配置）
        self.graph_frame = ttk.Frame(self.main_frame)
        self.graph_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # スライダ用フレームの作成（右側に配置）
        self.slider_frame = ttk.Frame(self.main_frame)
        self.slider_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # Matplotlib Figureの作成
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.figure, self.graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # スライダの作成
        self.slider_theta1 = ttk.Scale(self.slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=self.update_plot)
        self.slider_theta1.set(self.initial_theta1)
        ttk.Label(self.slider_frame, text='Theta1').pack(side=tk.TOP, fill=tk.X)
        self.slider_theta1.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        self.slider_theta2 = ttk.Scale(self.slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=self.update_plot)
        self.slider_theta2.set(self.initial_theta2)
        ttk.Label(self.slider_frame, text='Theta2').pack(side=tk.TOP, fill=tk.X)
        self.slider_theta2.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        self.slider_thetaF = ttk.Scale(self.slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=self.update_plot)
        self.slider_thetaF.set(self.initial_thetaF)
        ttk.Label(self.slider_frame, text='ThetaF').pack(side=tk.TOP, fill=tk.X)
        self.slider_thetaF.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

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
        ek.set_angles(self.initial_theta1, self.initial_theta2, self.initial_thetaF)
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

    def plot_line(self, point1, point2, color, label):
        """2点間の線をプロットするヘルパー関数"""
        if point1 is not None and point2 is not None:
            self.ax.plot([point1[0], point2[0]], [point1[1], point2[1]], f'{color}-o', label=label)

    def plot_extended_kinematics(self):
        self.ax.clear()

        result = self.ek.calculate()
        B1, M1, X, M2, B2, E, F = result["B1"], result["M1"], result["X"], result["M2"], result["B2"], result["E"], result["F"]

        # 線をプロット
        self.plot_line(B1, M1, 'r', 'B1-M1')
        self.plot_line(M1, X, 'b', 'M1-X')
        self.plot_line(X, M2, 'g', 'X-M2')
        self.plot_line(M2, B2, 'y', 'M2-B2')
        self.plot_line(X, E, 'm', 'X-E')
        self.plot_line(E, F, 'c', 'E-F')

        # ポイントをプロット
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

    def destroy(self):
        # Matplotlibのクリーンアップ
        self.canvas.get_tk_widget().destroy()
        plt.close(self.figure)  # Figureを閉じる
        super().destroy()  # Tkinterのdestroyを呼び出す

if __name__ == '__main__':
    app = KinematicsApp()
    app.mainloop()
