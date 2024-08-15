import tkinter as tk
from tkinter import ttk
import numpy as np
from extended_kinematics import ExtendedKinematics
from transformation import Transformation2D
from hip import Hip

class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Kinematics Visualization')
        self.geometry('1600x1000')
        self.resizable(False, True)

        # クラス属性として明示的に定義
        self.scale = 0.25
        self.offset_x = 0
        self.offset_y = 0

        self.setup_ui()
        self.setup_kinematics()
        self.set_initial_angles()
        self.last_angles = {'left': {}, 'right': {}}

        # 初期状態を計算
        self.initialize_kinematics()
        
        # キャンバスが描画された後にupdate_plotを呼び出す
        self.after(100, self.update_plot)        

    def setup_kinematics(self):
        left_leg = ExtendedKinematics(100, 200, 450, 500, 300, 300)
        right_leg = ExtendedKinematics(100, 200, 450, 500, 300, 300)
        self.hip = Hip(left_leg, right_leg)

        # 初期角度を設定（例として値を設定していますが、必要に応じて調整してください）
        self.initial_angles = {
            'left': {'theta1': -10, 'theta2': -100, 'thetaF': -50},
            'right': {'theta1': -10, 'theta2': -100, 'thetaF': -50}
        }

        # 初期角度を設定
        for leg in ['left', 'right']:
            self.hip.set_leg_angles(leg, **self.initial_angles[leg])

    def set_initial_angles(self):
        for leg in ['left', 'right']:
            for angle, value in self.initial_angles[leg].items():
                self.sliders[leg][angle].set(value)

    def initialize_kinematics(self):
        for leg in ['left', 'right']:
            initial_angles = {
                'theta1': self.sliders[leg]['theta1'].get(),
                'theta2': self.sliders[leg]['theta2'].get(),
                'thetaF': self.sliders[leg]['thetaF'].get()
            }
            self.hip.set_leg_angles(leg, **initial_angles)
            self.last_angles[leg] = initial_angles.copy()
            self.hip.compute_leg_positions()

    def setup_ui(self):
        # メインフレームの作成
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 左側のフレーム（スライダー用）
        left_frame = ttk.Frame(main_frame, width = 400)  # 幅を400に設定
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        left_frame.pack_propagate(False)  # サイズを固定

        # 右側のフレーム（キャンバス用）
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # スライダーの設定
        self.sliders = {'left': {}, 'right': {}}
        self.angle_labels = {'left': {}, 'right': {}}
        for leg in ['left', 'right']:
            leg_frame = ttk.LabelFrame(left_frame, text=f"{leg.capitalize()} Leg")
            leg_frame.pack(fill=tk.X, padx=5, pady=5)
            for angle in ['theta1', 'theta2', 'thetaF']:
                slider = ttk.Scale(leg_frame, from_=-180, to=180, orient=tk.HORIZONTAL, 
                                   command=lambda x, l=leg, a=angle: self.on_slider_change(l, a))
                slider.pack(fill=tk.X, padx=5, pady=5)
                self.sliders[leg][angle] = slider

                label_frame = ttk.Frame(leg_frame)
                label_frame.pack(fill=tk.X, padx=5)
                ttk.Label(label_frame, text=f"{angle}:").pack(side=tk.LEFT)
                value_label = ttk.Label(label_frame, text="0.0")
                value_label.pack(side=tk.RIGHT)
                self.angle_labels[leg][angle] = value_label

        # キャンバスの設定
        self.canvas2 = tk.Canvas(right_frame, bg='white')
        self.canvas2.pack(fill=tk.BOTH, expand=True)

    def on_slider_change(self, leg, angle):
        value = self.sliders[leg][angle].get()
        self.angle_labels[leg][angle].config(text=f"{value:.1f}")
        self.update_plot()

    def update_plot(self):
        for leg in ['left', 'right']:
            current_angles = {
                'theta1': self.sliders[leg]['theta1'].get(),
                'theta2': self.sliders[leg]['theta2'].get(),
                'thetaF': self.sliders[leg]['thetaF'].get()
            }
            if current_angles != self.last_angles.get(leg, {}):
                self.hip.set_leg_angles(leg, **current_angles)
                self.last_angles[leg] = current_angles.copy()

        self.hip.compute_leg_positions()
        self.draw_transformed_kinematics()
        self.canvas2.update()

    def draw_transformed_kinematics(self):
        self.canvas2.delete("all")
        self.draw_kinematics(self.canvas2)

    def draw_kinematics(self, canvas):
        canvas_width = canvas.winfo_width()
        canvas_height = canvas.winfo_height()

        scale = self.scale
        offset_x = canvas_width / 2
        offset_y = canvas_height / 2

        self.draw_grid(canvas, offset_x, offset_y)

        transformed_points = self.hip.get_transformed_points()
        for leg in ['left', 'right']:
            self.draw_links(canvas, transformed_points[leg], leg, scale, offset_x, offset_y)
            self.draw_points(canvas, transformed_points[leg], leg, scale, offset_x, offset_y)

    def draw_grid(self, canvas, offset_x, offset_y):
        canvas_width = canvas.winfo_width()
        canvas_height = canvas.winfo_height()

        # グリッドを描画
        grid_color = "#E0E0E0"  # 薄いグレー
        grid_spacing = 25  # グリッドの間隔（ピクセル単位）

        # 縦線を描画
        for x in range(int(-offset_x), int(canvas_width - offset_x), grid_spacing):
            canvas.create_line(x + offset_x, 0, x + offset_x, canvas_height, fill=grid_color)

        # 横線を描画
        for y in range(int(-offset_y), int(canvas_height - offset_y), grid_spacing):
            canvas.create_line(0, y + offset_y, canvas_width, y + offset_y, fill=grid_color)

        # X軸とY軸を描画（少し濃い色で）
        axis_color = "#A0A0A0"  # 濃いめのグレー
        canvas.create_line(0, offset_y, canvas_width, offset_y, fill=axis_color, width=2)  # X軸
        canvas.create_line(offset_x, 0, offset_x, canvas_height, fill=axis_color)  # Y軸

        # 軸のラベルを追加
        label_color = "#606060"  # ダークグレー
        canvas.create_text(canvas_width - 20, offset_y - 20, text="X", fill=label_color)
        canvas.create_text(offset_x + 20, 20, text="Y", fill=label_color)

        # 目盛りを追加
        for i in range(-5, 6):
            if i != 0:
                # X軸の目盛り
                x = offset_x + i * grid_spacing * 4
                canvas.create_line(x, offset_y - 5, x, offset_y + 5, fill=axis_color)
                canvas.create_text(x, offset_y + 20, text=str(i * 100), fill=label_color)

                # Y軸の目盛り
                y = offset_y - i * grid_spacing * 4
                canvas.create_line(offset_x - 5, y, offset_x + 5, y, fill=axis_color)
                canvas.create_text(offset_x - 20, y, text=str(i * 100), fill=label_color)

    def draw_links(self, canvas, points, leg, scale, offset_x, offset_y):
        for start, end, color in [('B1', 'M1', 'red'), ('M1', 'X', 'blue'), ('X', 'M2', 'green'),
                                  ('M2', 'B2', 'yellow'), ('X', 'E', 'magenta'), ('E', 'F', 'cyan')]:
            if points[start] is not None and points[end] is not None:
                x1, y1 = self.transform_point(points[start], scale, offset_x, offset_y)
                x2, y2 = self.transform_point(points[end], scale, offset_x, offset_y)
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                    link_width = 2
                else:
                    link_width = 4
                canvas.create_line(x1, y1, x2, y2, fill=color, width=link_width)

    def draw_points(self, canvas, points, leg, scale, offset_x, offset_y):
        for point, color in zip(['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F'],
                                ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan']):
            if points[point] is not None:
                x, y = self.transform_point(points[point], scale, offset_x, offset_y)
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                canvas.create_oval(x-3, y-3, x+3, y+3, fill=color)
                coord_text = f'({points[point][0]:.0f}, {points[point][1]:.0f})'
                canvas.create_text(x+10, y+10, text=coord_text, anchor='sw')

    def set_initial_slider_values(self):
        for leg in ['left', 'right']:
            for angle in ['theta1', 'theta2', 'thetaF']:
                self.sliders[leg][angle].set(0)  # デフォルト値を0に設定

    # その他のメソッド（draw_grid, draw_links, draw_points, etc.）は同じまま

    def transform_point(self, point, scale, offset_x, offset_y):
        return (point[0] * scale + offset_x, -point[1] * scale + offset_y)

    def lighten_color(self, color, amount=150):
        # 色名を16進数カラーコードに変換
        color_code = self.winfo_rgb(color)
        r, g, b = [x // 256 for x in color_code]

        # 各成分を明るくする
        r = min(255, r + amount)
        g = min(255, g + amount)
        b = min(255, b + amount)

        # RGB 値を16進数のカラーコードに戻す
        return f'#{r:02x}{g:02x}{b:02x}'

if __name__ == '__main__':
    app = KinematicsApp()
    app.mainloop()
