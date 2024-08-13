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

        # クラス属性として明示的に定義
        self.scale = 0.4
        self.offset_x = 0
        self.offset_y = 0

        self.setup_ui()
        self.setup_kinematics()
        self.set_initial_slider_values()
        self.last_angles = {'left': {}, 'right': {}}

        # 初期状態を計算
        self.initialize_kinematics()
        
        # キャンバスが描画された後にupdate_plotを呼び出す
        self.after(100, self.update_plot)        

    def initialize_kinematics(self):
        for leg, ek in [('left', self.ek_left), ('right', self.ek_right)]:
            initial_angles = {
                'theta1': self.sliders[leg]['theta1'].get(),
                'theta2': self.sliders[leg]['theta2'].get(),
                'thetaF': self.sliders[leg]['thetaF'].get()
            }
            ek.set_angles(**initial_angles)
            ek.compute_forward_kinematics()
            self.last_angles[leg] = initial_angles.copy()

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

        self.sliders = {}  # ここで self.sliders を初期化
        for leg in ['left', 'right']:
            leg_frame = ttk.LabelFrame(self.slider_frame, text=f"{leg.capitalize()} Leg")
            leg_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
            self.sliders[leg] = {}
            for angle in ['theta1', 'theta2', 'thetaF']:
                self.sliders[leg][angle] = self.create_slider_with_label(
                    f"{leg.capitalize()} {angle}", 
                    0,
                    leg_frame
                )

    def set_initial_slider_values(self):
        for leg in ['left', 'right']:
            self.sliders[leg]['theta1'].set(-45)
            self.sliders[leg]['theta2'].set(-135)
            self.sliders[leg]['thetaF'].set(-50)

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
        if not hasattr(self, 'ek_left') or not hasattr(self, 'ek_right'):
            return  # キネマティクスがまだセットアップされていない場合は早期リターン

        for leg, ek in [('left', self.ek_left), ('right', self.ek_right)]:
            current_angles = {
                'theta1': self.sliders[leg]['theta1'].get(),
                'theta2': self.sliders[leg]['theta2'].get(),
                'thetaF': self.sliders[leg]['thetaF'].get()
            }

            if not hasattr(self, 'last_angles'):
                self.last_angles = {'left': {}, 'right': {}}

            if current_angles != self.last_angles.get(leg, {}):
                ek.set_angles(**current_angles)
                ek.compute_forward_kinematics()
                self.last_angles[leg] = current_angles.copy()

        self.draw_extended_kinematics()
        self.draw_transformed_kinematics()

        # キャンバスのサイズが変更された場合に再描画
        self.canvas1.update()
        self.canvas2.update()

    def draw_extended_kinematics(self):
        self.canvas1.delete("all")
        self.draw_kinematics(self.canvas1, False)

    def draw_transformed_kinematics(self):
        self.canvas2.delete("all")
        self.draw_kinematics(self.canvas2, True)

    def draw_kinematics(self, canvas, transform):
        canvas_width = canvas.winfo_width()
        canvas_height = canvas.winfo_height()

        # リンク構造の描画
        left_points = self.ek_left.format_result()
        right_points = self.ek_right.format_result()

        transformed_left_points = {}
        transformed_right_points = {}

        scale = self.scale

        if transform:
            # 左足の変換
            angle_flower_left = self.angle_between_vectors((0, 0), (1, 0), left_points['E'], left_points['F'])
            transformer_left = Transformation2D(origin=left_points['E'], angle=-angle_flower_left, translation=-left_points['E'])
            transformed_left_points = {key: transformer_left.transform(value) for key, value in left_points.items()}

            # 右足の変換
            # 1. 左足のB1-B2ベクトルを計算
            left_b1b2_vector = np.array(transformed_left_points['B2']) - np.array(transformed_left_points['B1'])
            
            # 2. 右足のB1-B2ベクトルを計算
            right_b1b2_vector = np.array(right_points['B2']) - np.array(right_points['B1'])
            
            # 3. 回転角度を計算
            angle = np.arctan2(left_b1b2_vector[1], left_b1b2_vector[0]) - np.arctan2(right_b1b2_vector[1], right_b1b2_vector[0])
            angle_degrees = np.degrees(angle)
            
            # 4. 平行移動量を計算
            translation = np.array(transformed_left_points['B1']) - np.array(right_points['B1'])

            # 5. 変換を適用
            transformer_right = Transformation2D(origin=right_points['B1'], angle=angle_degrees, translation=translation)
            transformed_right_points = {key: transformer_right.transform(value) for key, value in right_points.items()}

            # 変換時のスケールとオフセットの計算
            all_points = list(transformed_left_points.values()) + list(transformed_right_points.values())
            min_x = min(p[0] for p in all_points)
            max_x = max(p[0] for p in all_points)
            min_y = min(p[1] for p in all_points)
            max_y = max(p[1] for p in all_points)
            
            width = max_x - min_x
            height = max_y - min_y
        #    scale = min(canvas_width / width, canvas_height / height) * 0.8
            offset_x = canvas_width / 2
            offset_y = canvas_height * 0.8
        else:
            transformed_left_points = left_points
            transformed_right_points = right_points
        #    scale = min(canvas_width / 1200, canvas_height / 1200)
            offset_x = canvas_width / 2
            offset_y = canvas_height / 2

        # グリッドを描画
        grid_color = "#E0E0E0"  # 薄いグレー
        grid_spacing = 100  # グリッドの間隔（ピクセル単位）

        # 縦線を描画
        for x in range(int(-offset_x), int(canvas_width - offset_x), grid_spacing):
            canvas.create_line(x + offset_x, 0, x + offset_x, canvas_height, fill=grid_color)

        # 横線を描画
        for y in range(int(-offset_y), int(canvas_height - grid_spacing)):
            canvas.create_line(0, y + offset_y, canvas_width, y + offset_y, fill=grid_color)

        # X軸とY軸を描画（少し濃い色で）
        axis_color = "#A0A0A0"  # 濃いめのグレー
        canvas.create_line(0, offset_y, canvas_width, offset_y, fill=axis_color, width=2)  # X軸
        canvas.create_line(offset_x, 0, offset_x, canvas_height, fill=axis_color, width=2)  # Y軸

        # 軸のラベルを追加
        label_color = "#606060"  # ダークグレー
        canvas.create_text(canvas_width - 20, offset_y - 20, text="X", fill=label_color)
        canvas.create_text(offset_x + 20, 20, text="Y", fill=label_color)

        # 目盛りを追加
        for i in range(-5, 6):
            if i != 0:
                # X軸の目盛り
                x = offset_x + i * grid_spacing
                canvas.create_line(x, offset_y - 5, x, offset_y + 5, fill=axis_color)
                canvas.create_text(x, offset_y + 20, text=str(i * 100), fill=label_color)

                # Y軸の目盛り
                y = offset_y - i * grid_spacing
                canvas.create_line(offset_x - 5, y, offset_x + 5, y, fill=axis_color)
                canvas.create_text(offset_x - 20, y, text=str(i * 100), fill=label_color)

        for leg, points in [('left', transformed_left_points), ('right', transformed_right_points)]:
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
