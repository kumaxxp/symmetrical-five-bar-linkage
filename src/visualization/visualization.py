import math
import numpy as np

class Visualization:
    def __init__(self, canvas):
        self.canvas = canvas
        self.scale = 0.5
        self.offset_x = 0
        self.offset_y = 0

    def draw_transformed_kinematics(self, hip):
        self.canvas.delete("all")
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        self.offset_x = canvas_width / 2
        self.offset_y = canvas_height / 2

        self.draw_grid()
        self.draw_kinematics(hip)

    def draw_grid(self):
        # グリッドを描画
        grid_color = "#E0E0E0"  # 薄いグレー
        grid_spacing = int(24 * self.scale)  # グリッドの間隔（ピクセル単位）

        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        # 縦線を描画
        for x in range(int(0), int(canvas_width/2), grid_spacing):
            self.canvas.create_line(self.offset_x - x, 0, self.offset_x - x, canvas_height, fill=grid_color)
            self.canvas.create_line(self.offset_x + x, 0, self.offset_x + x, canvas_height, fill=grid_color)

        # 横線を描画
        for y in range(int(0), int(canvas_height/2), grid_spacing):
            self.canvas.create_line(0, self.offset_y - y, canvas_width, self.offset_y - y, fill=grid_color)
            self.canvas.create_line(0, self.offset_y + y, canvas_width, self.offset_y + y, fill=grid_color)

        # X軸とY軸を描画（少し濃い色で）
        axis_color = "#A0A0A0"  # 濃いめのグレー
        self.canvas.create_line(0, self.offset_y, canvas_width, self.offset_y, fill=axis_color, width=2)  # X軸
        self.canvas.create_line(self.offset_x, 0, self.offset_x, canvas_height, fill=axis_color, width=2)  # Y軸

        # 軸のラベルを追加
        label_color = "#606060"  # ダークグレー
        self.canvas.create_text(canvas_width - 20, self.offset_y - 20, text="X", fill=label_color)
        self.canvas.create_text(self.offset_x + 20, 20, text="Y", fill=label_color)

        # 目盛りを追加
        for i in range(-10, 11):
            if i != 0:
                # X軸の目盛り
                x = self.offset_x + i * grid_spacing * 4
                self.canvas.create_line(x, self.offset_y - 5, x, self.offset_y + 5, fill=axis_color)
                self.canvas.create_text(x, self.offset_y + 20, text=str(int(i * 100)), fill=label_color)

                # Y軸の目盛り
                y = self.offset_y - i * grid_spacing * 4
                self.canvas.create_line(self.offset_x - 5, y, self.offset_x + 5, y, fill=axis_color)
                self.canvas.create_text(self.offset_x - 20, y, text=str(int(i * 100)), fill=label_color)

    def draw_kinematics(self, hip):
        rotated_points = hip.get_rotated_points()
        for leg in ['left', 'right']:
            self.draw_links(rotated_points[leg], leg)
            self.draw_points(rotated_points[leg], leg)

    def draw_links(self, points, leg):
        for start, end, color in [('B1', 'M1', 'red'), ('M1', 'X', 'blue'), ('X', 'M2', 'green'),
                                  ('M2', 'B2', 'yellow'), ('X', 'E', 'magenta'), ('E', 'F', 'cyan')]:
            if points[start] is not None and points[end]:
                x1, y1 = self.transform_point(points[start])
                x2, y2 = self.transform_point(points[end])
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                    link_width = 2
                else:
                    link_width = 4
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=link_width)

    def draw_points(self, points, leg):
        for point, color in zip(['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F'],
                                ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan']):
            if points[point] is not None:
                x, y = self.transform_point(points[point])
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                self.canvas.create_oval(x-3, y-3, x+3, y+3, fill=color)
                coord_text = f'({points[point][0]:.0f}, {points[point][1]:.0f})'
                self.canvas.create_text(x+10, y+10, text=coord_text, anchor='sw')

    def transform_point(self, point):
        return (point[0] * self.scale + self.offset_x, -point[1] * self.scale + self.offset_y)

    def lighten_color(self, color, amount=150):
        color_code = self.canvas.winfo_rgb(color)
        r, g, b = [x // 256 for x in color_code]
        r = min(255, r + amount)
        g = min(255, g + amount)
        b = min(255, b + amount)
        return f'#{r:02x}{g:02x}{b:02x}'
