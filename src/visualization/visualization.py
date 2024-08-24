import math
import numpy as np

class Visualization:
    def __init__(self, canvas):
        self.canvas = canvas
        self.scale = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.grid_items = []
        self.kinematics_items = {'left': {}, 'right': {}}

    def draw_transformed_kinematics(self, hip):
        self.canvas.delete("all")
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        new_offset_x = canvas_width / 2
        new_offset_y = canvas_height / 2

        ## オフセットが変更された場合、またはグリッドがまだ描画されていない場合、グリッドを描画
        #if new_offset_x != self.offset_x or new_offset_y != self.offset_y or not self.grid_items:
        self.offset_x = new_offset_x
        self.offset_y = new_offset_y
        self.redraw_grid()

        self.draw_kinematics(hip)

    def redraw_grid(self):
        for item in self.grid_items:
            self.canvas.delete(item)
        self.grid_items = []
        self.draw_grid()        

    def draw_grid(self):
        # グリッドを描画
        grid_color = "#E0E0E0"  # 薄いグレー
        grid_spacing = int(24 * self.scale)  # グリッドの間隔（ピクセル単

        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        # 縦線を描画
        for x in range(int(0), int(canvas_width/2), grid_spacing):
            item_id = self.canvas.create_line(self.offset_x - x, 0, self.offset_x - x, canvas_height, fill=grid_color)
            self.grid_items.append(item_id)
            item_id = self.canvas.create_line(self.offset_x + x, 0, self.offset_x + x, canvas_height, fill=grid_color)
            self.grid_items.append(item_id)

        # 横線を描画
        for y in range(int(0), int(canvas_height/2), grid_spacing):
            item_id = self.canvas.create_line(0, self.offset_y - y, canvas_width, self.offset_y - y, fill=grid_color)
            self.grid_items.append(item_id)
            item_id = self.canvas.create_line(0, self.offset_y + y, canvas_width, self.offset_y + y, fill=grid_color)
            self.grid_items.append(item_id)

        # X軸とY軸を描画（少し濃い色で）
        axis_color = "#A0A0A0"  # 濃いめのグレー
        item_id = self.canvas.create_line(0, self.offset_y, canvas_width, self.offset_y, fill=axis_color, width=2)  # X軸
        self.grid_items.append(item_id)
        item_id = self.canvas.create_line(self.offset_x, 0, self.offset_x, canvas_height, fill=axis_color, width=2)  # Y軸
        self.grid_items.append(item_id)

        # 軸のラベルを追加
        label_color = "#606060"  # ダークグレー
        item_id = self.canvas.create_text(canvas_width - 20, self.offset_y - 20, text="X", fill=label_color)
        self.grid_items.append(item_id)
        item_id = self.canvas.create_text(self.offset_x + 20, 20, text="Y", fill=label_color)
        self.grid_items.append(item_id)

        # 目盛りを追加
        for i in range(-10, 11):
            if i != 0:
                # X軸の目盛り
                x = self.offset_x + i * grid_spacing * 4
                item_id = self.canvas.create_line(x, self.offset_y - 5, x, self.offset_y + 5, fill=axis_color)
                self.grid_items.append(item_id)
                item_id = self.canvas.create_text(x, self.offset_y + 20, text=str(int(i * 100)), fill=label_color)
                self.grid_items.append(item_id)

                # Y軸の目盛り
                y = self.offset_y - i * grid_spacing * 4
                item_id = self.canvas.create_line(self.offset_x - 5, y, self.offset_x + 5, y, fill=axis_color)
                self.grid_items.append(item_id)
                item_id = self.canvas.create_text(self.offset_x - 20, y, text=str(int(i * 100)), fill=label_color)
                self.grid_items.append(item_id)

    def draw_kinematics(self, hip):
        rotated_points = hip.get_rotated_points()
        for leg in ['left', 'right']:
            self.update_links(rotated_points[leg], leg)
            self.update_points(rotated_points[leg], leg)

        # 重心の計算と描画
        self.draw_center_of_mass(hip)

    def draw_center_of_mass(self, hip):
        if hip.left_com is not None:
            self.draw_point(hip.left_com, 'blue', 8, "Left CoM")
        if hip.right_com is not None:
            self.draw_point(hip.right_com, 'red', 8, "Right CoM")
        if hip.center_com is not None:
            self.draw_point(hip.center_com, 'green', 10, "Total CoM")

    def draw_point(self, point, color, size, label):
        x, y = self.transform_point(point)
        self.canvas.create_oval(x-size, y-size, x+size, y+size, fill='', outline=color, width = 2)
        # self.canvas.create_text(x, y-15, text=label, fill=color)

    def update_links(self, points, leg):
        for start, end, color in [('B1', 'M1', 'red'), ('M1', 'X', 'blue'), ('X', 'M2', 'green'),
                                  ('M2', 'B2', 'yellow'), ('X', 'E', 'magenta'), ('E', 'F', 'cyan')]:
            if points[start] is not None and points[end] is not None:
                x1, y1 = self.transform_point(points[start])
                x2, y2 = self.transform_point(points[end])
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                    link_width = 2
                else:
                    link_width = 4
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=link_width)
                
            #    key = f"{start}-{end}"
            #    if key in self.kinematics_items[leg]:
            #        self.canvas.coords(self.kinematics_items[leg][key], x1, y1, x2, y2)
            #    else:
            #        self.kinematics_items[leg][key] = self.canvas.create_line(x1, y1, x2, y2, fill=color, width=link_width)

    def update_points(self, points, leg):
        for point, color in zip(['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F', 'W1', 'W2', 'W11', 'W21', 'W12', 'W22'],
                                ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan', 'red', 'red', 'red', 'red', 'red', 'red']):
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
