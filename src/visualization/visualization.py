import math
import numpy as np
from typing import Tuple, Optional

LENGTH = 350

class Visualization:
    def __init__(self, canvas):
        self.canvas = canvas
        self.offset_x = 0
        self.offset_y = 0
        self.grid_items = []
        self.kinematics_items = {'left': {}, 'right': {}}
        self.scale = 1  # スケールを初期化
        self.offset_y_ratio = 3/4   # 3:1の割合にするためY+を定義
        
    def draw_transformed_kinematics(self, hip):
        self.canvas.delete("all")
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        
        # Y方向のオフセットを3:1に調整
        new_offset_x = canvas_width / 2
        new_offset_y = canvas_height * self.offset_y_ratio

        self.offset_x = new_offset_x
        self.offset_y = new_offset_y
        self.scale = 1  # スケールを設定

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

        # 横線を描画(Y軸-側)
        diff_offset_y_ratio = 1.0 - self.offset_y_ratio
        for y in range(int(0), int(canvas_height*diff_offset_y_ratio), grid_spacing):
            item_id = self.canvas.create_line(0, self.offset_y + y, canvas_width, self.offset_y + y, fill=grid_color)
            self.grid_items.append(item_id)

        # 横線を描画(Y軸+側)
        diff_offset_y_ratio = 1.0 - self.offset_y_ratio
        for y in range(int(0), int(canvas_height*self.offset_y_ratio), grid_spacing):
            item_id = self.canvas.create_line(0, self.offset_y - y, canvas_width, self.offset_y - y, fill=grid_color)
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
        """
        重心位置を描画し、重心位置からX=0の直線に直角に交わる点線を描画する
        :param hip: Hipオブジェクト
        """
        if hip.left_com is not None:
            self.draw_point(hip.left_com, 'blue', 8, "Left CoM")
        if hip.right_com is not None:
            self.draw_point(hip.right_com, 'red', 8, "Right CoM")
        if hip.center_com is not None:
            self.draw_point(hip.center_com, 'green', 10, "Total CoM")
            self.draw_perpendicular_line(hip.center_com)

    def draw_perpendicular_line(self, com):
        """
        重心位置からX=0の直線に直角に交わる点線を描画し、定義した長さの位置に点をプロットする
        :param com: 重心位置 (x, y)
        """
        x, y = self.transform_point(com)
        canvas_height = self.canvas.winfo_height()
        self.canvas.create_line(x, y, x, canvas_height, fill='gray', dash=(4, 4))

        # 定義した長さの位置に点をプロット
        point_y = y + LENGTH
        self.canvas.create_oval(x-5, point_y-5, x+5, point_y+5, fill='red')
    
    def draw_point(self, point, color, size, label):
        x, y = self.transform_point(point)
        self.canvas.create_oval(x-size, y-size, x+size, y+size, fill='', outline=color, width = 2)
        # self.canvas.create_text(x, y-15, text=label, fill=color)

    def draw_arc(self, center: Tuple[float, float], P1: Tuple[float, float], P2: Tuple[float, float], r: float, color: str):
        """
        円の中心点からP1, P2を通る弧を描画します。
        """
        x0, y0 = center
        x1, y1 = P1
        x2, y2 = P2

        # 角度の計算
        angle2 = -math.degrees(math.atan2(y1 - y0, x1 - x0))
        angle1 = -math.degrees(math.atan2(y2 - y0, x2 - x0))

        # Tkinterのcreate_arcは開始角度が右方向（0度）から反時計回り
        # 弧の範囲を決定
        start = angle1
        extent = angle2 - angle1
        if extent <= 0:
            extent += 360

        self.canvas.create_arc(
            x0 - r, y0 + r, x0 + r, y0 - r,
            start=start, extent=extent, outline=color, width=2 , style="arc"
        )

    def update_links(self, points, leg):
        # リンクの表示
        for start, end, color in [('B1', 'M1', 'red'), ('M1', 'X', 'blue'), ('X', 'M2', 'green'),
                                  ('M2', 'B2', 'yellow'), ('X', 'E', 'magenta'), ('E', 'F', 'cyan'),
                                  ('E', 'H', 'red'), ('F', 'I', 'red')]:
            if points[start] is not None and points[end] is not None:
                x1, y1 = self.transform_point(points[start])
                x2, y2 = self.transform_point(points[end])
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                    link_width = 2
                else:
                    link_width = 4
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=link_width)

        """
        # ワイヤーの表示
        for start, end, color in [('W1', 'W11', 'black'), ('W11', 'W12', 'blue'), 
                                  ('W2', 'W21', 'black'), ('W21', 'W22', 'blue'),
                                  ('W31', 'W32', 'black'), ('W41', 'W42', 'black') ]:
            if points[start] is not None and points[end] is not None:
                x1, y1 = self.transform_point(points[start])
                x2, y2 = self.transform_point(points[end])
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                link_width = 2
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=link_width, dash=(4, 4))
        """

        # 円弧の表示
        ptG = self.transform_point(points['G'])
        ptE = self.transform_point(points['E'])
        ptF = self.transform_point(points['F'])

        color = 'red'
        if leg == 'right':
            color = self.lighten_color(color, amount=150)
            link_width = 2
        else:
            link_width = 4

        self.draw_arc(ptG, ptF, ptE, r = 500 , color = color)

            #    key = f"{start}-{end}"
            #    if key in self.kinematics_items[leg]:
            #        self.canvas.coords(self.kinematics_items[leg][key], x1, y1, x2, y2)
            #    else:
            #        self.kinematics_items[leg][key] = self.canvas.create_line(x1, y1, x2, y2, fill=color, width=link_width)

    def update_points(self, points, leg):
        # 関節の表示
        for point, color in zip(['B1', 'M1', 'X', 'M2', 'B2', 'E', 'F'],
                                ['red', 'red', 'blue', 'green', 'green', 'magenta', 'cyan']):
            if points[point] is not None:
                x, y = self.transform_point(points[point])
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                
                self.canvas.create_oval(x-3, y-3, x+3, y+3, fill=color)
                coord_text = f'({points[point][0]:.0f}, {points[point][1]:.0f})'
                self.canvas.create_text(x+10, y+10, text=coord_text, anchor='sw')

        # 腱や筋肉の接合部の表示(座標は書かない)
        for point, color in zip(['W1', 'W2', 'W11', 'W21', 'W12', 'W22', 'W31', 'W32', 'W41', 'W42'], 
                                ['red', 'red', 'red', 'red', 'red', 'red', 'red', 'red', 'red', 'red']):
            if points[point] is not None:
                x, y = self.transform_point(points[point])
                if leg == 'right':
                    color = self.lighten_color(color, amount=150)
                
                self.canvas.create_oval(x-5, y-5, x+5, y+5, fill=color)

    def transform_point(self, point):
        """
        座標をキャンバス上の座標に変換する
        :param point: 座標 (x, y)
        :return: キャンバス上の座標 (x, y)
        """
        x, y = point
        transformed_x = self.offset_x + x * self.scale
        transformed_y = self.offset_y - y * self.scale
        return transformed_x, transformed_y

    def lighten_color(self, color, amount=150):
        color_code = self.canvas.winfo_rgb(color)
        r, g, b = [x // 256 for x in color_code]
        r = min(255, r + amount)
        g = min(255, g + amount)
        b = min(255, b + amount)
        return f'#{r:02x}{g:02x}{b:02x}'
