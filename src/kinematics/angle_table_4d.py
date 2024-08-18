import numpy as np
import csv
import json

class AngleTable4D:
    def __init__(self, min_angle=-180.0, max_angle=180.0, step=1.0, data_callback_before=None, data_callback_after=None):
        """
        コンストラクタで角度範囲、刻み幅、データコールバックを設定する。
        
        :param min_angle: 角度範囲の最小値 (デフォルト -180.0)
        :param max_angle: 角度範囲の最大値 (デフォルト 180.0)
        :param step: 角度の刻み幅 (デフォルト 1.0)
        :param data_callback: 各角度で生成するデータを返すコールバック関数
        """
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.step = step
        self.data_callback_before = data_callback_before if data_callback_before else self.default_data_callback
        self.data_callback_after  = data_callback_after if data_callback_after else self.default_data_callback

        # 角度の範囲を生成
        self.angle_range = np.arange(self.min_angle, self.max_angle + self.step, self.step)
        self.dim_size = len(self.angle_range)

        # 角度をインデックスに変換する辞書を作成
        self.angle_to_index = np.arange(self.dim_size)
        
        # 4次元テーブルの初期化
        self.table_4d = None

    def default_data_callback(self, angle1, angle2, angle3, angle4):
        """
        デフォルトのデータコールバック関数。
        各角度に対応する辞書型データを生成。
        """
        return {
            "angle_1": angle1,
            "angle_2": angle2,
            "angle_3": angle3,
            "angle_4": angle4,
            "sum": angle1 + angle2 + angle3 + angle4,
            "product": angle1 * angle2 * angle3 * angle4
        }

    def generate_table(self):
        # Step 1: Generate 2D table for the first two dimensions
        table_2d = np.full((self.dim_size, self.dim_size), None)
        valid_indices = []

        for i in range(self.dim_size):
            for j in range(self.dim_size):
                angle1 = self.angle_range[i]
                angle2 = self.angle_range[j]
                result = self.data_callback_before(angle1, angle2)
                if result is not None:
                    table_2d[i, j] = True
                    valid_indices.append((i, j))

        # Step 2: Generate 4D table based on valid indices from 2D table
        self.table_4d = {}

        for i, j in valid_indices:
            for k, l in valid_indices:
                angle1 = self.angle_range[i]
                angle2 = self.angle_range[j]
                angle3 = self.angle_range[k]
                angle4 = self.angle_range[l]
                
                result = self.data_callback_after(angle1, angle2, angle3, angle4)
                if result is not None:
                    self.table_4d[(i, j, k, l)] = result

    def get_data_by_angle(self, angle1, angle2, angle3, angle4):
        """
        指定された角度に対応するテーブルのデータを取得する。
        
        :param angle1: 第1次元の角度
        :param angle2: 第2次元の角度
        :param angle3: 第3次元の角度
        :param angle4: 第4次元の角度
        :return: 指定された角度に対応する辞書型データ
        """

        # 角度を最も近い有効な角度に丸める
        rounded_angle1 = self.round_to_nearest(angle1)
        rounded_angle2 = self.round_to_nearest(angle2)
        rounded_angle3 = self.round_to_nearest(angle3)
        rounded_angle4 = self.round_to_nearest(angle4)
        
        # 角度をインデックスに変換
        index1 = self.angle_to_index.get(rounded_angle1)
        index2 = self.angle_to_index.get(rounded_angle2)
        index3 = self.angle_to_index.get(rounded_angle3)
        index4 = self.angle_to_index.get(rounded_angle4)

        # インデックスが有効であるか確認
        if index1 is not None and index2 is not None and index3 is not None and index4 is not None:
            return self.table_4d.get((index1, index2, index3, index4))
        else:
            raise ValueError(f"指定された角度が範囲外です: {angle1}, {angle2}, {angle3}, {angle4}")

    def round_to_nearest(self, angle):
        """
        与えられた角度を最も近い有効な角度に丸める
        """
        return round((angle - self.min_angle) / self.step) * self.step + self.min_angle

    def save_to_csv(self, filename):
        """
        テーブルの内容をCSVファイルに保存する
        """
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # ヘッダーを書き込む
            writer.writerow(['angle1', 'angle2', 'angle3', 'angle4'] + list(next(iter(self.table_4d.values())).keys()))
            # データを書き込む
            for (i, j, k, l), data in self.table_4d.items():
                angle1, angle2, angle3, angle4 = self.angle_range[i], self.angle_range[j], self.angle_range[k], self.angle_range[l]
                writer.writerow([angle1, angle2, angle3, angle4] + list(data.values()))

    def save_to_json(self, filename):
        """
        テーブルの内容をJSONファイルに保存する
        """
        json_data = {}
        for (i, j, k, l), data in self.table_4d.items():
            angle1, angle2, angle3, angle4 = self.angle_range[i], self.angle_range[j], self.angle_range[k], self.angle_range[l]
            key = f"{angle1},{angle2},{angle3},{angle4}"
            json_data[key] = data
        
        with open(filename, 'w') as jsonfile:
            json.dump(json_data, jsonfile, indent=2)

# 上位クラスの例
class AngleTableManager:
    def __init__(self, step=1.0):
        """
        AngleTableManagerは、AngleTable4Dの管理を行い、テーブル生成を呼び出す。
        
        :param step: 角度の刻み幅 (デフォルト 1.0)
        """
        self.angle_table = AngleTable4D(step=step, data_callback=self.custom_data_callback)

    def custom_data_callback(self, angle1, angle2, angle3, angle4): 
        # 実際の条件をここに記述
        if not self.is_valid_combination(angle1, angle2, angle3, angle4):
            return None
        return {
            "angle_1": angle1,
            "angle_2": angle2,
            "angle_3": angle3,
            "angle_4": angle4,
            "sum_of_squares": angle1**2 + angle2**2 + angle3**2 + angle4**2,
            "max_angle": max(angle1, angle2, angle3, angle4)
        }

    def is_valid_combination(self, angle1, angle2, angle3, angle4):
        # ここに実際の妥当性チェックロジックを実装
        # 例: return abs(angle1) + abs(angle2) + abs(angle3) + abs(angle4) <= 360
        pass

    def create_table(self):
        """
        テーブルを生成するメソッド。
        """
        self.angle_table.generate_table()

    def get_data(self, angle1, angle2, angle3, angle4):
        """
        指定された角度に対応するデータを取得するメソッド。
        """
        return self.angle_table.get_data_by_angle(angle1, angle2, angle3, angle4)

# 使用例
if __name__ == "__main__":
    # 上位クラスのインスタンスを作成
    manager = AngleTableManager(step=10.0)

    # テーブルを生成
    manager.create_table()

    # 角度で直接データを取得
    angle1 = 20.0
    angle2 = 20.0
    angle3 = 20.0
    angle4 = 20.0

    result = manager.get_data(angle1, angle2, angle3, angle4)
    print(f"角度 ({angle1}, {angle2}, {angle3}, {angle4}) に対応するデータ: {result}")
