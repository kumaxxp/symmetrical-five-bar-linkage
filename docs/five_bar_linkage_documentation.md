# Symmetrical Five-Bar Linkage

## 概要

このドキュメントでは、前後対称な五節リンクの設計について説明します。以下の図は、五節リンクのコンポーネントとその接続を示しています。

## PlantUMLコード

以下は、前後対称な五節リンクを示すPlantUMLコードです。

``` plantuml
@startuml

skin rose

component [B2 モーター付き] as B2
component [B1 モーター付き] as B1
component [M1 ] as M1
component [X ] as X
component [M2 ] as M2
component [E エンドエフェクタ] as E

B2 -r[hidden]-> B1
B1 -d[hidden]-> M1
M1 -d[hidden]-> X
B2 -d[hidden]-> M2
X -d[hidden]-> E

B1 -r- B2:l

B1 -d- M1:b
B2 -d- M2:b
M1 -d- X:m
M2 -d- X:m
X -d- E:e
@enduml

```

---

順運動学における計算式を以下にまとめます。これにより、五節リンクの各点の位置を計算できます。以下の計算式を順番に適用して、各点の座標を求めます。


# 順運動学の設計

初期値として以下の数値が設定される。

- \( y = Y_b \)  の直線上に、B1,B2を配置する
- \( l \) は B2-B1 のリンク長
- \( b \) は B1-M1 および B2-M2 のリンク長
- \( m \) は M1-X および M2-X のリンク長
- \( \theta1 \) は B1-M1 の角度
- \( \theta2 \) は B2-M2 の角度

**点 \( B1 \) と \( B2 \) の座標** は初期値で決定します。

   - \( B1 = \left(\frac{l}{2}, Y_b\right) \)
   - \( B2 = \left(-\frac{l}{2}, Y_b\right) \)



## 1. 点 \( M1 \) と \( M2 \) の計算


**点 \( M1 \) と \( M2 \) の座標** は以下の式で計算されます。

- \(M1 = B1 + [b \cdot \cos(\theta1), b \cdot \sin(\theta1)]\)
- \(M2 = B2 + [b \cdot \cos(\theta2), b \cdot \sin(\theta2)]\)

ここで、
- \( \theta1 \) は B1-M1 の角度
- \( \theta2 \) は B2-M2 の角度
- \( b \) は B1-M1 および B2-M2 のリンク長

## 2. 点 X の計算
点 X の座標は、M1 および M2 の位置から、2つの円の交点として求められます。

円の中心間距離 \( d \) は以下の式で計算します。
- \( d = \|M1 - M2\| \)

交点の計算式は以下の通りです。
- \( a = \frac{r1^2 - r2^2 + d^2}{2d} \)
- \( h = \sqrt{r1^2 - a^2} \)
- \( x0 = x1 + \frac{a \cdot (x2 - x1)}{d} \)
- \( y0 = y1 + \frac{a \cdot (y2 - y1)}{d} \)
- \( X1 = [x0 + \frac{h \cdot (y2 - y1)}{d}, y0 - \frac{h \cdot (x2 - x1)}{d}] \)
- \( X2 = [x0 - \frac{h \cdot (y2 - y1)}{d}, y0 + \frac{h \cdot (x2 - x1)}{d}] \)

X1とX2のどちらをXとして採用するかは、凸形状の判定で決定する。

## 3. 凸形状の判定
凸多角形の判定には、以下の外積を利用します。

- 外積 = \( (b_x - a_x) \cdot (c_y - b_y) - (b_y - a_y) \cdot (c_x - b_x) \)

外積が正であれば凹形状と判定します。

## 注意事項
- 点 X の選定は、凸形状の条件を満たすかどうかで決定します。
- 凸多角形でない場合、エラーメッセージが表示されます。
