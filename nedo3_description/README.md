nedo3_description
==================================================

## 概要
本パッケージは，NEDO3デモ環境とそれに含まれるオブジェクトのモデルを提供する．

## urdfサブディレクトリ
- [nedo3_demo.urdf.xacro](./urdf/nedo3_demo.urdf.xacro): NEDO3デモ環境モデル．ラックやアームの配置を変更する場合は，本ファイルを編集する
- [rack_b.urdf.xacro](./urdf/rack_b.urdf.xacro): 床置きするベースラックのモデル
- [rack_t.urdf.xacro](./urdf/rack_t.urdf.xacro): ベースラックの上に乗せるトップラックのモデル
- [floor.urdf.xacro](./urdf/floor.urdf.xacro): テクスチャを貼った床のモデル
- [part_box_urdf.xacro](./urdf/parts_box.urdf.xacro): 部品箱のモデル．幅，奥行き，高さ，厚さ等を指定することによって様々な形状の箱を生成可能
- [put_boxes_on_racks.urdf.xacro](./urdf/put_boxes_on_racks.urdf.xacro): ラック上に部品箱を乗せるためのxacroファイル

個々のモデル(`xxx.urdf.xacro`)は，拡張子が`urdf`である対応ファイル(`xxx.urdf`)によってモデルツリーのルートを付加することにより，以下のコマンドで可視化できる．
```
$ roslauch urdf_tutorial display.launch model:=xxx.urdf
```
ただし，全体環境モデル`nedo3_demo.urdf.xacro`には既にツリーのルートが含まれているので，直接可視化できる．
```
$ roslauch urdf_tutorial display.launch model:=nedo3_demo.urdf.xacro
```
## configサブディレクトリ
 - [box_props.yaml](./config/box_props.yaml): 部品箱の属性
 - [part_props.yaml](./config/part_props.yaml): 部品の属性．この部品を入れている部品箱の種別も記録されている
 - [rack_props.yaml](./config/rack_props.yaml): ラックの属性．上段，下段それぞれに載せられている部品もここで指定する

## meshesサブディレクトリ
2種のラックのメッシュモデルを含む．[visualサブディレクトリ](./meshes/visual)には表示用の高解像度モデルが，[collisionサブディレクトリ](./meshes/collision/)には衝突判定用の低解像度モデルがそれぞれ収められている．これらは，URDFファイルから参照される．

## mediaサブディレクトリ
床モデルに貼るテクスチャが収められている．
