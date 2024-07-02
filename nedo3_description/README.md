nedo3_description
==================================================

## 概要
本パッケージは，NEDO3デモ環境とそれに含まれるオブジェクトのモデルを提供する．

## urdfサブディレクトリ
- [nedo3_demo.urdf.xacro](./urdf/nedo3_demo.urdf.xacro): NEDO3デモ環境モデル．ラックやアームの配置を変更する場合は，本ファイルを編集する
- 
また，アームのエンドエフェクタに取り付けるマグネットグリッパに関連するモデルとして，次の4モデルを含む．
- [fste_hdb_45.urdf.xacro](./urdf/fste_hdb_45.urdf.xacro): シュマルツのスプリングプランジャー[FSTE-HDB](https://www.schmalz.co.jp/ja-jp/vacuum-technology-for-automation/vacuum-components/mounting-elements/spring-plungers/spring-plungers-fste-hdb-307103/)(45mmストローク)のモデル
- [fste_hdb_80.urdf.xacro](./urdf/fste_hdb_80.urdf.xacro): シュマルツのスプリングプランジャー[FSTE-HDB](https://www.schmalz.co.jp/ja-jp/vacuum-technology-for-automation/vacuum-components/mounting-elements/spring-plungers/spring-plungers-fste-hdb-307103/)(80mmストローク)のモデル
- [sgm_hp30.urdf.xacro](./urdf/sgm_hp30.urdf.xacro): シュマルツのマグネットグリッパ[SGM-HP30](https://www.schmalz.co.jp/ja-jp/vacuum-technology-for-automation/vacuum-components/special-grippers/magnetic-grippers/magnetic-grippers-sgm-hp-ht-306089/10.01.17.00316/)のモデル
- [plunger_fixture.urdf.xacro](./urdf/plunger_fixture.urdf.xacro): プランジャーをエンドエフェクタに取り付ける治具のモデル

さらに，次の4モデルを含む．
- [conveyor.urdf.xacro](./urdf/conveyor.urdf.xacro): ベルトコンベア（足を含まない）のモデル．長さと幅を指定可能
- [conveyor_pillar.urdf.xacro](./urdf/conveyor_pillar.urdf.xacro): 左右一本ずつ一組みのコンベアの足のモデル．コンベア原点からの距離，左右の足の間隔および足の高さを指定可能
- [part_box.urdf.xacro](./urdf/parts_box.urdf.xacro): モーター回収箱のモデル．長さ，幅および高さを指定可能
- [table.urdf.xacro](./urdf/table.urdf.xacro): アームの架台のモデル．長さ，幅および高さを指定可能

個々のモデル(`xxx.urdf.xacro`)は，拡張子が`urdf`である対応ファイル(`xxx.urdf`)によってモデルツリーのルートを付加することにより，以下のコマンドで可視化できる．
```
$ roslauch urdf_tutorial display.launch model:=xxx.urdf
```
ただし，全体環境モデル`nedo3_demo.urdf.xacro`には既にツリーのルートが含まれているので，直接可視化できる．
```
$ roslauch urdf_tutorial display.launch model:=nedo3_demo.urdf.xacro
```
## meshesサブディレクトリ
2種のラックのメッシュモデルを含む．[visualサブディレクトリ](./meshes/visual)には表示用の高解像度モデルが，[collisionサブディレクトリ](./meshes/collision/)には衝突判定用の低解像度モデルがそれぞれ収められている．これらは，URDFファイルから参照される．
