nedo3_demoパッケージのインストール法と使用法
==================================================

## 概要
本ソフトウェアは，NEDO3プロジェクトのデモシステムを構築するためのROSパッケージコレクションである．依存する他のROSパッケージのうち，ROSの公式リポジトリや一般外部リポジトリに無いもの，あるいは既存のものに手を加えているものは，以下のとおりである．

- [nedo3_demo](https://github.com/Automation-Research-Team/nedo3_demo): 本ソフトウェア
- [universal_robot](https://github.com/Automation-Research-Team/universal_robot): DhaibaWorks上で表示するために，UR5eの可視化用メッシュの形式を[オリジナル](https://github.com/ros-industrial/universal_robot.git)における`Collada-DAE`から`STL`に変更してある
- [aist_robotiq](https://github.com/Automation-Research-Team/aist_robotiq): Robotiq社のハンドを駆動するためのドライバとコントローラ

## インストール手順
まずUbuntu-20.04上に`ros-noetic-desktop-full`をインストールした後，python関係のツールをインストールする．
```bash
$ sudo apt-get install python3-catkin-tools python3-rosdep python3-dev python3-numpy python3-pip python3-setuptools
```
次に，`rosdep`を初期化する．
```bash
$ sudo rosdep init
$ rosdep update
```
`github`から`nedo3_demo`を入手し，`develop`ブランチを取り出す．
```bash
$ cd catkin_ws/src
$ git clone git@github.com:Automation-Research-Team/nedo3_demo.git
$ cd nedo3_demo
$ git checkout develop
```
`github`から`universal_robot`を入手し，`use-stl`ブランチを取り出す．
```bash
$ cd catkin_ws/src
$ git clone git@github.com:Automation-Research-Team/universal_robot.git
$ cd universal_robot
$ git checkout use_stl
```
なお，既にオリジナルの`universal_robot`がインストールされている場合は，それを`catkin_ws`の外に退避して無効化しておくこと．

`github`から`aist_robotiq`を入手し，`develop`ブランチを取り出す．
```bash
$ cd catkin_ws/src
$ git clone git@github.com:Automation-Research-Team/aist_robotiqs.git
$ cd aist_robotiq
$ git checkout develop
```
`nedo3_demo`, `universal_robot`および`aist_robotiq` が必要とするパッケージをインストールする．
```bash
$ cd catkin_ws/src
$ rosdep install -i --from-paths .
```
最後に，ワークスペース全体をコンパイルする．
```bash
$ cd catkin_ws
$ catkin build
```
エラーが出た場合は，必要パッケージがインストールされていないので，適宜手動でインストールする．

## 使用法
### アームとグリッパの動作を確認
以下の手順でアームとグリッパを立ち上げる．
```bash
$ roslaunch toyota2_bringup toyota2_bringup.launch [sim:=true] [config:=higashifuji]
```
 - `sim:=true` を指定すればシミュレータが起動し，シミュレーションでアームの動作を確認できる．ただし，グリッパ動作のシミュレーションはサポートされていない．`sim:=false`ならば，実アームおよび実グリッパが起動する．デフォルトは`false`
 - `config:=higashifuji` を指定すれば，東富士のテスト環境におけるアームとコンベアの配置で起動する．`config:=toyota2`ならば，産総研のテスト環境の配置で起動する．デフォルトは`toyota2`

`Rviz`も一緒に起動する．アームには予めいくつかの姿勢(`home`, `ready`, `mirrored_ready`, etc.)が登録されており，その中の一つをMotionPlanningタブ中の"Goal State:"から選んで"Plan & Execute"ボタンを押せばそこに移動させることができる．あるいは，Interactive Markerで対話的にアームを動かすこともできる．

別のターミナルを開いて以下のコマンドで対話操作プログラムを起動すれば，コマンドラインからアームとグリッパを対話的に制御できる．
```bash
$ roslaunch toyota2_routines interactive.launch
```
"?"コマンドを打てば，簡単なコマンド一覧が表示される．詳しくは直接[ソース](./toyota2_routines/src/toyota2_routines/__init__.py#L57-156)を参照されたい．

### モーター検出をシミュレートしてピッキング動作を確認

まず，以下のコマンドでアーム，グリッパ，`MoveIt`, `object_generator`および`conveyor`を起動する．
```bash
$ roslaunch toyota2_bringup picking.launch object_generator:=true [sim:=true] [config:=higashifuji]
```
 - `object_generator:=true`を指定すれば，`object_generator`と`conveyor`が起動する．`object_generator`は，カメラ実機とYOLOの代わりに，（擬似的な）モーターの検出位置を`conveyor`に供給する役割を果たす
 - `sim:=true` を指定すればシミュレータが起動し，シミュレーションでアームの動作を確認できる．ただし，グリッパ動作のシミュレーションはサポートされていない．`sim:=false`ならば，実アームおよび実グリッパが起動する．デフォルトは`false`
 - `config:=higashifuji` を指定すれば，東富士のテスト環境におけるアームとコンベアの配置で起動する．`config:=toyota2`ならば，産総研のテスト環境の配置で起動する．デフォルトは`toyota2`
 
 `Rviz`も一緒に起動する．また，現在使用されていないが，アームの直交座標サーボサーバも起動する．シミュレータと実機のいずれの場合でも，ピッキング対象となるモーターがランダムに生成され，コンベア上を流れる様子がシミュレートされる．

 次に，別のターミナルを開いて以下のコマンドでピッキング作業用対話操作プログラムを起動する．
```
$ roslaunch toyota2_routines run_picking.launch [config:=higashifuji]
```
 - `config:=higashifuji` を指定すれば，東富士のテスト環境用のパラメータを読み込んで起動する．`config:=toyota2`ならば，産総研のテスト環境用のパラメータで起動する．デフォルトは`toyota2`．`picking.launch`の起動時に与えた設定に合わせること

 以下の２つのコマンドでピッキング動作を実行する．

 - **opick**: コンベアを流れるモーターに追従してpick & place動作を開始する．コマンドを打つとトライ回数の入力を求められ，ここで指定した回数だけ動作が反復される．特に負数（-1など）を指定すると，**ocancel**コマンドでキャンセルされるまで繰り返す
 - **ocancel**: 現在実行中の**opick**コマンドを中断する

**opick**コマンドで実行されるピッキング動作にはアームの制御遅延を補償する機構が組み込まれており，動作の繰り返し毎にモーターとアームの動きのずれが改善される様子が`rqt_plot`に表示される．

### 実画像を用いて検出されたモーターのピッキング
以下の手順に従って，実際のカメラから得た画像を用いてpick & placeを行う．

まず，以下のコマンドでアーム，グリッパ，`MoveIt`を起動する．
```bash
$ roslaunch toyota2_bringup picking.launch [config:=higashifuji]
```
 - `config:=higashifuji` を指定すれば，東富士のテスト環境におけるアームとコンベアの配置で起動する．`config:=toyota2`ならば，産総研のテスト環境の配置で起動する．デフォルトは`toyota2`
 
 `Rviz`も一緒に起動する．また．現在使用されていないが，アームの直交座標サーボサーバも起動する．

 次に，以下のコマンドで画像処理パイプライン（詳細は[toyota2_bbox_tracker](./toyota2_bbox_tracker)パッケージの[README](./toyota2_bbox_tracker/README.md)を参照）を起動する．
 ```bash
$ roslaunch toyota2_bbox_tracker run.launch
```
これにより，カメラから画像が取り込まれ，YOLOによるモーター検出が開始する．

最後に，以下のコマンドでピッキング作業用対話操作プログラムを起動する．
```
$ roslaunch toyota2_routines run_picking.launch [config:=higashifuji]
```
操作法は，前節のモーター検出をシミュレートした場合と同じである．

### 画像処理パイプラインの別ホストへの分離
負荷分散のため，上記２番目のコマンド(`run.launch`)を別のホストで実行し，アーム制御のホスト(`host-c`)と画像処理パイプラインのホスト(`host-v`)を分けることも可能である．そのためには，`host-v:/etc/hosts`に`host-c`のIPアドレスを登録した上で，`host-v:~/.bashrc`に環境変数
```
export ROS_MASTER_URI=http://host-c:11311
```
を設定する．これにより，`host-v`は`host-c`をマスタとするROSネットワークの一員として動作する．

なお，複数のホストから成るROSネットワークを構成する際は，全ホストの時刻を[chorony](https://ja.wikipedia.org/wiki/Chrony)等で同期させておく必要がある．

### 主要なパラメータ設定ファイル
ピッキングを行う上で調整を要する重要なパラーメータは，以下のファイルで定義されている．

- [toyota2_bbox_tracker/config/camera_aravis.yaml](./toyota2_bbox_tracker/config/camera_aravis.yaml): aravis_cameraの設定
- [toyota2_bbox_tracker/config/bbox_tracker.yaml](./toyota2_bbox_tracker/config/bbox_tracker.yaml): bbox_trackerの設定
- [toyota2_conveyor/config/conveyor.yaml](./toyota2_conveyor/config/conveyor.yaml): conveyorの設定
- [toyota2_routines/config/toyota2.yaml](./toyota2_routines/config/toyota2.yaml): picking動作(産総研)の設定
- [toyota2_routines/config/higashifuji.yaml](./toyota2_routines/config/higashifuji.yaml): picking動作(東富士)の設定

個々のパラメータの詳細は，それぞれのパッケージのREADMEを参照のこと．
