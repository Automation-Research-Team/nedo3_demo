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
$ git clone git@github.com:Automation-Research-Team/aist_robotiq.git
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
$ roslaunch nedo3_bringup nedo3_bringup.launch [sim:=true] [robot_ip:=<IP address>]
```
 - `sim:=true` を指定すればシミュレータが起動し，シミュレーションでアームとグリッパの動作を確認できる．`sim:=false`ならば，実アームおよび実グリッパが起動する．デフォルトは`false`
 - `sim:=false`の場合に`robot_ip:=<IP address>`によってURアームのIP addressを指定する．


`Rviz`も一緒に起動する．アームには予めいくつかの姿勢(`home`, `back`, `mirrored_ready`, etc.)が登録されており，その中の一つをMotionPlanningタブ中の"Goal State:"から選んで"Plan & Execute"ボタンを押せばそこに移動させることができる．あるいは，Interactive Markerで対話的にアームを動かすこともできる．

