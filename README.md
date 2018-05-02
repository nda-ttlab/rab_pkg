# rab_pkg
## 構成
Ether-TOP,TOP-URGを使用。
モータドライバはT-FrogのTF-2MD3-R6。
ubuntu14.04 + indigo + OpenCV2.4.9 で動作確認済み。
ubuntu16.04 + kineticだとgazeboが動かない。

## インストール方法
### 1. workspaceの作成、リポジトリのclone
```bash
$ mkdir -p {your_workspace_name}/src
$ cd {your_workspace_name}/src
$ catkin_init_workspace
$ git clone https://github.com/https://github.com/nda-ttlab/rab_pkg.git
```

### 2. `wstool`コマンドによるパッケージ(apt-getできないもの)のダウンロード
```bash
$ cd {your_workspace_name}
$ wstool init src
$wstool merge -t src src/rab_pkg/dependencies.rosinstall
```

### 3. `rosdep`コマンドによるパッケージ(apt-getできるもの)のダウンロード
```bash
$ cd {your_workspace_name}
$ rosdep install -i -r -y --from-paths src --ignore-src
```
rosdepコマンドは各パッケージフォルダにあるpackage.xmlのdependを読み込み、足りていないものをインストールすることができる。

### 4. `catkin_make`(コンパイル)とパス通し
```bash
$ cd {your_workspace_name}
$ catkin_make
$ source devel/setup.bash
```
`source
devel/setup.bash`はターミナルを立ち上げる(タブでも)度に実行する必要がある。

## 地図の作り方
### ジョイスティック操作によるコースの走行、ログ取り(コマンドはそれぞれのターミナルで)
{your_workspace_name}の一番上にいるとしてコマンド実行(続けて書いてあるコマンドは同じターミナルで)
```bash
$ cd src/rab_pkg/nishidalab_ypspur_driver/config/
$ sh nishidlab_ypspur_starter.sh

$ roslaunch nishidalab_ypspur_driver nishidalab_ypspur.launch

$ roslaunch rab_bringup multi_urg_merge.launch

$ roslaunch rab_bringup 3d_sensor.launch

$ roslaunch rab_bringup joy_control.launch

$ roslaunch rab_bringup avoidance_joy.launch

$ cd log/
$ rosbag record -a
```





