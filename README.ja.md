# ros2_whill_applications

WHILLの[電動モビリティプラットフォーム](https://whill-mrp.notion.site/WHILL-f975baf4015e4eebbb243a7d331efb0a)用ROS 2ドライバである[whill (whill-labs/ros2_whill)](https://github.com/whill-labs/ros2_whill/)を使ったアプリケーション実装例です。

![](./docs/mrp.png)

電動モビリティプラットフォーム以外にセンサ等の準備が必要になるサンプルもあります。

## Examples

### whill_auto_stop

* 動作確認済みディストリビューション：Jazzy
* 追加ハードウェア：あり

TwistメッセージをSubscribeして走行します。その際、周囲の障害物に近づき過ぎたら停止するサンプルです。

ロボット台車にOuster（旧Velodyne）社製 VLP-16を搭載しています。

詳細は[whill_auto_stop/README.md](whill_auto_stop/README.md)を参照ください。

## Usage

### 1. ROS 2のインストール


#### 公式ドキュメントに沿って進める方法

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html を参考に、ROS 2 Jazzyをインストールします。

また、 https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html を参考にビルドツールであるcolconをインストールします。

#### インストールスクリプトを使って進める方法

有志が公開する（非公式の）インストールスクリプトでもインストールできます。  
その場合は下記のコマンドでROS 2 Jazzyをインストールします。

```sh
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
cd ros2_setup_scripts_ubuntu
./ros2-jazzy-desktop-main.sh
```

### 2. WHILLパッケージのインストール

`$HOME/ros2_ws`を使う場合、下記のコマンドでインストールできます。

```sh
# ダウンロード
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO https://github.com/whill-labs/ros2_whill_interfaces.git
git clone -b $ROS_DISTRO https://github.com/whill-labs/ros2_whill.git
git clone https://github.com/whill-labs/ros2_whill_applications.git
# ビルド
cd ~/ros2_ws
colcon build
source install/setup.bash  # zshの場合はsetup.zsh
```

### 3. 各デバイスを接続する

PCとロボット台車、LiDAR等を接続します。

### 4. launchファイルを起動する

whill_auto_stopの場合は以下のコマンドを実行します。

```
ros2 launch whill_auto_stop bringup_launch.py
```

詳細は[whill_auto_stop/README.md](whill_auto_stop/README.md)を参照ください。

## Contact

Email: `mrp.contact[at]whill.inc`

WHILLの電動モビリティプラットフォームについては下記サイトをご覧ください。

[電動モビリティプラットフォーム](https://whill-mrp.notion.site/WHILL-f975baf4015e4eebbb243a7d331efb0a)

## LICENSE

Copyright (c) 2024 WHILL, Inc.

MITライセンスで公開しています。詳細は[LICENSE](./LICENSE)をご参照ください。
