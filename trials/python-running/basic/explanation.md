## 目的

rosのpythonスクリプトをlinux中で動作させる手順を理解する

## 実行権限の付与

pythonはスクリプト言語なのでこれ自体はビルドは必要ありません。ただpythonファイルを実行するためには実行権限を付ける必要があります。

```shell
roscd py_lecture/scripts/
chmod +x python_talker.py 
chmod +x python_listener.py 
```

![1755299927611](image/explanation/1755299927611.png)

# 実行

各ターミナルごとに実行前に `source ~/catkin_ws/devel/setup.bash`を実行する必要があります。

**1つ目のターミナル**

```shell
roscore
```

**2つ目のターミナル**

```
rosrun py_lecture python_talker.py _content:=data
```

**3つ目のターミナル**

```
rosrun py_lecture python_listener.py
```

```shell
catkin_ws/src/ros_lecture/py_lecture/scripts
```

## 結果：動作しました

talker: send: dataという文字列データをpublish

listener: publishされたデータを受け取って、表示。さらに受け取った日時を表示

3つめのpython_timerの動作が謎・・・

![1755301610778](image/explanation/1755301610778.png)

## エラー

Unable to locate package ros-noetic-desktop-full

When you encounter the error message "unable to locate package ros-noetic-desktop-full" while trying to install ROS Noetic, it indicates that the package is not available in your system's package repository.

Example

**sudo** apt **install** ros-noetic-desktop-full

Reading package lists... Done

Building dependency tree

Reading state information... Done

E: Unable to **locate** package ros-noetic-desktop-full

Steps to Resolve the Issue

1. Update Package Lists

Ensure your package lists are up-to-date by running:

**sudo** apt update

2. Add ROS Repository

Add the ROS repository to your system:

**sudo** sh -c **'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'**

Then, add the ROS key:

**sudo** apt-key adv --keyserver **'hkp://keyserver.ubuntu.com:80'** --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

3. Install ros-noetic-desktop-full

After updating the package lists and adding the ROS repository, install the package:

**sudo** apt update

**sudo** apt **install** ros-noetic-desktop-full

4. Verify Installation

To verify that ROS Noetic is installed correctly, run:

**source** /opt/ros/noetic/setup.**bash**

Add this line to your *.bashrc* file to source it automatically:

**echo** **"source /opt/ros/noetic/setup.bash"** >> ~/.bashrc

**source** ~/.bashrc

By following these steps, you should be able to resolve the "unable to locate package ros-noetic-desktop-full" error and successfully install ROS Noetic on your system
