## 目的

Visual SlamとROSの関係性について確認する。


## きっかけ

このサイトでLSD-SLAMなるリアルタイムのSLAMのアプローチです。


[tum-vision/lsd_slam: LSD-SLAM](https://github.com/tum-vision/lsd_slam#22-ros-indigo--ubuntu-1404)


## 実際の関係性

**Visual-SLAM（Simultaneous Localization and Mapping, 視覚的SLAM）** と **ROS（Robot Operating System）** の関係は「アルゴリズム」と「実行基盤」という立場で整理すると分かりやすいです。

---

### 1. Visual-SLAMとは？

* **カメラ画像を使って、同時に自己位置推定（Localization）と環境地図作成（Mapping）を行う技術**です。
* 代表的なアルゴリズムやフレームワーク：
  * ORB-SLAM / ORB-SLAM2 / ORB-SLAM3
  * LSD-SLAM
  * RTAB-Map
* ロボットが **GPSの効かない屋内** や **未知環境** で自律移動するための基礎技術。

---

### 2. ROSとは？

* ロボット開発のための  **ミドルウェア（フレームワーク）** 。
* 機能：
  * センサー（カメラ、LiDAR、IMUなど）からデータを取得し、ノードとして処理する
  * 各種アルゴリズムをノード化して通信する
  * ロボットのモーター制御、マップ管理、ナビゲーションなどをモジュールとして組み合わせられる

---

### 3. 両者の関係

* **Visual-SLAMはアルゴリズム、ROSはその実行・統合基盤** 。
* 具体的には：
  * Visual-SLAMのアルゴリズムを **ROSノードとして実装**することで、他のセンサーや制御システムと連携できる。
  * 例：
    * カメラからの画像を `sensor_msgs/Image` で受け取り
    * Visual-SLAMノードで処理し
    * 結果を `nav_msgs/Odometry` や `geometry_msgs/Pose` として出力
    * その情報をROSの **ナビゲーションスタック（move_baseなど）** に渡して経路計画や自律走行を実現

---

### 4. 例：ROSでのVisual-SLAM利用

* **RTAB-Map（rtabmap_rosパッケージ）**

  → RGB-Dカメラやステレオカメラの入力からリアルタイムで地図と自己位置を生成し、ROSのナビゲーションに接続可能。
* **ORB-SLAM2 (ros wrapper付き)**

  → モノカメラ / ステレオ / RGB-DカメラをROSトピックとして受け取り、位置推定をROSメッセージとして配信。

---

### まとめ

* Visual-SLAMは「ロボットの目（自己位置推定と地図作成）」
* ROSは「ロボットの頭脳と神経（アルゴリズム実行・情報通信・統合制御）」
* ROS上でVisual-SLAMを動かすことで、**カメラから得た環境理解をロボット制御・ナビゲーションに直結**できる。

ということで実機にVisual Slamを実装する場合はROSが必須と考えて間違いないと思います。


## 実装例

実装例を **ROS1（Noetic）**と**ROS2（Humble）**でそれぞれ示します。

最短ルートは**RTAB-Map** 、もう少し玄人向けが**ORB-SLAM3**です。

---

### ① RTAB-Map を使う（最短・多機能）

ROS1（Noetic）例：RGB-Dカメラ（RealSenseなど）

1) 依存パッケージ

```bash
sudo apt install ros-noetic-rtabmap-ros ros-noetic-rgbd-launch \
                 ros-noetic-realsense2-camera  # RealSenseの場合
```

2) カメラドライバ起動

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

主要トピック（例）

* 画像: `/camera/color/image_raw`
* 深度: `/camera/aligned_depth_to_color/image_raw`
* カメラ情報: `/camera/color/camera_info`
* IMU（あれば）: `/camera/accel`, `/camera/gyro`

3) RTAB-Mapノード起動

```bash
roslaunch rtabmap_ros rtabmap.launch \
   rtabmap_args:="--delete_db_on_start" \
   rgb_topic:=/camera/color/image_raw \
   depth_topic:=/camera/aligned_depth_to_color/image_raw \
   camera_info_topic:=/camera/color/camera_info \
   frame_id:=base_link \
   approx_sync:=true
```

ポイント

* `--delete_db_on_start` は毎回DBを作り直すオプション（検証向け）。
* `approx_sync:=true` は画像と深度のタイムスタンプを近似同期。
* 出力（例）：`/rtabmap/odom`（自己位置）、`/rtabmap/cloud_map`（点群地図）、`/map`（2D Occupancy）

4) 可視化（RViz）

```bash
rosrun rviz rviz
```

* `TF`, `Odometry (/rtabmap/odom)`, `PointCloud2 (/rtabmap/cloud_map)`, `Map (/map)` を追加。

5) 代替：ステレオカメラ

```bash
roslaunch rtabmap_ros rtabmap.launch \
   stereo:=true \
   left_image_topic:=/stereo/left/image_raw \
   right_image_topic:=/stereo/right/image_raw \
   left_camera_info_topic:=/stereo/left/camera_info \
   right_camera_info_topic:=/stereo/right/camera_info \
   frame_id:=base_link
```

6) rosbagで再生（データセット検証）

```bash
rosbag play dataset.bag --clock -r 0.5
rosparam set use_sim_time true
```

---

ROS2（Humble）例：RGB-Dカメラ

1) 依存パッケージ

```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-realsense2-camera
```

2) 起動（Composable版の例）

```bash
ros2 launch realsense2_camera rs_launch.py align_depth:=true
ros2 launch rtabmap_ros rtabmap.launch.py \
   args:="--delete_db_on_start" \
   rgb_topic:=/camera/color/image_raw \
   depth_topic:=/camera/aligned_depth_to_color/image_raw \
   camera_info_topic:=/camera/color/camera_info \
   frame_id:=base_link \
   approx_sync:=true \
   qos:=2
```

* ROS2はQoSの不一致で落ちることがあるため、`qos`指定が有効（`2` はSensorData相当）。

3) RViz2

```bash
ros2 run rviz2 rviz2
```

---

② ORB-SLAM3 を使う（高精度・軽量：Mono/Mono+IMU/ステレオ/RGB-D）

> 公式は純C++ですが、**ROSラッパ**を使うとROSトピックで入出力できます。

ROS1（Noetic）例：Mono + IMU

1) ビルド

```bash
sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge \
                 ros-noetic-tf ros-noetic-message-filters
# Pangolin, Eigen, OpenCV等を事前に用意
cd ~/catkin_ws/src
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
git clone https://github.com/<wrapper-author>/orbslam3_ros_wrapper.git  # 代表的なラッパを使用
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

2) 例：launch（抜粋）

```xml
<!-- orbslam3_mono_inertial.launch -->
<launch>
  <arg name="voc_file" default="$(find orb_slam3)/Vocabulary/ORBvoc.txt" />
  <arg name="cam_yaml" default="$(find orb_slam3_ros_wrapper)/config/camera.yaml" />
  <arg name="imu_yaml" default="$(find orb_slam3_ros_wrapper)/config/imu.yaml" />
  <arg name="image_topic" default="/camera/image_raw" />
  <arg name="imu_topic"   default="/imu" />
  <arg name="publish_tf"  default="true" />
  <node pkg="orbslam3_ros_wrapper" type="mono_inertial_node" name="orbslam3" output="screen">
    <param name="voc_file" value="$(arg voc_file)"/>
    <param name="camera_yaml" value="$(arg cam_yaml)"/>
    <param name="imu_yaml" value="$(arg imu_yaml)"/>
    <param name="image_topic" value="$(arg image_topic)"/>
    <param name="imu_topic" value="$(arg imu_topic)"/>
    <param name="publish_tf" value="$(arg publish_tf)"/>
    <param name="load_map" value="false"/>
  </node>
</launch>
```

入出力

* 入力: `sensor_msgs/Image`（Mono）、`sensor_msgs/Imu`（IMU）
* 出力: `nav_msgs/Odometry`（推定姿勢）、`/tf`（`map→odom→base_link` など）

3) カメラ＆IMU

* 例：D435iならIMUも取得可能（`/camera/accel/sample`, `/camera/gyro/sample`）→ラッパ側のIMU対応に合わせて変換・同期。

ROS2（Humble）例：ステレオ

* 手順は概ね同じ。`colcon build`、`ament`系、`launch.py`での引数渡しに読み替え。
* QoS設定（`SensorData`）と `image_transport`の一致に注意。

---

③ ナビゲーションとの接続（move_base / Nav2）

* **自己位置**を `/tf`または `/odom`/`/pose`で公開すれば、
  * ROS1：`amcl`を使わずに、SLAM→`/map`→`move_base`で経路計画
  * ROS2：Nav2（`nav2_bt_navigator`など）に `/map`と `/odom`/`/tf`を供給
* RTAB-Mapは2D Occupancyも出せるので、**地図サーバ**不要でそのまま使えます。

---

④ よくあるハマりどころ（チェックリスト）

* **TFツリー** ：`map → odom → base_link → camera_link` が切れていないか
* **タイムスタンプ** ：センサー時刻が未来/過去にズレていないか（`use_sim_time`時は特に）
* **同期** ：ステレオ/IMUは近似同期の許容or厳密同期の設定を見直す
* **レンズ歪み** ：`camera_info`が未整備だと精度が落ちる
* **QoS（ROS2）** ：Publisher/SubscriberのQoS不一致で購読できない

---

⑤ 最小サンプルまとめ（コピペ用）

RTAB-Map（ROS1, RealSense）

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
roslaunch rtabmap_ros rtabmap.launch \
  rtabmap_args:="--delete_db_on_start" \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=base_link approx_sync:=true
```

ORB-SLAM3（ROS1, Mono+IMU）

```bash
roslaunch orbslam3_ros_wrapper orbslam3_mono_inertial.launch \
  image_topic:=/camera/image_raw imu_topic:=/imu publish_tf:=true
```

---

必要なら、あなたのカメラ（型番/ROS1 or ROS2/Mono or Stereo or RGB-D/IMU有無）に合わせて**具体的なlaunchファイルや設定YAML**をこちらで作成します。


## 参考サイト

[【7Days自由研究】Visual SLAMとROSを使えるようになる話 Day-2 - EnsekiTT Blog](https://ensekitt.hatenablog.com/entry/visualslam2)
