## ROSコード開発→デプロイのベストプラクティス


## 1. **プロジェクト構成（catkinワークスペースの利用）**

ROS1では **catkinワークスペース** を作り、その中にパッケージを配置。

```arduino
~/catkin_ws/
 ├── src/                ← ソースコード（パッケージ群）
 │    ├── my_robot/      ← 例: パッケージ
 │    │    ├── CMakeLists.txt
 │    │    ├── package.xml
 │    │    └── scripts/  ← Pythonノード
 │    │         └── talker.py
 │    └── ...
 ├── build/              ← ビルド生成物
 ├── devel/              ← 実行環境（setup.bash あり）
 └── install/            ← (optional) インストール先

```

* `src/` にすべてのパッケージを置く
* Pythonノードは `scripts/` に置く（実行権限をつける `chmod +x`）
* C++ノードは `src/` にソースを置き、`CMakeLists.txt` にビルドルールを書く


## 2. **コードの作り方**

### (1) Pythonノード（rospy）

* **例: talker.py**

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = "hello world"
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    talker()

```

* `scripts/talker.py` に配置
* 実行権限を付与

```bash
chmod +x scripts/talker.py
```


### (2) C++ノード（roscpp）

* `src/talker.cpp` に記述し、`CMakeLists.txt` にビルドターゲットを追加
* `catkin_make` でビルドすると `devel/lib/<package_name>/talker` にバイナリが生成


## 3. **ビルド & 実行**

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash   # 環境を有効化
rosrun my_robot talker.py
```

__開発サイクル：__

1. コードを編集
2. `catkin_make` （Pythonだけなら不要な場合もある）
3. `source devel/setup.bash`
4. `rosrun` / `roslaunch` で実行


## 4. **launchファイルでのデプロイ**

実際のロボットや複雑なシステムでは **`roslaunch`** でまとめてノードを立ち上げます。

例: `launch/talker_listener.launch`

```xml
<launch>
  <node pkg="my_robot" type="talker.py" name="talker" output="screen"/>
  <node pkg="my_robot" type="listener.py" name="listener" output="screen"/>
</launch>

```

__実行__

```bash
roslaunch my_robot talker_listener.launch

```

こうすることで

- 複数ノードの同時起動
- パラメータの設定
- ログや出力管理が楽になる


## 5. **パラメータ・設定管理**

* パラメータサーバ（`rosparam`）に設定値を置く
* `.yaml` ファイルを用意し、`roslaunch` で読み込むのがベスト

例: `config/params.yaml`

```yaml
robot_name: "my_bot"
max_speed: 1.0

```

launchファイルで読み込み：

```xml
<launch>
  <rosparam file="$(find my_robot)/config/params.yaml" command="load"/>
</launch>

```


## 6. **デプロイのベストプラクティス**

* **ローカル開発 → シミュレーション（Gazebo, rviz） → 実機** の流れでテスト
* Gitでパッケージごとに管理（特に `src/<package>` 単位で）
* DockerやWSL2を使って開発環境を統一（依存関係の崩壊を防ぐ）
* 実機には `roslaunch` でまとめてデプロイ
* ロボット上で `roscore` を立て、リモートPCからノードを追加起動するのが一般的


# 結論

1. catkinワークスペースを作成し、パッケージ単位で管理
2. pythonノードはscripts、C++はsrc
3. catkin_make + rosrun/roslaunchが基本サイクル
4. パラメータはyaml + rosparamで管理
5. デプロイはluanchで複数ノードを一括起動
6. 実機ではroscoreを一つのみ、他はリモートから接続



# PC上でのデプロイ


なるほど 👍 PC上の **シミュレーション環境のみ** を前提にするなら、実機デプロイの大変な部分を省けるので開発はかなり楽になります。

シミュレーション環境前提のベストプラクティスをまとめると👇

---

# 🖥️ PC上でのROSシミュレーション運用ベストプラクティス

## 1. **ワークスペースの構築**

* まずは `catkin_ws` を作り、その中に自分のパッケージを入れる
* 例：
  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin_make
  source devel/setup.bash
  ```

👉 実機がない場合でも **ワークスペースをGitで管理** しておくと良いです

---

## 2. **ノード開発の流れ**

* Pythonノードなら `scripts/` に配置（実行権限を忘れずに）
* C++ノードなら `src/` に配置して `CMakeLists.txt` に追記

👉 Pythonで始める方が圧倒的に早く試せます

👉 シミュレーション用途ならまずは **`rospy`** をおすすめ

---

## 3. **launchファイルで統合**

シミュレーションでは「Gazebo」「rviz」「自作ノード」をまとめて起動することが多いので、 **launchファイル必須** 。

例：`launch/sim.launch`

```xml
<launch>
  <!-- Gazebo起動 -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>

  <!-- 自作ノード -->
  <node pkg="my_robot" type="talker.py" name="talker" output="screen"/>
</launch>
```

実行：

```bash
roslaunch my_robot sim.launch
```

---

## 4. **パラメータ管理**

シミュレーションでは「センサー範囲」「ロボットサイズ」「制御ゲイン」などを調整するので、`yaml` にまとめておくのがベスト

例：`config/sim_params.yaml`

```yaml
lidar_range: 10.0
max_speed: 0.5
```

launchファイルで読み込む：

```xml
<rosparam file="$(find my_robot)/config/sim_params.yaml" command="load"/>
```

---

## 5. **可視化・デバッグ**

* **rviz** : センサーや座標系の確認
* **rqt_graph** : ノード・トピックのつながり確認
* **rosbag** : 実行ログを記録して再生可能

👉 シミュレーションのみなら `rviz` と `rosbag` が特に強力

---

## 6. **デプロイ（PC内での実行）**

シミュレーション運用では **「実機へのコピー」は不要** です。

流れは：

1. `catkin_make`
2. `source devel/setup.bash`
3. `roslaunch` でまとめて起動

👉 **Docker + VSCode Remote** を組み合わせると、複数PCでも環境を再現しやすいです

---

# ✅ まとめ（シミュレーション前提）

* **catkinワークスペース + Git** でコード管理
* ノードはPython中心（素早く試せる）
* **launchファイル** でGazebo/rviz/ノードを一括起動
* **yaml** でパラメータを管理
* デバッグは **rviz, rqt_graph, rosbag** を駆使
* 実機がないので「PCで全部完結」してOK
