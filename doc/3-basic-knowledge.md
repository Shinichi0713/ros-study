# roscoreの役割


ROS の `roscore` は **ROS システムの中枢サーバ** で、ノード同士が通信するための「司令塔」のような役割を果たします。

---

## 🔑 roscore の役割

1. **ROS Master（マスターノード）の起動**

   * ROS における「名前解決サーバ」
   * 各ノードが起動するとき、まず **ROS Master** に「自分はこういう名前で、こういうトピックを発行/購読するよ」と登録します
   * これによって、ノード同士が直接 IP/ポートを知らなくても通信可能になります

   👉 例:

   * ノードA「`/chatter` を publish するよ」
   * ノードB「`/chatter` を subscribe したい」
   * Master「じゃあノードAとBをつなげるね」

---

2. **Parameter Server（パラメータサーバ）**
   * `rosparam` で設定する値（例: ロボットのセンサー閾値、座標系の設定など）を保持
   * ノードはこのサーバを通じてパラメータを読み書きできる

---

3. **rosout の起動**
   * `ROS_INFO`, `ROS_WARN`, `ROS_ERROR` などのログを一括管理する仕組み
   * 全ノードのログを `/rosout` というトピックに集約する

---

## 🚗 たとえ話

* `roscore` = 交通整理員 + 住所録 + 公共掲示板
* ノード = 車
* トピック = 道路

車（ノード）は自分だけでは「どこに行けばいいか」分からないので、まず交通整理員（roscore / ROS Master）に「自分はここにいる、誰とつながりたい」と報告します。整理員は「じゃあこの道を通ってあの車とつながりなさい」と教えてくれるわけです。

---

## もし `roscore` を起動しなかったら？

* ノードは名前解決ができず、トピックを publish/subscribe しても相手を見つけられない
* `rosrun` や `roslaunch` でノードを起動しても「ROS master が見つかりません」というエラーになります

---

✅ まとめると：

`roscore` は **ROS Master + Parameter Server + rosout** をまとめて起動するコマンドであり、ROS ノード同士をつなぐための「必須の司令塔」です。



# source ~/catkin_ws/devel/setup.bash

とても重要なところに目をつけましたね 👍

---

## ✅ `source ~/catkin_ws/devel/setup.bash` の役割

これは **自分のシェル環境に ROS ワークスペースを認識させるための設定** です。

具体的には以下を行います：

---

### 1. **環境変数 `ROS_PACKAGE_PATH` にワークスペースを追加**

* ROS はパッケージを探すとき、`ROS_PACKAGE_PATH` を参照します。
* `source` することで、`~/catkin_ws/src` が検索パスに追加されます。
* これにより `rospack find <package>` や `roscd <package>` が自作パッケージも認識できるようになります。

---

### 2. **環境変数 `PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH` の更新**

* `~/catkin_ws/devel/lib`, `~/catkin_ws/devel/lib/python3/dist-packages` などが環境変数に追加されます。
* これにより、ビルドした自作ノードや Python モジュールをシステムが見つけられるようになります。

---

### 3. **ROS のオーバーレイを設定**

* ROS には「 **オーバーレイ** 」という仕組みがあり、下層の ROS インストール（例: `/opt/ros/noetic`）に対して、自分のワークスペースを上書きできるようになっています。
* `devel/setup.bash` を読み込むと：
  * まず `/opt/ros/noetic` のパッケージが使える
  * さらに `~/catkin_ws/src` の自作パッケージが優先される
* これによって「ROS本体 + 自作パッケージ」が同時に使えるようになるわけです。

---

## 🚗 たとえ話

ROS の世界を「地図」と考えると：

* `/opt/ros/noetic/setup.bash` → 国が提供する公式の地図
* `~/catkin_ws/devel/setup.bash` → あなたが書き足した町や道路（自作パッケージ）

`source` しないと、自作の町が地図に載らないので、車（ノード）は存在を認識できません。

---

## ✅ 実務での使い方

* ワークスペースをビルドした後は必ず：
  ```bash
  source ~/catkin_ws/devel/setup.bash
  ```
* 毎回打つのが面倒なら `~/.bashrc` に追加しておくのが一般的です：
  ```bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  ```

---

👉 確認ですが、あなたは今 `rosrun` でノードが「見つからない」状態を解決したいですか？

それなら、この `source` ができているかどうかが一番怪しいポイントです。
