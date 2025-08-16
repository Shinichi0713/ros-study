コードは記載されているものを使いまわし



## CmakeListの設定

パッケージ直下に `CmakeLists.txt`がありここにビルド用の情報を書き足します。今回の場合は `CmakeLists.txt`に以下の4行を書き加えるだけです。

```c
project(basic_lecture)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_simple_talker   src/basic_simple_talker.cpp)
add_executable(basic_simple_listener src/basic_simple_listener.cpp)

target_link_libraries(basic_simple_talker
  ${catkin_LIBRARIES}
)
target_link_libraries(basic_simple_listener
  ${catkin_LIBRARIES}
)

```


その後、ビルドします。

```
cd ~/catkin_ws
catkin build

```
