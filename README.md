# hairo22_ws
第七回廃炉想像ロボコン(2022)に出場したロボットのROS2のワークスペースです。

この他にもmicro-ROSのセットアップやv4l2_cameraパッケージやjoyパッケージをcloneしています。

|パッケージ|URL|
|---|---|
|micro_ros_setup|https://github.com/micro-ROS/micro_ros_setup|
|v4l2_camera|https://gitlab.com/boldhearts/ros2_v4l2_camera|
|joystick_drivers|https://github.com/ros-drivers/joystick_drivers|

## custom_msgs
本システムで利用したカスタムメッセージを格納するパッケージです。

vectorのサイズが決められていないメッセージ(MultiArray系)はマイコン側で扱うことができなかったことに加えて、一つのメッセージでコマンドを一気に送ったほうが動作が安定したため、カスタムメッセージを利用しています。

## cmd_pub_node/convert_joy2cmd

ジョイスティックの情報を格納するsensor_msgs/Joy型のメッセージではボタン情報やジョイスティックの情報をサイズが決まっていないvectorで管理しています。

このため、MultiArray系と同様にマイコン側でそのまま扱うことができないので、一度このノードでカスタムメッセージに変換しています。

## 反省

micro-ROSで複数のメッセージを扱うと何故か通信ができなくなってしまったため、カスタムメッセージで強引に複数のメッセージをまとめるという手法をとりましたが、もう少しmicro-ROSについて勉強しておけばもう少しマシなシステムになったかと思います。

特に足回りの制御コマンドをTwist型のメッセージで送っていないのは流石によくないなと思っています。Vector3がdouble型の変数を3つも有しているのでマイコンには処理が重いのではないかとビビってしまいました。

次回もJoyからカスタムメッセージのコマンドに変換する必要はありそうですが、足回りと上物のメッセージは別にしたいところです。

また、PC側で大した処理をしていない影響でせっかくのリソースがもったいない気がするので、操縦を楽にするユーザーインターフェースにも力を入れていきたいです。
