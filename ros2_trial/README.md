# ROS2 trial

r2rを試してみる。
公式: https://github.com/sequenceplanner/r2r

## 使い方

前提条件
- ROS2 Foxy Fitzroyがされていること
- libclangがインストールされていること
   - in Ubuntu: `sudo apt install libclang-dev`

```
. /opt/ros/foxy/setup.sh
cargo build
cargo run --example subscriber
# In other shell
ros2 topic pub /topic std_msgs/msg/String "data: 'Hello, world'"
```