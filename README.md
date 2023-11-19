# px4_gz

px4 在 ros2-gazebo 下的仿真配置与相关实现

## 基本配置

- px4 >= v1.14.0
- gz garden (注意区分 gazebo classic 和 gazebo，这俩不是一个东西)
- ros2 humble

### 可视化相关
- [yalantinglibs](https://github.com/alibaba/yalantinglibs.git)
- [ros_gz](https://github.com/gazebosim/ros_gz.git)

### PX4 相关
- [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent.git)
- [px4_msgs](https://github.com/PX4/px4_msgs.git)

### 编译方法

编译方法更常规的 colcon 工作空间编译基本一致，只是有了 `ros_gz` 需要配置一个环境变量才行

```sehll
GZ_VERSION=garden colcon build --symlink-install
```

## 文件结构

- `description` : 模型文件 urdf 或者 sdf
- `gazebo` : gazebo 相关的代码和配置，一般都是些插件什么的
- `application` : ros2 相关的代码和配置
- `bringup` : launch 文件与一些小工具


## 启动方法

在启动仿真之前，需要先编译出 px4_sitl 的固件，具体方法可参见 px4 v1.14.0 的官方文档

然后修改 `px4_gz_bringup/config/launch_px4.sh` 中 px4 可执行文件的路径

完成上面两步后，可以按照下面的启动仿真

```shell
export GZ_SIM_RESOURCE_PATH=<path_to_px4_source>/Tools/simulation/gz/models:<path_to_px4_source>/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH
source <path_to_ros2_workspace>/install/setup.bash
ros2 launch px4_gz_bringup px4_gz.launch.py
```