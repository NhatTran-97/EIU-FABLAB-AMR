## OLE Lidar Driver of Ros2 humble ## 
## 详细指导文档分解 ##
#### 1.新建文件夹 ros2_ws
```
mkdir ros2_ws
```
#### 2.将源码解压到 ros2_ws 下, ros2_ws
```
cd ros2_ws
tar xvf oleros2.tar.xz
```

即，ros2_ws下出现src文件夹以及readme.md指导文档
ros2_ws/src
	readme.md
#### 3.进入ros2_ws，安装依赖组件
```
sudo apt-get install -y ros-humble-diagnostic-updater
sudo apt-get install -y python3-colcon-common-extensions
sudo apt-get install -y libpcap-dev
```
#### 4.ros2 环境配置

```
source /opt/ros/humble/setup.bash
```

#### 5.执行编译

```
colcon build
```

#### 6.配置执行环境

```
. install/setup.bash
```

#### 7.执行3d ,参数见 param/ole3dv2.yaml

```
ros2 launch install/ros2_lidar/share/ros2_lidar/launch/ole3dv2_launch.py
```

#### 8.执行2d ,参数见 param/ole2dv2.yaml

```
ros2 launch install/ros2_lidar/share/ros2_lidar/launch/ole2dv2_launch.py
```

#### 9.新建终端启动ros2 环境配置 后执行rviz2

```
cd ros2_ws
source /opt/ros/humble/setup.bash
rviz2 -f laser_data_frame
```
### 输出Topic
```
ros2 topic echo /scan
```
#### 10. 3d 时，rviz2 操作

##### add --> by topic --> pointcloud2

##### map->laser_data_frame                     可以看见点云，

#### 11. 2d 时，rviz2 操作

##### add --> by topic --> laserscan

##### map->laser_data_frame                     可以看见点云，

#### 12.关于frame和tf2

 目前有3个， laser_data_frame 对应ros坐标系，sensor_data_frame 对应雷达系，旋转矩阵在yaml中，要校验，imu_data_frame 目前保留，
