**Log 20210820**

完成了添加障碍物的代码，接下会将继续封装库函数。下周将开始视觉部分的联动以及任务的逻辑流程。



**Log 20210822**

解决python下的cv_bridge库问题，构建比赛程序图像调用部分。



**Log 20210823**

1. **Moveit控制：**根据现有的障碍物环境，将所有障碍物添加进了plan_scene中，后续如果有需要可以再添加。 

**TODO:**

对控制库的进一步封装(陈凯)

![img](https://cdn.nlark.com/yuque/0/2021/png/12407527/1629734227021-6e3d023f-d365-46a9-a575-a2e424c39656.png)

2. **图像部分：**调试好了图像处理的通讯部分，其中包括识别结果的返回。

**TODO**：

1. 1. 对识别结果的解析(郭嘉欣)
   2. 采集数据集(侯睿明，刘晟)

1. 1. 数据集的增强，例如：图像的旋转，反转，亮度变化等(段庆玲辅导侯睿明，刘晟)
   2. 对物体识别的进一步网络训练(陈凯)

1. 1. 外参的标定以及空间坐标转换(陈凯，郭嘉欣)

3. **夹爪：**完成了夹爪的设计工作。

**TODO：**

  a. follow夹爪(吸盘)的制作事宜(唐义丰)

  b. 吸盘的控制问题，包括驱动以及python控制接口。(唐义丰)

  c. 吸盘的模型文件，例如urdf，srdf等格式的制作，用于moveit的规划场景中避障。(唐义丰，陈凯)

1. **抓取逻辑**：**TODO(暂未开始)**

1. **剩余道具的购买**，例如糖包，量杯等有待补充。(孙喜龙，郭嘉欣)

 

**Log 20210825**

1. **图像部分：**完成了kinect的外参标定，对eye_on_hand的相关方法进行预研。对识别结果进行了解析，使用icp方法完成了刀叉角度的判断。对新添置的道具采集了新的数据集，并完成了数据集增强的方法，增强后的效果有待进一步测试。

   **TODO:**

   1. 网络训练的调参优化(段庆玲)

   2. 物体识别的进一步训练及效果验证(陈凯)
   3. 空间坐标的转换以及精度的确认(郭嘉欣)

2. **夹爪问题：**夹爪问题有待进一步联系瑞森可方进行设计，夹爪的设计与制作还要加快。在新的夹爪没有安装之前，先用自带夹爪进行抓取任务。

3. **抓取任务**等待夹爪安置完成后开展。

4. 剩余道具的购买，目前还差量杯，咖啡壶等。

   **另感谢刘晟、侯睿明同学今晚对打标签工作的支持。**

### 识别标签

识别种类的划分，英文为识别的类别结果，中文为打标签时进行区分。

```
1  - saladfork  小叉子
2  - dinnerfork 大叉子
3  - soupspoon  大勺子
4  - knife      刀
5  - tablespoon 小勺子
6  - waterglass 蓝色杯子
7  - wineglass  小高脚杯
8  - bowl       小碗
9  - tweezer    镊子
10 - cup        咖啡杯
11 - saucer     茶托
12 - bucket     冰桶
13 - saladplate 沙拉盘
14 - plate      大盘子
15 - sugar      糖包
```



### 仿真下使用方法

在`.bashrc`中添加了以下代码：

```
alias load_env='source /home/znfs/project_ws/intera/activate.sh'
```

这样在执行`load_env`时，执行`activate`脚本文件。将自动加载好整个比赛的实验环境。



**注意：**需要安装pyrobot

##### 启动gazebo

```
roslaunch sawyer_gazebo sawyer_world.launch electric_gripper:=true
```

##### 启动action_server

```
rosrun intera_interface joint_trajectory_action_server.py
```

##### 启动Moveit

```
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true

```

`**src/pub_collasion.cpp**`
该文件用来发布桌子的障碍物信息，避免在规划时打到桌子。

`**launch/robot_server.launch**`
该launch文件用来启动`kinematics.py`和`moveit_bridge.py`，这两个文件是用来启动pyrobot库所需要的服务的。



#### 加载障碍物

```
# 预添加了桌子障碍物，将该障碍物信息添加到规划场景中去
rosrun iros2021 pub_collasion
```



### 驱动安装

#### Kinect驱动安装

```
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libturbojpeg0-dev
sudo apt-get install libglfw3-dev
sudo apt-get install beignet-dev
sudo apt-get install libopenni2-dev
```

在编译之前我将`CMakeLists.txt`中的39行设置为了OFF，这里**不**选择用cuda来启动相机。

```
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

# for test
./bin/Protonect
```



#### 安装iai_kinect（ros下的kinect启动方式）

```
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
# 到src的上层目录，编译
catkin_make -DCMAKE_BUILD_TYPE="Release"
```

##### 注：在比赛中还使用了realsense，可以参考官网安装方法



### 编译方法

#### 对cv_bridge的单独编译后再全局编译（解决cv_bridge的版本问题）

```
# 对单个包编译方法# 方法1catkin_make -DCATKIN_WHITELIST_PACKAGES="cv_bridge" -DPYTHON_EXECUTABLE=/home/znfs/anaconda3/envs/detectron/bin/python3 # step 1# 恢复对所有包编译catkin_make -DCATKIN_WHITELIST_PACKAGES="" # step 2## 注意顺序，我们可以首先编译cv_bridge，这也是本方法中首要解决的问题，不然不能很好的解决图像转换问题## 编译完cv_bridge之后，对所有包进行编译。# 方法2catkin_make --only-pkg-with-deps cv_bridge -DPYTHON_EXECUTABLE=/home/znfs/anaconda3/envs/detectron/bin/python3
```



### 自定义消息格式

#### message格式

```
float32[] position    # 记录位置float32[] orientation # 记录方向
```

#### service格式

```
# TODO 有待进一步扩充信息内容string camerauint8 method---iros2021/ImageProcMsg[] objectint8[] classify
```



### 相机参数

#### Kinect中的参数参考

```
<arg name="base_name"         default="kinect2"/>
<arg name="sensor"            default=""/>
<arg name="publish_tf"        default="false"/>
<arg name="base_name_tf"      default="$(arg base_name)"/>
<arg name="fps_limit"         default="-1.0"/>
<arg name="calib_path"        default="$(find kinect2_bridge)/data/"/>
<arg name="use_png"           default="false"/>
<arg name="jpeg_quality"      default="90"/>
<arg name="png_level"         default="1"/>
<arg name="depth_method"      default="default"/>
<arg name="depth_device"      default="-1"/>
<arg name="reg_method"        default="default"/>
<arg name="reg_device"        default="-1"/>
<arg name="max_depth"         default="12.0"/>
<arg name="min_depth"         default="0.1"/>
<arg name="queue_size"        default="5"/>
<arg name="bilateral_filter"  default="true"/>
<arg name="edge_aware_filter" default="true"/>
<arg name="worker_threads"    default="4"/>
<arg name="machine"           default="localhost"/>
<arg name="nodelet_manager"   default="$(arg base_name)"/>
<arg name="start_manager"     default="true"/>
<arg name="use_machine"       default="true"/>
<arg name="respawn"           default="true"/>
<arg name="use_nodelet"       default="true"/>
<arg name="output"            default="screen"/>
```



#### Realsense中的参数参考

```
<arg name="camera"              default="camera"/>
<arg name="tf_prefix"           default="$(arg camera)"/>
<arg name="external_manager"    default="false"/>
<arg name="manager"             default="realsense2_camera_manager"/>

<!-- Camera device specific arguments -->
<arg name="serial_no"           default=""/>
<arg name="usb_port_id"         default=""/>
<arg name="device_type"         default=""/>
<arg name="json_file_path"      default=""/>
<arg name="fisheye_width"       default="-1"/>
<arg name="fisheye_height"      default="-1"/>
<arg name="enable_fisheye"      default="false"/>
<arg name="depth_width"         default="-1"/>
<arg name="depth_height"        default="-1"/>
<arg name="enable_depth"        default="true"/>
<arg name="infra_width"         default="-1"/>
<arg name="infra_height"        default="-1"/>
<arg name="enable_infra1"       default="false"/>
<arg name="enable_infra2"       default="false"/>
<arg name="color_width"         default="-1"/>
<arg name="color_height"        default="-1"/>
<arg name="enable_color"        default="true"/>
<arg name="fisheye_fps"         default="-1"/>
<arg name="depth_fps"           default="-1"/>
<arg name="infra_fps"           default="-1"/>
<arg name="color_fps"           default="-1"/>
<arg name="gyro_fps"            default="-1"/>
<arg name="accel_fps"           default="-1"/>
<arg name="enable_gyro"         default="false"/>
<arg name="enable_accel"        default="false"/>
<arg name="enable_pointcloud"   default="false"/>
<arg name="enable_sync"         default="true"/>
<arg name="align_depth"         default="true"/>
<arg name="filters"             default=""/>
<arg name="publish_tf"          default="true"/>
<arg name="tf_publish_rate"     default="0"/> <!-- 0 - static transform -->

<!-- rgbd_launch specific arguments -->
<!-- Arguments for remapping all device namespaces -->
<arg name="rgb"                             default="color" />
<arg name="ir"                              default="infra1" />
<arg name="depth"                           default="depth" />
<arg name="depth_registered_pub"            default="depth_registered" />
<arg name="depth_registered"                default="depth_registered" unless="$(arg align_depth)" />
<arg name="depth_registered"                default="aligned_depth_to_color" if="$(arg align_depth)" />
<arg name="depth_registered_filtered"       default="$(arg depth_registered)" />
<arg name="projector"                       default="projector" />

<!-- Disable bond topics by default -->
<arg name="bond"                            default="false" />
<arg name="respawn"                         default="$(arg bond)" />

<!-- Processing Modules -->
<arg name="rgb_processing"                  default="true"/>
<arg name="debayer_processing"              default="false" />
<arg name="ir_processing"                   default="false"/>
<arg name="depth_processing"                default="false"/>
<arg name="depth_registered_processing"     default="true"/>
<arg name="disparity_processing"            default="false"/>
<arg name="disparity_registered_processing" default="false"/>
<arg name="hw_registered_processing"        default="$(arg align_depth)" />
<arg name="sw_registered_processing"        default="true" unless="$(arg align_depth)" />
<arg name="sw_registered_processing"        default="false" if="$(arg align_depth)" />
```
