# lidar_camera_calibrator([English README](./README.md))

lidar_camera_calibrator是一个半自动的，高精度的，基于特征的相机与雷达外参的标定工具。一般来说，基于特征的标定方法往往需要一定的人工干预，比如手动选择特征点，使用带有固定图案的标定板等，导致这种方法的自动化程度较低。lidar_camera_calibrator的特点是在需要较少人工干预的前提下，可以半自动的标定出高精度的外参，并提供可交互的可视化Qt界面。

希望它对你有所帮助。

<img src="doc/img/calibrator-sim.gif" style="zoom: 50%;" />

- [lidar_camera_calibrator(English README)](#lidar_camera_calibratorenglish-readme)
  - [依赖](#依赖)
  - [编译](#编译)
  - [测试](#测试)
  - [使用](#使用)
    - [a. 准备数据](#a-准备数据)
    - [b. 标定](#b-标定)
  - [许可证](#许可证)
  - [致谢](#致谢)
## 依赖

- ROS

- Qt 5 (系统自带的也可)

- Eigen 3（3.3版本以上）

- [ceres](https://github.com/ceres-solver/ceres-solver.git)（commit: e51e9b4）

  > ```bash
  > # ceres的依赖
  > $ sudo apt-get install  liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
  > ```

- [g2o](https://github.com/RainerKuemmerle/g2o.git)（tag: 20200410_git）

  > ```bash
  > # g2o的依赖
  > $ sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
  > ```

- [sophus](https://github.com/strasdat/Sophus.git)（commit: 13fb328）

## 编译

```bash
$ mkdir -p ws_calibrator/src && cd ws_calibrator
$ git clone REPOSITORY_GIT_LINK src/lidar_camera_calibrator 
$ catkin_make
```

## 测试

我们在实物和仿真的VLP16与普通RGB相机上进行了测试，并且在`data`文件夹下提供了测试数据。

测试视频1(with AprilTag)：[Youtube链接](https://youtu.be/uew143NcVQw) , [Bilibili链接](https://www.bilibili.com/video/BV1ML4y1s7Rm/)

测试视频2(without ArpilTag)：[Youtube链接](https://youtu.be/0UBl0rEK3ig)，[Bilibili链接](https://www.bilibili.com/video/BV1s34y1y7X9/)

```bash
# 使用带有AprilTag的标定板标定
roslaunch lidar_camera_calibrator calibrate.launch 

# 使用不带有AprilTag的标定板标定
roslaunch lidar_camera_calibrator calibrate.launch input_path:=`rospack find lidar_camera_calibrator`/data/data-hitsz 
```

## 使用

### a. 准备数据

- 标定板要求：**矩形板**（越大越好，建议在1m*1m左右，尽量是长方形，而不是梯形，标定板尽量是平面，这直接影响标定的精度）

  > 如果标定板上有类似`data/data-sim`中的整张AprilTag码的话，就可以自动地提取图像特征

- 标定操作要求：

  - 在标定过程中标定板要倾斜放置**同时保证左边点和右边点不在一个水平面上！！** **[注意]：4个角点中的任意两个角点都不能在同一水平高度**

  - 在不同位置获取对应的图像和点云数据，尽可能在四组以上，并且远近位置都包括在内，参考下图

    <img src="doc/img/demo_data.png" alt="demo_data" style="zoom: 50%;" />

- 制作标定数据

  我们提供了获取同步数据的脚本来得到时间软同步的雷达和相机数据

  修改`get_sync_data.launch`中**图像和点云的话题名称和存储路径**，然后在终端运行以下命令，通过rviz可视化图像与点云，**在终端**摁空格键来保存当前的同步数据

  ```bash
  $ roslaunch lidar_camera_calibrator get_sync_data.launch
  ```

- 准备config.json

  参考`data/data-sim`的格式，修改config.json中对应的参数

  >config.json参数说明
  >
  >```yaml
  >"cam": #相机内参
  >"lidar_pose":"descend" #ascend表示lidar相对于相机是倒放的，"descend"表示lidar相对于相机是正放的。
  >"pc": 
  >        "PerspectiveParams": # pcl_viewer视角，不需要修改
  >        "filter": # 下面参数设定了一个点云的环形区域，区域内的点会被保留
  >             "angle_start":  # 环形区域的起始角度，在界面中可以实时修改
  >             "angle_size":   # 环形区域的角度，在界面中可以实时修改
  >             "min_distance": # 环形区域的内径，在界面中可以实时修改
  >             "max_distance": # 环形区域的外径，在界面中可以实时修改
  >             "ceil_gap":     # 环形区域距离点云最高处的距离，在界面中可以实时修改
  >             "floor_gap":    # 环形区域距离点云最低处的距离，在界面中可以实时修改
  >             "min_distance_threshold": # min_distance取值的最大阈值
  >             "max_distance_threshold": # max_distance取值的最大阈值
  >             "max_ceil_gap": # ceil_gap取值的最大阈值
  >             "max_floor_gap": # floor_gap取值的最大阈值
  >        "plane":  # 点云提取平面时的参数，基本不需要修改
  >"size": # 使用的几何标定板的边数,目前程序只适用于矩形标定板
  >"tag_size": # 矩形标定板的长宽高
  >"tf": # 最后标定的外参结果，需要通过save result来保存
  >```

  **这里重点是要在标定前设置好标定板大小```tag_size```，其他参数都可以在运行界面在线调整**

### b. 标定

1. 修改launch文件中input_path为准备好的标定数据的绝对路径，然后参考上面的教程进行标定

   ```bash
   $ roslaunch lidar_camera_calibrator calibrate.launch
   ```

2. 调整pointcloud control 里面的参数，直到右面出现合适的标定板点云平面，然后点击`extract`。如果在点云中提取到正确的特征，就点`next pose`，否则就`skip pose` 

3. 如果标定板使用apriltag，image control部分会自动检测角点，如果检测错误或者标定板没有apriltag，需要点击 `start selection`然后在图像中**顺序**选择四个角点，最后点击`finish selection`。**[注意]: 要确保图像中的序号```1234```要与点云中的```1234```对应，并且标定板出现在相机和lidar的fov中。**

4. 重复步骤2和3，直到每组数据都完成选择，最后点击`calibrate`，在终端可以看到外参的输出。通过`next pose`和`previous pose`可以看到标定效果，并且可以`save result`保存外参结果到config.json。


## 许可证

lidar_camera_calibrator 遵循 [GPL-3.0 许可证](./LICENSE)。

## 致谢

该项目的图像特征提取部分参考了[AprilTag](https://github.com/AprilRobotics/apriltag)，点云特征提取参考了[Jiunn-Kai Huang的工作](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration)，Qt界面以及框架部分参考了RAM实验室的[plycal](https://github.com/ram-lab/plycal)，非常感谢他们的工作。

