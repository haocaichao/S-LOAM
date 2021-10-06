# S-LOAM

## 1、S-LOAM概述

S-LOAM(Simple LOAM) 是一种简单易学的激光SLAM算法，主要思想来源于LOAM算法系列（LOAM,A-LOAM,LEGO-LOAM）。

S-LOAM利用多种工具库（Eigen,PCL,ROS,Ceres,Gtsam）简化了SLAM程序，整个程序只有几百行代码，十分方便学习与试验分析。

S-LOAM主要包括5个部分，内容如下。

（1）点云索引重建

（2）点云特征提取

（3）里程计计算

（4）里程计因子图优化

（5）里程计闭环优化

S-LOAM的效果如下图所示。

![eloam](./pics/eloam.gif)



## 2、环境依赖

环境配置往往是一件让人头痛的事情，有时会出现版本不兼容，有时会出现下载失败的情况。S-LOAM的运行环境与A-LOAM和LEGO-LOAM相同。主要包含下面5种依赖。

（1）Ubuntu 16.04

（2）ROS Kinetic （里面包含ROS,TF,PCL,Eigen,rviz,rqt等）

（3）Eigen 3.3.7

（4）Gtsam 4.0.0

（5）Ceres 2.0.0



## 3、测试数据

这里提供了3个测试数据（如下），已经放到我的百度网盘中。

百度网盘地址（https://pan.baidu.com/s/1EUNOlCNct71_4_OF0STjuA）。

提取码（dydg）。

（1）kitti_loop_eloam.bag（6G，64线）

（2）kitti_small_eloam.bag（300M，64线）

（3）nsh_indoor_outdoor.bag（700M，16线）

第一个数据是最主要的测试数据，就是上面效果图所用数据，能够满足建图与闭环优化。

第二个数据是从第一个数据中截取的小数据，方便网速不好的同学可以快速下载使用（无闭环）。

第三个数据是LOAM中最常使用的16线数据（无闭环）。



## 4、编译运行

下载S-LOAM工程，并将此文件夹作为ROS功能包，放入自己建立的ROS工程中，编译并运行。

代码地址为（https://github.com/haocaichao/S-LOAM）。

打开一个终端，输入下面两条命令，启动sloam程序。

```
source devel/setup.bash
roslaunch sloam run.launch
```

打开另一个终端，输入下面命令，运行数据。

```
rosbag play kitti_loop_eloam.bag
```



## 5、S-LOAM的优缺点

优点：简单，全面，方便学习与修改。

缺点：点云特征点单一，相关处理过少，在有的数据上运行效果不佳。

S-LOAM作为一个简单激光SLAM框架，对于学习和试验分析来说，它的优点远大于缺点，值得大家关注。

S-LOAM的改进版本正在路上。



## 6、相关网站

关于S-LOAM更多信息，可以关注下面的网站。


网站一：个人csdn博客

S-LOAM 最简单的激光SLAM

https://blog.csdn.net/hccmap/article/details/119984889

运行S-LOAM激光SLAM程序

https://blog.csdn.net/hccmap/article/details/120626920


网站二：这个是个人网站，内容包括python和slam，将在至少5年内持续维护。

http://www.pythonck.com/archives/docs/slam

