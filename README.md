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
