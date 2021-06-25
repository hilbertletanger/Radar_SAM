
#### Radar-SAM

**前端ICP 后端SAM 正在开发中**

## 各分支简述

# master分支
	为ICP前端+RS回环检测
# dev分支
	目前主要在这个分支开发，为ICP前端+SC回环检测+添加gps边
# Newman分支
	目前基本废弃，只是做一些尝试性的工作，包括尝试将前端更换为NewmanICP，以及开始局部建图，效果不佳

## 传感器配置和数据结构

* **`imu`** ([sensor_msgs::Imu])

	包括六轴陀螺仪和加速度计，以及从三轴绝对方向角计算而来的 orientation.

* **`gps`** ([sensor_msgs::NavSatFix])

	gps

* **`4D毫米波雷达`** ([sensor_msgs::PointCloud2])
	
	使用4D毫米波雷达覆盖智能驾驶车辆周围空间，本例一共使用5个

#### 依赖和安装
* **`依赖`**

	本项目依赖项和LIO-SAM完全相同，详见如下：
	```
	https://github.com/TixiaoShan/LIO-SAM
	```

* **`安装`**
	```
	mkdir src
	cd src
	git clone https://github.com/hilbertletanger/Radar_SAM.git
	cd ..
	catkin_make
	```

* **`运行`**
	```
	source devel/setup.bash
	roslaunch radar_sam run.launch 
	```


#### 整体介绍  TODO

* **算法** 

* **参数** 

* **注意** 


