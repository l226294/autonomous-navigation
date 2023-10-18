# Lanelet2地图制作

Lanelet2地图是一种矢量地图，其详细介绍可查看[官方文档](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)。

可以根据卫星影像或点云地图绘制Lanelet2地图，下面将分别介绍这两种方法。

## 系统环境要求

地图制作过程只在以下系统环境下进行过验证。

- Ubuntu 20.04
- python 3.8
- ROS2 Foxy

## 使用卫星影像绘制

### 使用到的工具或软件包

- [JOSM](https://josm.openstreetmap.de/)
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)软件包
- [lanelet2_map_provider](???)软件包
- [earth_to_map_tf](???)软件包

### 具体步骤

1、在试验场地中记录两条不平行的GPS轨迹

（1）启动GPS驱动

```
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py 
```

（2）控制小车分别沿着两条互不平行的车道线行驶，记录GPS数据

```
ros2 topic echo /fix >gps.txt
```

（3）将gps.txt文件转为gps.osm文件，gps.osm文件中内容及格式如下图所示。其中，id不能重复，lat是纬度，lon是经度，记录足够多的数据点即可。

![](D:\开源\images\2023-06-06 17-33-51屏幕截图.png)

2、用JOSM软件打开gps.osm文件，对卫星影像进行纠偏

（1）打开gps.osm

![](D:\开源\images\2023-06-06 18-46-49屏幕截图.png)

（2）选中顶部菜单栏中的**影像->Bing航空影像**

<img src="D:\开源\images\2023-06-06 18-51-06屏幕截图.png" alt="2023-06-06 18-51-06屏幕截图" style="zoom:80%;" />

（3）选中顶部菜单栏中的**影像->新偏移**，按下左键拖动影像的车道线与GPS轨迹对齐

![2023-06-06 18-54-41屏幕截图](F:\待处理\2023-06-06 18-54-41屏幕截图.png)

3、绘制Lanelet2地图，并保存

（1）安装Lanelet2插件，详情见[lanelet2_maps](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_maps)

（2）使用左侧菜单栏中**绘制节点**功能，沿车道线绘制两条平行的直线

<img src="F:\待处理\2023-06-09 14-56-27屏幕截图.png" alt="2023-06-09 14-56-27屏幕截图" style="zoom:67%;" />

（3）同时选中这两条直线后，单击顶部菜单栏中**预设组合->Lanelet2->Lanelets->City road**

<img src="D:\开源\images\2023-06-09 15-00-51屏幕截图.png" alt="2023-06-09 15-00-51屏幕截图" style="zoom:80%;" />

（4）在弹出的菜单栏中选择**新建关系**

<img src="D:\开源\images\2023-06-09 14-16-30屏幕截图.png" alt="2023-06-09 14-16-30屏幕截图" style="zoom:80%;" />

（5）设置左车道线的**角色**为**left**，右车道线的**角色**为**right**

![](D:\开源\images\2023-06-09 14-42-16屏幕截图.png)

这样就绘制好了一条Lanelet，重复步骤（2）~（5）可绘制多条Lanelet。更多软件操作请参考[JOSM入门教程](https://learnosm.org/zh_CN/josm/start-josm/)

![2023-06-09 15-03-42屏幕截图](D:\开源\images\2023-06-09 15-03-42屏幕截图.png)

4、对地图进行处理，使地图中元素的id为正值

处理前：

![2023-06-09 15-38-23屏幕截图](D:\开源\images\2023-06-09 15-38-23屏幕截图.png)

处理后：

![2023-06-09 15-39-07屏幕截图](D:\开源\images\2023-06-09 15-39-07屏幕截图.png)

（1）下载[Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)软件包

（2）将刚才绘制好的地图放在与make_ids_positive.py（在Lanelet2-master/lanlet2_python/scripts目录下）文件相同目录下

![](D:\开源\images\2023-06-06 19-17-27屏幕截图.png)

（3）在该目录下右键打开终端，输入指令

```
python3 make_ids_positive.py map_new.osm map_new_modified.osm
```

生成处理好的lanelet2地图**map_new_modified.osm**

![2023-06-06 19-21-39屏幕截图](F:\待处理\2023-06-06 19-21-39屏幕截图.png)

5、对项目软件包进行修改

（1）将地图复制到**lanelet2_map_provider**软件包的**data**目录下

（2）修改**lanelet2_map_provider.launch.py**中的地图文件名

![Snipaste_2023-06-09_15-25-17](D:\开源\images\Snipaste_2023-06-09_15-25-17.png)

（3）修改**earth_to_map.param.yaml**和**lanelet2_map_provider.param.yaml**文件中的经纬度坐标（可以任意设置坐标原点的经纬度，但最好使坐标原点在地图范围内）。

![](D:\开源\images\2023-06-06 19-27-45屏幕截图.png)![2023-06-06 19-28-12屏幕截图](D:\开源\images\2023-06-06 19-28-12屏幕截图.png)

（4）重新编译**earth_to_map_tf**包和**lanelet2_map_provider**包

```
colcon build --packages-select earth_to_map_tf lanelet2_map_provider
```

（5）在rviz2中查看地图

```
ros2 launch map_launch map.launch.xml
```

![2023-06-09 15-15-43屏幕截图](D:\开源\images\2023-06-09 15-15-43屏幕截图.png)

## 使用点云地图绘制

点云地图的制作方法可以参考以下链接：

- https://github.com/TixiaoShan/LIO-SAM/tree/ros2

- https://blog.csdn.net/unlimitedai/article/details/107378759
- http://t.csdn.cn/X0tok

使用点云地图绘制Lanelet2地图的步骤详情可见：

- http://t.csdn.cn/WyQIA

- https://www.bilibili.com/video/BV1Ku411f7xi/?share_source=copy_web

- https://docs.web.auto/en/user-manuals/vector-map-builder/introduction

  