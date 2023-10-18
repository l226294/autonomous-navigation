# ROS-BRIDGE

为了能够通过由pygame创建的窗口控制Carla仿真器中的场景，对Carla官方提供的ros-bridge代码进行了一些修改。

<img src="..\..\images\Snipaste_2023-07-12_22-47-00.png" alt="Snipaste_2023-07-12_22-47-00" style="zoom:50%;" />

## 功能

按下F键，仿真器中静止的车辆（仅包括由客户端创建的车辆）开始运动；按下V键，仿真器窗口切换为俯视图，并跟踪ego_vehicle，此时不能通过Q/W/E/A/S/D键改变仿真器窗口中的视角，再按下V键，可以通过Q/W/E/A/S/D键改变仿真器窗口中的视角；按下N键，仿真器中创建10辆运动的车辆。

## 修改

- carla_manual_control.py

增加了以下部分代码

<img src="..\..\images\Snipaste_2023-07-12_23-18-47.png" alt="drawing"/>



<img src="..\..\images\Snipaste_2023-07-12_23-21-54.png" alt="drawing"/>



<img src="..\..\images\Snipaste_2023-07-12_23-23-43.png" alt="drawing"/>



<img src="..\..\images\Snipaste_2023-07-12_23-24-25.png" alt="drawing"/>



- bridge_v2.py

用bridge_v2.py代替了bridge.py，bridge_v2.py包含了bridge.py的所有代码，并增加或修改了以下部分代码。

<img src="..\..\images\Snipaste_2023-07-12_23-32-02.png" alt="drawing"/>



<img src="..\..\images\Snipaste_2023-07-12_23-33-57.png" alt="drawing"/>



<img src="..\..\images\Snipaste_2023-07-12_23-34-54.png" alt="drawing"/>



<img src="..\..\images\Snipaste_2023-07-12_23-35-28.png" alt="drawing"/>



<img src="..\..\images\Snipaste_2023-07-12_23-35-57.png" alt="drawing"/>

