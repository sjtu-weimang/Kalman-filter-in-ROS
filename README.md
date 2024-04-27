使用PID控制和卡尔曼滤波在ROS中实现了一个自动送餐系统。

启动方法：
将工程文件夹解压缩到 ~/catkin_ws/src/ 目录下,且包名称为cylinder_robot

（~/catkin_ws为自己之前创建的工作空间）

运行 cd ~/catkin_ws/src/cylinder_robot/script

运行 chmod +x * ，该操作是为当前目录下所有文件赋运行权限

打开终端，依次执行以下操作：
- cd ~/catkin_ws/
- source ./devel/setup/bash
- roslaunch cylinder_robot runCylinder.launch

