一：仿真功能使用

1.将该功能包放在工作空间XXX_ws的src下

2.修改~XXX_ws/src/mpc_track/CMakeLists.txt中add_library()中的库函数的绝对路径。

3.catkin_make

4.将scripts文件夹下的mpc_optimizer.py设置为可执行文件（属性里面）

5.配置路径点相关的参数(path_planning.launch中)，控制器相关参数(MPC_track.launch中)

6.roslaunch mpc_track MPC_track.launch

7.roslaunch mpc_track path_planning.launch

二：实车运行使用

1.将该功能包放在工作空间的src下

2.修改mpc_track/CMakeLists.txt中add_library()中的库函数的绝对路径。

3.catkin_make

4.将scripts文件夹下的mpc_optimizer.py设置为可执行文件（属性里面）

5.打开机器人底盘、雷达，保证能接收到雷达数据
   可以配置功能包中的launch/turn_on_base_and_ladiar.launch文件来同时启动地盘雷达

6.开启机器人定位功能，保证有map到base_link(或base_footprint)的tf转换，
   如果有cartographer的话，可以运行如下命令：roslaunch mpc_track carto_robot_localization.launch

7.进行机器人定位工作，并前往路径起始点附近，别放在超过路径起始点一米以外，跟踪效果会变差

8.配置路径点相关的参数(path_planning.launch中)，控制器相关参数(MPC_track_world.launch中)，监听的目标坐标系名称(如base_link/base_footprint,于MPC_track_world.launch中)

9.roslaunch mpc_track MPC_track_world.launch

10.roslaunch mpc_track path_planning.launch

11.rviz需要自己进行修正，配置话题接口
