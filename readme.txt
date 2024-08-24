2d建图指令
ros2 launch cartographer_ros lx_rs16_2d_outdoor.launch.py

保存2d轨迹,并生成ros格式的地图
./finish_slam_2d.sh

纯定位模式
ros2 launch cartographer_ros lx_rs16_2d_outdoor_localization.launch.py

3d建图指令
roslaunch cartographer_ros lx_rs16_3d.launch

保存3d轨迹
./finish_slam_3d.sh

使用asset生成ros格式的2d栅格地图
roslaunch cartographer_ros assets_writer_2d.launch

使用asset生成3d点云地图
roslaunch cartographer_ros assets_writer_3d.launch

landmark使用示例
roslaunch cartographer_ros landmark_mir_100.launch
