  现在这样启动仿真：

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch tb4_rectangle_patrol rectangle_patrol_bringup.launch.py

  明天实机/外部仿真用：

  ros2 launch tb4_rectangle_patrol rectangle_patrol_bringup.launch.py start_sim:=false
  use_sim_time:=false
