mkdir -p ~/ws_6dof/src
cd ~/ws_6dof/src
git clone https://github.com/qhsong02/6dof_trajectory_planning.git

cd ~/ws_6dof
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
ros2 launch sixdof_rviz_control bringup.launch.py
