# itcs_5150_project
Project board: https://github.com/users/samchristoph/projects/1/views/2

*Setup*
Prerequisites:
  -ROS 2 Humble/Foxy, Python 3.8+, LIMO Pro Framework.
  -Required ROS 2 packages: geometry_msgs, nav_msgs, sensor_msgs, std_msgs, slam_toolbox.
Install the Package:
  cd ~/ros2_ws
  git clone https://github.com/samchristoph/itcs_5150_project.git src/itcs_5150_project
  colcon build --packages-select itcs_5150_project
  source install/setup.bash

In order to run the experiment, the following steps need to be followed:
- Create an environment, using the LIMO Pro framework (as used in class).
- Place blocks and other obstacles around said environment, providing the robot with the ability to localize its position.
- Run SLAM toolbox (e.g. command: ros2 launch slam_toolbox online_async_launch.py params_file:=/home/user/ros2_ws/src/itcs_5150_project/config/mapper_params_online_async.yaml)
- Use aStar.py and its class methods to translate the OccupancyGrid object into an aStar.py-usable object, such that the grid values (0 being free, -1 being unknown, and 100 being occupied) are accurately represented.
- Loop in obstacleDetection.py (i.e. "Laser_Scan" class) for the control piece.
- Place start and end points.
- Watch the robot move where you want it to go!
