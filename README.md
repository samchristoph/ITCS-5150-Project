# itcs_5150_project
Project board: https://github.com/users/samchristoph/projects/1/views/2

In order to run the experiment, the following steps need to be followed:
- Create an environment, using the LIMO Pro framework (as used in class).
- Place blocks and other obstacles around said environment, providing the robot with the ability to localize its position.
- Run SLAM toolbox (e.g. command: ros2 launch slam_toolbox online_async_launch.py params_file:=/home/user/ros2_ws/src/itcs_5150_project/config/mapper_params_online_async.yaml)
- Use aStar.py and its class methods to translate the OccupancyGrid object into an aStar.py-usable object, such that the grid values (0 being free, -1 being unknown, and 100 being occupied) are accurately represented.
- Loop in obstacleDetection.py (i.e. "Laser_Scan" class) for the control piece.
- Place start and end points.
- Watch the robot move where you want it to go!