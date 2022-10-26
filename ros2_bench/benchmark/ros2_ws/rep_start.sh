cd /benchmark/ros2_ws
colcon build
source install/local_setup.bash
ros2 run pubsub_bench rep
