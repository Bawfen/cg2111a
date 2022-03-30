# cg2111a

## SETUP

1. `roscore` on laptop
2. Split out 4 screens on laptop (screen A, B, C, D)
3. `source ros_setup.sh` on the raspi. This will set `ROS_IP={laptop IP}` and `ROS_MASTER_URI=http://{laptop IP}:11311`
4. `rosrun joy joy_node` on laptop in terminal B
5. `lidar` command which is an alias in the raspi bashrc for `roslaunch rplidar_ros rplidar.launch`
6. `./Desktop/cg2111a_test/w9s2/alex-pi` to start the Alex executable
7. `hector` command on laptop on terminal D