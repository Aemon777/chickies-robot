#launch the IMU publisher
echo "Launching IMU publisher..."
rosrun ros_icm20948 talker.py &
sleep 10
#roslaunch  ros_icm20948 icm20948.launch &
#launch the encoder publisher
echo "Launching encoder publisher..."
rosrun encoders publish_odometry.py &
sleep 10
#launch the LIDAR publisher
#echo "Launching LIDAR publisher..."
#roslaunch rplidar_ros rplidar.launch
#sleep 10
#launch the GPS publisher
#echo "Launching GPS publisher..."
#roslaunch ublox_gps ublox_zed-f9p.launch
#sleep 10
