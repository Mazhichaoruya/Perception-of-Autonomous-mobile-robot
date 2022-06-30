source ~/ws_livox/devel/setup.bash
sleep 5
roslaunch cams_recorder video_record.launch &
roslaunch livox_ros_driver livox_lidar_msg.launch &
sleep 10
rostopic list &

echo "start recorder success!"
rosbag record /livox/lidar /livox/imu /image_raw0

exit 0

