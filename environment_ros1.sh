source /opt/ros/noetic/setup.bash
if [ $# -gt 0 ]; then
	export ROS_MASTER_IP=$1
    echo "ROS_MASTER_IP set to $ROS_MASTER_IP"
    source set_ros_master.sh $ROS_MASTER_IP
else
    source set_ros_master.sh 127.0.0.1
fi

if [ $# -gt 0 ]; then
	export ROS_IP=$2
    echo "ROS_IP set to $ROS_IP"
    source set_ros_ip.sh $ROS_IP
else
    source set_ros_ip.sh 127.0.0.1
fi