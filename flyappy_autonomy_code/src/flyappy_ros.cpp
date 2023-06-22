#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Example of publishing acceleration command to Flyappy
    geometry_msgs::Vector3 acc_cmd;

    // TODO: Create a method in Flyappy class that returns the wanted acceleration in x and y


    flyappy_.set_vx_data(msg->x, msg->y);

    // acc_cmd.x = flyappy_.get_x_acceleration();
    // acc_cmd.y = flyappy_.get_y_acceleration();

    acc_cmd.x = 0;
    acc_cmd.y = 0;

    pub_acc_cmd_.publish(acc_cmd);
}

void FlyappyRos::threadloop_ros()
{
    flyappy_.threadloop();
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // TODO Properly send Lidar data

    flyappy_.set_lidar_data(msg->angle_min, msg->angle_max, msg->angle_increment, msg->time_increment,
                          msg->scan_time, msg->range_min, msg->range_max,
                          msg->ranges, msg->intensities);

    // Example of printing laser angle and range
    // ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }

    flyappy_ = {};
}
