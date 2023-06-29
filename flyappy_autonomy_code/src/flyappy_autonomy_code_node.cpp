#include <ros/ros.h>
#include <thread>

#include "flyappy_autonomy_code/flyappy_ros.hpp"

void threadLoop(FlyappyRos& flyappy_ros) 
{

  // Set the desired loop rate to 30 Hz
  ros::Rate loop_rate(120);

  // Custom thread loop
  while (ros::ok()) 
  {
    
    flyappy_ros.threadloop_ros();

    // Sleep to maintain the desired loop rate
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flyappy_autonomy_code");
    ros::NodeHandle nh;

    FlyappyRos flyappy_ros{nh};

    // Create a separate thread for the thread loop
    std::thread customThread(threadLoop, std::ref(flyappy_ros));

    ros::spin();

    return 0;
}
