#pragma once

// Own Includes

#include <math.h>
#include <iostream>
#include <vector>


class Flyappy
{
  public:
    Flyappy();

    void set_lidar_data(float angle_min, float angle_max, float angle_increment, float time_increment,
                      float scan_time, float range_min, float range_max,
                      const std::vector<float>& ranges, const std::vector<float>& intensities);

    void find_the_gap();

    void controller_y_acceleration();

    void controller_x_acceleration();

    void pass_rock(); 

    void threadloop();

  private:

    // Simulation counter

    bool started_simulation = false; 

    // Controller PID 

    double dt_ = 1/30;
    double P_ = 1;
    double I_ = 1;
    double D_ = 1;

    // Starting information

    double starting_height_ = 0;

    // Pose of the system 

    double current_height_ = 0; 

    // Velocity & Acceleration of the system

    double v_x_ = 0;
    double v_y_ = 0;
    double a_x_ = 0;
    double a_y_ = 0; 

    // Lidar Information 

    struct LidarData
    {
        float angle_min;
        float angle_max;
        float angle_increment;
        float time_increment;
        float scan_time;
        float range_min;
        float range_max;
        std::vector<float> ranges;
        std::vector<float> intensities;
    };

    LidarData lidar_data_;







};
