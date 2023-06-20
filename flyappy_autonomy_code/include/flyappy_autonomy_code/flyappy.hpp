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

    double get_y_acceleration();

    void controller_y_acceleration();

    void controller_x_acceleration();

    void pass_rock(); 

    void threadloop();

    float get_squared_y_value(unsigned int num, float value);

    float get_x_value();

    float WeightedMovingAverageFilter(std::vector<double> vec, double value);

  private:

    // Test bool

    bool test_ = true;

    // Simulation counter

    bool started_simulation = false; 

    // Controller Data

    double dt_ = 1/30;
    double P_ = 1;
    double I_ = 1;
    double D_ = 1;

    double fly_up_value_ = 0;
    double fly_down_value_ = 0; 

    double y_acceleration_command_ = 0;
    double weighted_y_acceleration_command_ = 0;

    // Filter Information 

    // std::vector<double> MMAF(10, 0.0);
    // std::vector<double> WeightedMovingAverageFilterData_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> WeightedMovingAverageFilterData_ = {0.0};

    // Starting information

    double starting_height_ = 0;
    double increment_angle_ = 0.19634954631328583;
    double start_angle_ = -0.7853981852531433;

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

    std::vector<float> control_data_; 







};
