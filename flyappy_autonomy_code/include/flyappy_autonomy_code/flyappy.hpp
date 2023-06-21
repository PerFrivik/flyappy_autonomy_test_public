#pragma once

// Own Includes

#include <math.h>
#include <iostream>
#include <vector>
#include <chrono>


class Flyappy
{
  public:
    Flyappy();

    void set_lidar_data(float angle_min, float angle_max, float angle_increment, float time_increment,
                      float scan_time, float range_min, float range_max,
                      const std::vector<float>& ranges, const std::vector<float>& intensities);

    void set_vx_data(float vel_x, float vel_y);

    void find_the_gap();

    bool ready_to_zoom();

    double get_y_acceleration();

    double get_x_acceleration();

    void controller_y_acceleration();

    void controller_x_acceleration();

    void pass_rock(); 

    void threadloop();

    float get_abs_y_value(unsigned int num, float value);

    float get_squared_y_value(unsigned int num, float value);

    float get_weighted_squared_y_value(unsigned int num, float value1, float value2, float value3);

    float get_x_value();

    void track_velocity();

    void track_height(); 

    float WeightedMovingAverageFilter(std::vector<double> vec, double value);

  private:

    // Test bool

    bool test_ = true;

    // Modes 

    bool zoom_ = false; 

    bool explore_ = true;

    double threashold_ = 0.4;

    // Simulation counter

    bool started_simulation = false; 

    // Controller Data

    double dt_ = 1.0/30.0;
    double kp_ = 0.3;
    double ki_ = 0.0;
    double kd_ = 0.3;
    double integral_ = 0;
    double derivative_ = 0;

    double fly_up_value_ = 0;
    double fly_down_value_ = 0; 

    double error_ = 0;
    double previous_error_ = 0;
    double previous_error_u_ = 0;
    double weighted_y_acceleration_command_ = 0;
    double send_command_y_ = 0;
    double send_command_x_ = 0;

    // Controller for x acceleration 

    double safe_distance = 2.0;

    double v_x_ = 0.0; 

    double requested_v_x_ = 0;

    double requested_v_y_ = 0; 

    bool zoomer_ = false; 

    double kp1_ = 10.0;
    double ki1_ = 0.0;
    double kd1_ = 5.0;

    double kp_v_ = 2;
    double ki_v_ = 0;
    double kd_v_ = 0; 
    double integral_v_ = 0;
    double derivative_v_ = 0; 
    double error_v_ = 0; 
    double previous_error_v_ = 0;

    std::chrono::high_resolution_clock::time_point current_time_; 
    std::chrono::high_resolution_clock::time_point last_time_;
    std::chrono::high_resolution_clock::time_point zoom_timer_start_;
    double zoom_total_time_ = 0.4; 


    // Filter Information 

    // std::vector<double> MMAF(10, 0.0);
    // std::vector<double> WeightedMovingAverageFilterData_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> WeightedMovingAverageFilterData_ = {0.0};

    // Starting information

    double starting_height_ = 0;
    double increment_angle_ = 0.19634954631328583;
    double start_angle_ = -0.7853981852531433;

    // Pose of the system 

    double current_height_ = 0; 

    double current_distance_ = 0; 

    double last_distance_ = 0; 

    double track_this_height_ = 0; 

    bool got_start_height_ = false;

    // Velocity & Acceleration of the system

    // double v_x_ = 0;
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

    struct v
    {
      float x;
      float y;
    };

    v vel_;

    std::vector<float> control_data_; 







};
