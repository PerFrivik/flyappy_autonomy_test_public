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

    // ------------------------------------------------------ Get Data --------------------------------------------//

    void get_lidar_data(float angle_min, float angle_max, float angle_increment, float time_increment,
                      float scan_time, float range_min, float range_max,
                      const std::vector<float>& ranges, const std::vector<float>& intensities);
    void get_vx_data(float vel_x, float vel_y);
    void initialize_lidar(); 
    float get_x_value(unsigned int num, float value);
    float get_y_value(unsigned int num, float value, bool abs_value);
    float get_y_value_at_wall(unsigned int num, float value);

    // ------------------------------------------------------ Set Data --------------------------------------------//

    double set_y_acceleration();
    double set_x_acceleration();

    // ------------------------------------------------------ State Estimation --------------------------------------------//

    void calibration(); 
    void state_estimation();
    void update_dt(); 

    // ------------------------------------------------------ Baby SLAM  --------------------------------------------//

    void baby_slam(); 

    bool baby_slam_run(); 
    void baby_slam_reset(); 
    void baby_slam_update_map();

    void baby_slam_longest_sequence(); 

    // ------------------------------------------------------ Controller --------------------------------------------//

    void threadloop(); 
    void x_pid();
    void y_pid(); 





  private:

    // Data
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

    std::vector<float> lidar_ranges_; 
    std::vector<float> lidar_intensities_; 

    struct s
    {
      float x;
      float y;
    };

    s state_;
    struct v
    {
      float x;
      float y;
    };

    v velocity_;

    struct a
    {
      float x;
      float y;
    };

    a acceleration_;

    // Initialization ---------------------------------------------------------------------



    // State Estimation --------------------------------------------------------------------

    double dt_ = 0; 

    bool calibrated_time_ = false; 

    bool calibrated_state_ = false; 

    std::chrono::high_resolution_clock::time_point current_time_; 
    std::chrono::high_resolution_clock::time_point previous_time_;

    double distance_to_wall_ = 0; 

    double start_angle_ = -0.7853981852531433;

    double increment_angle_ = 0.19634954631328583;

    // SLAM -----------------------------------------------------------------------------------

    bool run_slam_ = false; 

    bool reset_slam_ = false; 

    double upper_limit_ = 0; 
    double lower_limit_ = 0; 

    std::vector<int> map_;

    // Controller ------------------------------------------------------------------------

    bool steady_state_ = false; 
    double steady_state_threshold_ = 0.2; 

    //< Y - Controller 

    double steady_state_y_ = 0; 

    double kp_y_ = 1; 
    double ki_y_ = 1; 
    double kd_y_ = 1; 

    double error_y_ = 0; 
    double last_error_y_ = 0; 
    double integral_y_ = 0; 

    //< X - Controller

    double steady_state_x_ = 0; 

    double kp_x_ = 1; 
    double ki_x_ = 1; 
    double kd_x_ = 1; 

    double error_x_ = 0; 
    double last_error_x_ = 0; 
    double integral_x_ = 0; 












};
