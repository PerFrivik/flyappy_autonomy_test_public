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

    void dt_calibration(); 
    void state_estimation();
    void update_dt(); 

    // ------------------------------------------------------ Baby SLAM  --------------------------------------------//

    void baby_slam(); 

    bool baby_slam_run(); 
    bool baby_slam_reset(); 
    void baby_slam_initialize_map();
    void baby_slam_update_map();

    void baby_slam_longest_sequence(); 

    void baby_slam_maintain_state(); 

    // ------------------------------------------------------ Controller --------------------------------------------//

    void threadloop(); 
    void x_pid();
    void y_pid(); 
    void emergency_controller(); 





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
    double last_distance_to_wall_ = 0;

    double start_angle_ = -0.7853981852531433;

    double increment_angle_ = 0.19634954631328583;

    // SLAM -----------------------------------------------------------------------------------

    bool run_slam_ = false; 

    bool game_started_ = false;

    bool received_steady_state_goal_position_ = false; 

    double steady_state_goal_position_x_ = 0; 
    double steady_state_goal_position_y_ = 0; 

    double upper_limit_ = 0; 
    double lower_limit_ = 0; 

    bool initialized_map_ = false; 

    std::vector<int> map_;

    int map_accuracy_ = 403; 

    int gap_size_ = 0; 

    // int min_gap_size_ = 24; 

    int min_gap_size_ = 8; 

    int emergency_gap_size_ = 24; 


    bool emergency_mode_ = false; 

    double longest_sequence_ = 0; 

    bool emergency_ = false; 

    bool should_reset_ = false;

    // Controller ------------------------------------------------------------------------

    bool steady_state_ = false; 
    double steady_state_threshold_ = 0.2; 

    //< Y - Controller 

    double steady_state_y_ = 0; 

    double kp_y_ = 5.5; 
    double ki_y_ = 0.0; 
    double kd_y_ = 5.0; 

    double error_y_ = 0; 
    double last_error_y_ = 0; 
    double integral_y_ = 0; 
    double derivative_y_ = 0; 

    double requested_y_position_ = 2; 
    double last_requested_y_position_ = 0;

    bool going_up_ = false;
    bool going_down_ = false; 

    //< X - Controller

    double steady_state_x_ = 0; 

    double wanted_x_velocity_ = 1.25;

    double kp_x_ = 1.0; 
    double ki_x_ = 0.0; 
    double kd_x_ = 0.0; 

    double error_x_ = 0; 
    double last_error_x_ = 0; 
    double integral_x_ = 0; 
    double derivative_x_ = 0; 

    double requested_x_velocity_ = 0; 

    












};
