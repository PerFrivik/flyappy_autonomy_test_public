#include "flyappy_autonomy_code/flyappy.hpp"

// ------------------------------------------------------ Get Data --------------------------------------------//


Flyappy::Flyappy()
{
     current_time_ = std::chrono::high_resolution_clock::now(); 
}

void Flyappy::get_lidar_data(float angle_min, float angle_max, float angle_increment, float time_increment,
                           float scan_time, float range_min, float range_max,
                           const std::vector<float>& ranges, const std::vector<float>& intensities)
{
    lidar_data_.angle_min = angle_min;
    lidar_data_.angle_max = angle_max;
    lidar_data_.angle_increment = angle_increment;
    lidar_data_.time_increment = time_increment;
    lidar_data_.scan_time = scan_time;
    lidar_data_.range_min = range_min;
    lidar_data_.range_max = range_max;
    lidar_data_.ranges = ranges;
    lidar_data_.intensities = intensities;
}

void Flyappy::get_vx_data(float velocity_x, float velocity_y)
{
    velocity_.x = velocity_x; 
    velocity_.y = velocity_y; 
}

void Flyappy::initialize_lidar()
{
    lidar_ranges_ = lidar_data_.ranges; 
    lidar_intensities_ = lidar_data_.intensities; 
}

float Flyappy::get_x_value(unsigned int num, float value)
{
    return cos(start_angle_ + num*increment_angle_)*value;
}

float Flyappy::get_y_value(unsigned int num, float value, bool abs_value)
{   
    if(abs_value)
    {
        return abs(sin(start_angle_ + num*increment_angle_)*value);
    } else 
    {
        return sin(start_angle_ + num*increment_angle_)*value;
    }
} 

float Flyappy::get_y_value_at_wall(unsigned int num, float value)
{   
    double factor = distance_to_wall_/get_x_value(num, value); // factor to get correct y value for the "gap" as if i just take the normal y value its worong too far up or down
    return sin(start_angle_ + num*increment_angle_)*value*factor;
}

// ------------------------------------------------------ Set Data --------------------------------------------//


double Flyappy::set_y_acceleration()
{
    return acceleration_.x; 
}

double Flyappy::set_x_acceleration()
{
    return acceleration_.y; 
}

// ------------------------------------------------------ State Estimation --------------------------------------------//

void Flyappy::calibration()
{
    update_dt();
    if(dt_ > 0.4 && dt_ < 0.5)
    {
        calibrated_time_ = true; 
    }

}

void Flyappy::state_estimation()
{   
    if(!calibrated_state_){
        state_.y = get_y_value(0, lidar_ranges_[0], true);
        calibrated_state_ = true;
         // initialize state x with 0
         state_.x = 0; 
    } else {
        state_.y += velocity_.y;
        state_.x += velocity_.x; 
    }

    // Gives and avergage distance to the next wall, only starts if wall if closer than 2
    double _total_x_distance_to_wall = 0;
    double _counter = 0;

    for(unsigned int i = 0; i < lidar_ranges_.size(); i++)
    {
        if(get_x_value(i, lidar_ranges_[i]) <= 2)
        {
            _total_x_distance_to_wall += get_x_value(i, lidar_ranges_[i]);
            _counter += 1; 
        }
    }
    if(_counter != 0){
        // We only start counting once a wall is within 2 
        distance_to_wall_ = _total_x_distance_to_wall/_counter; 
    } else {
        // If no wall is in sight, we get -1 
        distance_to_wall_ = -1; 
    }
}

void Flyappy::update_dt()
{
    dt_ = (current_time_ - previous_time_).count();
}

// ------------------------------------------------------ Baby SLAM --------------------------------------------//

bool Flyappy::run_baby_slam()
{
    if(distance_to_wall_ == -1){
        return false;
    } else if (run_slam_ == true){
        return true; 
    } else {
        return false; 
    }
}

void baby_slam()
{
    
}

// ------------------------------------------------------ Controller --------------------------------------------//

// Controller running at 30 hz
void Flyappy::threadloop()
{   
    
    // Update clock
    current_time_ = std::chrono::high_resolution_clock::now(); 

    // Check that games has started and received Lidar data -> starts controller 
    if( !lidar_data_.ranges.empty() )
    {

        // Initialize lidar_ranges_ and lidar_intensities_ 
        initialize_lidar(); 

        // Calibrate and initialize dt_ properly -> first we calibrate time, then we calibrate the state -> then we run babySLAM
        if(!calibrated_time_)
        {
            calibration();
        } else {
            update_dt();
            state_estimation(); 
        }

        // SLAM part
        if(run_baby_slam()){
            baby_slam(); 
        }
            



    }
    // Update clock 
    previous_time_ = current_time_; 

}



