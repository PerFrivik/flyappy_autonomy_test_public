#include "flyappy_autonomy_code/flyappy.hpp"

// current problem right now, if the distance is too big from the last scan, so bird is up but gap is down the laser wont pick up anything larger than 14 and wont update correctly. fix might be increasing the laser ranges

// longest_sequence_ <= min_gap_size_ && distance_to_wall_ < 1.0 idea works well i think, i just need to implement it somewhere earlier as a function that checks if it should go into emerg state, and this should be early
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


double Flyappy::set_x_acceleration()
{
    return acceleration_.x; 
}

double Flyappy::set_y_acceleration()
{   
    return acceleration_.y; 
}

// ------------------------------------------------------ State Estimation --------------------------------------------//

void Flyappy::dt_calibration()
{
    update_dt();
    if(dt_ > 0.02 && dt_ < 0.04)
    {
        calibrated_time_ = true; 
    }
    acceleration_.x = 0; 
    acceleration_.y = 0;
}

void Flyappy::state_estimation()
{   
    if(!calibrated_state_){
        state_.y = get_y_value(0, lidar_ranges_[0], true);
        calibrated_state_ = true;
         // initialize state x with 0
         state_.x = 0; 
    } else {
        state_.y += velocity_.y * dt_;
        state_.x += velocity_.x * dt_; 
    }

    // Gives and avergage distance to the next wall, only starts if wall if closer than 2
    double _total_x_distance_to_wall = 0;
    double _counter = 0;

    for(unsigned int i = 0; i < lidar_ranges_.size(); i++)
    {
        if(get_x_value(i, lidar_ranges_[i]) <= 2 && i != 0 && i != 8)
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
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_time_ - previous_time_);
    dt_ = time_span.count();
}

// ------------------------------------------------------ Baby SLAM --------------------------------------------//

bool Flyappy::baby_slam_run()
{
    if(distance_to_wall_ == -1 && !game_started_)
    {
        return false; 
    } else {
        game_started_ = true;
        return true;
    }
}

void Flyappy::baby_slam_initialize_map()
{   
    std::cout << "reset vector!!!!" << std::endl;
    map_ = std::vector<int>(map_accuracy_, 3);
}

bool Flyappy::baby_slam_reset()
{   
    std::cout << "distance to wall: " << distance_to_wall_ << std::endl;
    // if(distance_to_wall_ <= 0.5 && (abs(requested_y_position_ - state_.y) < 0.1))
    if(distance_to_wall_ <= 0.5)
    {
        return true;  
    } else {
        return false; 
    }
}

void Flyappy::baby_slam_maintain_state()
{
    run_slam_ = false; 

    if(!received_steady_state_goal_position_)
    {   
        if(distance_to_wall_ < 0){
            steady_state_goal_position_x_ = state_.x + 0.6;
        } else {
            steady_state_goal_position_x_ = state_.x + 0.35 + distance_to_wall_; 
        }
        steady_state_goal_position_y_ = requested_y_position_; 
        received_steady_state_goal_position_ = true; 
    }
    std::cout << "almost out of steady state" << steady_state_goal_position_x_ - state_.x << std::endl;
    if((steady_state_goal_position_x_ - state_.x) <= 0.0)
    {
        run_slam_ = true; 
        steady_state_ = false;
        received_steady_state_goal_position_ = false;
        longest_sequence_ = 0; 
    }

}

void Flyappy::baby_slam()
{   
    std::cout << "im back in baby_slam" << std::endl;
    if(!initialized_map_)
    {
        baby_slam_initialize_map();
        initialized_map_ = true; 
    }

    emergency_controller();

    if(!baby_slam_reset() && !steady_state_)
    {
        baby_slam_update_map();
        baby_slam_longest_sequence();
        if(upper_limit_ + lower_limit_ == 0){
            requested_y_position_ = 2.0; 
        } else {
            requested_y_position_ = (upper_limit_ + lower_limit_)/200.0;
        }
    } else {
        if(!emergency_ && longest_sequence_ >= min_gap_size_ && (requested_y_position_ - state_.y) <= 0.05){
            steady_state_ = true;
            std::cout << "steady state" << std::endl;
            baby_slam_maintain_state(); 
            if(!steady_state_){
                baby_slam_initialize_map();
            }
        }
            // steady_state_ = true;
            // std::cout << "steady state" << std::endl;
            // baby_slam_maintain_state(); 
            // if(!steady_state_){
            //     baby_slam_initialize_map();
            // }
    }

}

void Flyappy::baby_slam_update_map()
{
    for(unsigned int i = 0; i < lidar_ranges_ .size(); i++)
    {   
        double _point_location = state_.y + get_y_value_at_wall(i, lidar_ranges_ [i]);
        int _map_location = (int) (_point_location * 100);
        if(_map_location < 0){
            _map_location = 0;
        } else if (_map_location > 403){
            _map_location = 403;
        }

        if(abs(get_x_value(i, lidar_ranges_ [i]) - distance_to_wall_) < 0.3){
            if (map_[_map_location] != 0){
                map_[_map_location] = 1; 
            }
        } else {
            map_[_map_location] = 0;

        }
    }
    // for (int i = 0; i < map_.size(); i++)
    // {
    //     std::cout << map_[i] << " ";
    // }
}

void Flyappy::baby_slam_longest_sequence()
{   
    double _sequence = 0; 

    for(unsigned int i = 1; i < map_.size() - 1; i++)
    {
        if(map_[i] == 0)
        {
            _sequence +=1;      
        }
        else if ((map_[i] == 1 || map_[i] == 3) && map_[i-1] == 0 && map_[i+1] == 0)
        {
            // Include a 1 or 3 if they are sandwiched between 0s
            _sequence += 1;
        }
        else 
        {
            if (_sequence > longest_sequence_) {
                longest_sequence_ = _sequence; 
                if(longest_sequence_ >= min_gap_size_){
                    upper_limit_ = i -1; 
                    lower_limit_ = i - _sequence; 
                }
            }
            _sequence = 0; // Sequence ended, so reset
        }
    }
    
    // Check for the longest sequence one last time after the loop ends
    // if (_sequence > _longest_sequence) {
    //     _longest_sequence = _sequence; 
    //     upper_limit_ = map_.size() - 1; 
    //     lower_limit_ = map_.size() - _sequence;
    // }

    std::cout << "gap_size: " << upper_limit_ - lower_limit_ << " " << upper_limit_ << " " << lower_limit_ << std::endl;

}

// ------------------------------------------------------ Controller --------------------------------------------//

void Flyappy::emergency_controller()
{
    if(longest_sequence_ <= min_gap_size_ && distance_to_wall_ < 0.75 )
    {
        emergency_ = true; 
    } else {
        emergency_ = false;
    }
}

void Flyappy::x_pid()
{   
    if(!steady_state_ && !emergency_)
    {
        // requested_x_velocity_ = (lidar_ranges_[3] + lidar_ranges_[4] + lidar_ranges_[4])/15;
        requested_x_velocity_ = (lidar_ranges_[3] + lidar_ranges_[4]*3 + lidar_ranges_[5])/30 - abs(error_y_)*10;
        if(lidar_ranges_[4] < 0.5){
            requested_x_velocity_ = 0; 
        }
        error_x_ = requested_x_velocity_ - velocity_.x; 
    } else if (emergency_) {
        requested_x_velocity_ = 0; 
        error_y_ = requested_x_velocity_ - velocity_.x; 
        std::cout << "emergency x" << std::endl;
    } else {
        std::cout << "im in the steadystate" << std::endl;
        error_x_ = steady_state_goal_position_x_ - state_.x; 
    }

    integral_x_ += error_x_ * dt_; 

    derivative_x_ = (error_x_ - last_error_x_) / dt_; 

    if(derivative_x_ > 100){
        derivative_x_ = 0;
    }

    acceleration_.x = kp_x_ * error_x_ + ki_x_ * integral_x_ + kd_x_ * derivative_x_; 

    last_error_x_ = error_x_;

}

void Flyappy::y_pid()
{   
    if(!steady_state_ && !emergency_)
    {
        error_y_ = requested_y_position_ - state_.y; 
    } else if (emergency_ && !steady_state_) {
        emergency_ = true; 
        if(state_.y < 1.5)
        {
            requested_y_position_ = 2.5; 
        } else if(state_.y > 2.5) {
            requested_y_position_ = 1.5; 
        } else {
            std::cout << "should I b ehere? " << std::endl;
            requested_y_position_ = last_requested_y_position_;
        }
        last_requested_y_position_ = requested_y_position_;
        error_y_ = requested_y_position_ - state_.y; 
        std::cout << "emergency y" << std::endl;
    } else {
        emergency_ = false;
        error_y_ = steady_state_goal_position_y_ - state_.y;
    }
    
    integral_y_ += error_y_ * dt_; 

    derivative_y_ = (error_y_ - last_error_y_) / dt_; 

    if(derivative_y_ > 100){
        derivative_y_ = 0;
    }

    // std::cout << " " << std::endl;
    std::cout << "requested y position: " << requested_y_position_ << " current y position " << state_.y << std::endl;
    std::cout << "largest gap: " << longest_sequence_ << std::endl;

    acceleration_.y = kp_y_ * error_y_ + ki_y_ * integral_y_ + kd_y_ * derivative_y_; 

    last_error_y_ = error_y_;
}

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
            dt_calibration();
        } else {
            update_dt();
            state_estimation(); 

            // SLAM part
            if(baby_slam_run() ){
                baby_slam(); 
            } else {
                std::cout << "im not running slam at all" << std::endl;
            }

            
            y_pid();
            x_pid();

            std::cout << " " << std::endl;

        }

    }
    // Update clock 
    previous_time_ = current_time_; 
}



