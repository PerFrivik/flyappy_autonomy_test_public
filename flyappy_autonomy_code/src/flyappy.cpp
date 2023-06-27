#include "flyappy_autonomy_code/flyappy.hpp"

// current problem right now, if the distance is too big from the last scan, so bird is up but gap is down the laser wont pick up anything larger than 14 and wont update correctly. fix might be increasing the laser ranges

// longest_sequence_ <= min_gap_size_ && distance_to_wall_ < 1.0 idea works well i think, i just need to implement it somewhere earlier as a function that checks if it should go into emerg state, and this should be early

// Issue that it does not currectly reset, but thats the last real issue i think

// Main issue is that does not correct reset the vector, so the gap gets huge sometimes, I blieve the other resest works i tested it but im not sure, maybe its because of the steady state thing, i shoudl restet after steady state 
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
    //std::cout<< acceleration_.x << std::endl;
    return acceleration_.x; 
    // return 0; 
}

double Flyappy::set_y_acceleration()
{   
    return acceleration_.y; 
    // return 0; 
}

// ------------------------------------------------------ State Estimation --------------------------------------------//

void Flyappy::dt_calibration()
{
    update_dt();
    // if(dt_ > 0.02 && dt_ < 0.04)
    if(dt_ > 0.007 && dt_ < 0.009)
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

    // last_distance_to_wall_ = distance_to_wall_; 
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
    //std::cout<< "reset vector!!!!" << std::endl;
    map_ = std::vector<int>(map_accuracy_, 1);
}

bool Flyappy::baby_slam_reset()
{    
    //std::cout<< "dist: " << distance_to_wall_ << " " << last_distance_to_wall_ << std::endl;
    if(abs(distance_to_wall_ - last_distance_to_wall_) >= 0.3)
    {   

        //std::cout<< "i should reset the SLAM now" << std::endl;
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
            steady_state_goal_position_x_ = state_.x + 0.35;
        } else {
            steady_state_goal_position_x_ = state_.x + last_distance_to_wall_ + 0.4; 
        }
        steady_state_goal_position_y_ = requested_y_position_; 
        received_steady_state_goal_position_ = true; 
    }
    //std::cout<< "almost out of steady state" << steady_state_goal_position_x_ - state_.x << std::endl;
    if((steady_state_goal_position_x_ - state_.x) <= 0.0)
    {   
        //std::cout<< "steady state is over im running slam again" << std::endl;
        run_slam_ = true; 
        steady_state_ = false;
        received_steady_state_goal_position_ = false;
        longest_sequence_ = 0; 
    }

}

void Flyappy::baby_slam()
{   
    if(!initialized_map_)
    {
        baby_slam_initialize_map();
        start_visualization();
        initialized_map_ = true; 
    }

    emergency_controller();
    baby_slam_check_for_collision();

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
        //std::cout<< "i should go into steady steate: " << emergency_ << " "  << longest_sequence_  << " " << abs(requested_y_position_ - state_.y) << std::endl;
        if(!emergency_ && longest_sequence_ >= min_gap_size_ /*&& (abs(requested_y_position_ - state_.y) <= 0.1)*/){
            steady_state_ = true;
            //std::cout<< "running steady state" << std::endl;
            baby_slam_maintain_state(); 
            if(!steady_state_){
                //std::cout<< "im resetting the map because the steady state is over" << std::endl;
                baby_slam_initialize_map();
            }
        }
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

        if(abs(get_x_value(i, lidar_ranges_ [i]) - distance_to_wall_) < 0.4){
            if (map_[_map_location] != 0 || (i == 4) || (distance_to_wall_ < 0.7)){
                if(map_[_map_location] == 0 && _map_location >= lower_limit_ && _map_location <= upper_limit_){
                    longest_sequence_ = 0;
                } else {
                    // if(i == 2)
                    std::cout << i<<  " map location: " << _map_location << " " << "im a rock" << std::endl;
                    map_[_map_location] = 1; 
                }
            }
        } else {
            // if(i == 2)
            std::cout << i << " map location: " << _map_location << " " << "im a gap" << std::endl;
            map_[_map_location] = 0;

        }
        // if(i == 2){
        std::cout << i << " distance_to_wall_: " << distance_to_wall_ << " get_x_value(i, lidar_ranges_ [i]) " << get_x_value(i, lidar_ranges_ [i]) << " " <<  (get_x_value(i, lidar_ranges_ [i]) - distance_to_wall_) <<  std::endl; 
        // }
    }
    // for (int i = map_.size(); i > 0; i--)
    // {
    //     std::cout<< map_[i] << " ";
    // }
}

// void Flyappy::baby_slam_longest_sequence()
// {   
//     double _sequence = 0; 

//     for(unsigned int i = 1; i < map_.size() - 1; i++)
//     {
//         if(map_[i] == 0)
//         {
//             _sequence +=1;      
//         }
//         else if ((map_[i] == 1 || map_[i] == 3) && map_[i-1] == 0 && map_[i+1] == 0)
//         {
//             // Include a 1 or 3 if they are sandwiched between 0s
//             _sequence += 1;
//         }
//         else 
//         {
//             if (_sequence > longest_sequence_) {
//                 longest_sequence_ = _sequence; 
//                 if(longest_sequence_ >= min_gap_size_){
//                     upper_limit_ = i -1; 
//                     lower_limit_ = i - _sequence; 
//                 }
//             }
//             _sequence = 0; // Sequence ended, so reset
//         }
//     }

// }

void Flyappy::baby_slam_longest_sequence()
{   
    double _sequence = 0;
    int _tolerance = 3;  // Maximum number of consecutive 1s allowed in a sequence
    int _consecutive_ones = 0;

    for(unsigned int i = 1; i < map_.size() - 1; i++)
    {
        if(map_[i] == 0)
        {
            _sequence += 1;
            _consecutive_ones = 0;  // Reset the counter of consecutive 1s
        }
        else if (map_[i] == 1 && _consecutive_ones < _tolerance) 
        {
            _sequence += 1;
            _consecutive_ones += 1;  // Increase the counter of consecutive 1s
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
            _consecutive_ones = 0;  // Reset the counter of consecutive 1s
        }
    }
}


void Flyappy::baby_slam_check_for_collision()
{
    double _top_of_bird = state_.y + 0.15;
    double _bottom_of_bird = state_.y - 0.15; 

    if((_top_of_bird >= upper_limit_ || _bottom_of_bird <= lower_limit_) && distance_to_wall_ <  0.5)
    {
        out_of_bounds_ = true;
    } else {
        out_of_bounds_ = false; 
    }
}


void Flyappy::start_visualization() {
    // Start the visualization in a new thread
    std::thread vis_thread(&Flyappy::visualize_map, this);

    // Detach the thread so it runs independently
    vis_thread.detach();
}

void Flyappy::visualize_map() {
    while (is_running_) {
        // Set up the image
        int width = 1;
        int height = map_.size();
        cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    
        // Set pixel colors based on map values
        for (int i = map_.size(); i > 0; --i) {
            int y = 403 - i;
            
            if (map_[i] == 0)
                img.at<cv::Vec3b>(y, 0) = cv::Vec3b(0, 0, 0); // Black for 0
            else if (map_[i] == 1)
                img.at<cv::Vec3b>(y, 0) = cv::Vec3b(255, 255, 255); // White for 1
            else if (map_[i] == 3)
                img.at<cv::Vec3b>(y, 0) = cv::Vec3b(0, 0, 255); // Red for 3
        }
        img.at<cv::Vec3b>(403 - state_.y*100, 0) = cv::Vec3b(0, 0, 255); // Red for 3
        img.at<cv::Vec3b>(403 - upper_limit_, 0) = cv::Vec3b(255, 0, 255); // Red for 3
        img.at<cv::Vec3b>(403 - lower_limit_, 0) = cv::Vec3b(255, 0, 255); // Red for 3

        // Resize the image to be more visible
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(width * 100, height * 2)); // Adjust the dimensions as needed

        // Display the image
        cv::imshow("Map", resized_img);
        cv::waitKey(1);  // non-blocking

        // Sleep for a while to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



// ------------------------------------------------------ Controller --------------------------------------------//

void Flyappy::emergency_controller()
{   
    //std::cout<< "emergency data: longest sequnce: " << longest_sequence_ << " distance to wall: " << distance_to_wall_ << std::endl;
    if(longest_sequence_ <= emergency_gap_size_ && distance_to_wall_ < 0.45 )
    {
        emergency_ = true; 
        //std::cout<< "emergency!!!" << std::endl;
    } else if (longest_sequence_ >= 40) {
        emergency_ = false;
        going_down_ = false;
        going_up_ = false;
    }
}

void Flyappy::x_pid()
{   
    if(!steady_state_ && !emergency_)
    {
        if(longest_sequence_ != 0){
            requested_x_velocity_ = wanted_x_velocity_ - abs(error_y_)*10 + (longest_sequence_ - emergency_gap_size_)/20;

        } 
        else 
        {
            requested_x_velocity_ = wanted_x_velocity_ - abs(error_y_)*10;

        }
        // requested_x_velocity_ = wanted_x_velocity_ - abs(error_y_)*10 + (longest_sequence_ - emergency_gap_size_)/20;

        //std::cout<< "positive vel term: " << (lidar_ranges_[3] + lidar_ranges_[4]*3 + lidar_ranges_[5])/20 << " negative vel term: " << abs(error_y_)*2 << std::endl;
        //std::cout<< "running normal pid" << std::endl;

        if(lidar_ranges_[4] < 0.3){
            requested_x_velocity_ = -10; 
        }
        error_x_ = requested_x_velocity_ - velocity_.x; 
    } 
    else if (emergency_ || out_of_bounds_) 
    {
        //std::cout<< "running emerg x pid" << std::endl;
        requested_x_velocity_ = 0; 
        error_x_ = -3 + (requested_x_velocity_ - velocity_.x)*10; 
        //std::cout<< requested_x_velocity_ << " " << velocity_.x << "!!!!!!!!!!!!!!!!!!!!!! emergg" << std::endl;
        // //std::cout<< "emergency x" << std::endl;
    } 
    else if (steady_state_)
    {
        // //std::cout<< "im in the steadystate" << std::endl;
        //std::cout<< "running steady x pid" << std::endl;
        requested_x_velocity_ = 1.5 - abs(error_y_)*10 + (longest_sequence_ - emergency_gap_size_)/20;
        error_x_ = (requested_x_velocity_ - velocity_.x); 
    }

    integral_x_ += error_x_ * dt_; 

    derivative_x_ = (error_x_ - last_error_x_) / dt_; 

    if(derivative_x_ > 100){
        derivative_x_ = 0;
    }

    acceleration_.x = kp_x_ * error_x_ + ki_x_ * integral_x_ + kd_x_ * derivative_x_; 

    //std::cout<< "my x acceleration " << acceleration_.x << std::endl;

    if(emergency_)
    {
        acceleration_.x = kp_x_ * error_x_; 
        //std::cout<< "my x emergency acceleration " << acceleration_.x << std::endl;
    }


    last_error_x_ = error_x_;

}

void Flyappy::y_pid()
{   
    if(!steady_state_ && !emergency_)
    {
        error_y_ = requested_y_position_ - state_.y; 
        //std::cout<< "running normal y pid" << std::endl;

    } else if (emergency_ && !steady_state_) {
        // emergency_ = true; 
        //std::cout<< "running emerg y pid" << std::endl;

        if(state_.y < 2 && !going_down_)
        {   
            going_up_ = true;
            requested_y_position_ = 3; 
        } else if(state_.y > 2 && !going_up_) {
            going_down_ = true;
            requested_y_position_ = 1; 
        } else if(state_.y < 1.1) {
            going_down_ = false;
            going_up_ = true; 
        } else if(state_.y > 2.9){
            going_up_ = false;
            going_down_ = true;
        } else {
            // //std::cout<< "should I b ehere? " << std::endl;
            requested_y_position_ = last_requested_y_position_;
        }
        last_requested_y_position_ = requested_y_position_;
        error_y_ = requested_y_position_ - state_.y; 
        // //std::cout<< "emergency y" << std::endl;
    } else if (steady_state_) {
        //std::cout<< "running steady y pid" << std::endl;

        // emergency_ = false;
        error_y_ = (steady_state_goal_position_y_ - state_.y)*2;
    }
    
    integral_y_ += error_y_ * dt_; 

    derivative_y_ = (error_y_ - last_error_y_) / dt_; 

    if(derivative_y_ > 100){
        derivative_y_ = 0;
    }

    //std::cout<< " " << std::endl;
    //std::cout<< "requested y position: " << requested_y_position_ << " current y position " << state_.y << std::endl;
    //std::cout<< "largest gap: " << longest_sequence_ << std::endl;
    //std::cout<< upper_limit_ << " " << lower_limit_ << std::endl;

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
        is_running_ = true; 
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
                // //std::cout<< "im not running slam at all" << std::endl;
                is_running_ = false; 
            }

            
            y_pid();
            x_pid();

            last_distance_to_wall_ = distance_to_wall_; 


            //std::cout<< "distance to wall: " << distance_to_wall_ << std::endl;
            std::cout<< " " << std::endl;

        }

    }
    // Update clock 
    previous_time_ = current_time_; 
    // is_running_ = false; 
}



