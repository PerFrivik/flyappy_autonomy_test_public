#include "flyappy_autonomy_code/flyappy.hpp"

// current problem right now, if the distance is too big from the last scan, so bird is up but gap is down the laser wont pick up anything larger than 14 and wont update correctly. fix might be increasing the laser ranges

// longest_sequence_ <= min_gap_size_ && distance_to_wall_ < 1.0 idea works well i think, i just need to implement it somewhere earlier as a function that checks if it should go into emerg state, and this should be early

// Issue that it does not currectly reset, but thats the last real issue i think

// Main issue is that does not correct reset the vector, so the gap gets huge sometimes, I blieve the other resest works i tested it but im not sure, maybe its because of the steady state thing, i shoudl restet after steady state 
// ------------------------------------------------------ Get Data --------------------------------------------//

// Did not really use this, should/could have used it more if I wanted to use a seperate yaml file for the pid values 
Flyappy::Flyappy()
{
     current_time_ = std::chrono::high_resolution_clock::now(); 
}

// Get lidar data from a seperate thread running, check flyappy_autonomy_code_node.cpp for more information 
void Flyappy::get_lidar_data(float angle_min, float angle_max, float angle_increment, 
                             float time_increment, float scan_time, float range_min, 
                             float range_max, const std::vector<float>& ranges, 
                             const std::vector<float>& intensities)
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

// Get the velocity data from the topic 
void Flyappy::get_vx_data(float velocity_x, float velocity_y)
{
    velocity_.x = velocity_x; 
    velocity_.y = velocity_y; 
}

// Intialize lidar ranges and intensities to simplify the code 
void Flyappy::initialize_lidar()
{
    lidar_ranges_ = lidar_data_.ranges; 
    lidar_intensities_ = lidar_data_.intensities; 
}

// Helper functions 
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
    // Send the x acceleration to the flyappy_ class
    return acceleration_.x; 
}

double Flyappy::set_y_acceleration()
{   
    // Send the y acceleration to the flyappy_ class
    return acceleration_.y; 
}

// ------------------------------------------------------ State Estimation --------------------------------------------//

// Calibrate dt_ 
void Flyappy::dt_calibration()
{   
    // Function to calculate the current dt_ 
    update_dt();

    // On initialization the dt_ value will be wrong, so we wait for it to calibrate before we preceed with the controller
    if(dt_ > 0.007 && dt_ < 0.009)
    {
        calibrated_time_ = true; 
    }

    // Initialize the acceleration of the bird 
    acceleration_.x = 0; 
    acceleration_.y = 0;
}

// State estimation, gives us position of flyappy 
void Flyappy::state_estimation()
{   
    // Start by calibrating the height and initializing the position 
    if(!calibrated_state_){
        state_.y = get_y_value(0, lidar_ranges_[0], true);
        calibrated_state_ = true;
         state_.x = 0; 
    } else {
        state_.y += velocity_.y * dt_;
        state_.x += velocity_.x * dt_; 
    }

    // Gives and avergage distance to the next wall, only starts if wall if closer than 2 to avoid it also using the walls behind 
    double _total_x_distance_to_wall = 0;
    double _counter = 0;

    // Loop through all the lidar ranges and get their average x-distance to the wall
    for(unsigned int i = 0; i < lidar_ranges_.size(); i++)
    {
        if(get_x_value(i, lidar_ranges_[i]) <= 2 /*&& i != 0 && i != 8*/)
        {
            _total_x_distance_to_wall += get_x_value(i, lidar_ranges_[i]);
            _counter += 1; 
        }
    }
    if(_counter != 0){
        distance_to_wall_ = _total_x_distance_to_wall/_counter; 
    } else {
        // If no wall is in sight, we get -1 
        distance_to_wall_ = -1; 
    }
}

// Function to calculate the current dt_ 
void Flyappy::update_dt()
{   
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_time_ - previous_time_);
    dt_ = time_span.count();
}

// ------------------------------------------------------ Baby SLAM --------------------------------------------//

// Checks wether we should currently be running the baby SLAM or not. 
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

// Function to both initialize and reset the baby SLAM map (array) and fill it with 1's 
void Flyappy::baby_slam_initialize_map()
{   
    map_ = std::vector<int>(map_accuracy_, 1);
}

// Function to check when the baby SLAM should be reset aka. callling baby_slam_initialize_map()
bool Flyappy::baby_slam_reset()
{    
    // It resets when it realizes that flyappy has gone through the wall or when it realizes that it has a large enough gap already, but turns back on again for emergencies 
    if(abs(distance_to_wall_ - last_distance_to_wall_) >= 0.3 || (longest_sequence_ > allow_steady_state_ && in_bounds_) && !emergency_) 
    {   
        return true;  
    } else {
        return false; 
    }
}

// Function to maintain steady state for flyappy, this is when we have found a large enough gap and we just want to cruise through the gap and maintain our attitude
void Flyappy::baby_slam_maintain_state()
{
    // We stop the baby SLAM for safety reasons 
    run_slam_ = false; 

    // In the first iteration we calculate how far we need to be in out steady state by checking how far away the wall is 
    if(!received_steady_state_goal_position_)
    {   
        if(distance_to_wall_ < 0){
            steady_state_goal_position_x_ = state_.x + 0.4;
        } else {
            steady_state_goal_position_x_ = state_.x + last_distance_to_wall_ + 0.4; 
        }
        steady_state_goal_position_y_ = gap_middle_y_;  
        received_steady_state_goal_position_ = true; 
    }
    
    // When we have reached our steady state goal, its to time run baby SLAM again 
    if((steady_state_goal_position_x_ - state_.x) <= 0.0)
    {   
        run_slam_ = true; 
        steady_state_ = false;
        received_steady_state_goal_position_ = false;
        longest_sequence_ = 0; 
    }

}

// Main baby SLAM function, that is responsible for calling all sub baby SLAM functions 
void Flyappy::baby_slam()
{   
    // Initialize the map for the first time before the game starts 
    if(!initialized_map_)
    {
        baby_slam_initialize_map();
        start_visualization();
        initialized_map_ = true; 
    }

    // Emergency controller && collison check function is always being called while running baby SLAM 
    emergency_controller();
    baby_slam_check_for_collision();

    // If running SLAM
    if(!baby_slam_reset() && !steady_state_)
    {   
        // Update the current map && find the correct gap
        baby_slam_update_map();
        baby_slam_longest_sequence();
        if(upper_limit_ + lower_limit_ == 0)
        {
            requested_y_position_ = 2.0; 
        } 
        else 
        {
            // Calculate the middle of the gap 
            requested_y_position_ = (upper_limit_ + lower_limit_)/200.0;
            gap_middle_y_ = requested_y_position_;
        }
    } 
    else 
    {
        if(!emergency_ && longest_sequence_ >= min_gap_size_ /*&& (abs(requested_y_position_ - state_.y) <= 0.1)*/)
        {
            steady_state_ = true;
            baby_slam_maintain_state(); 
            if(!steady_state_)
            {
                baby_slam_initialize_map();
            }
        }
    }
}

// Function to update the map which flyappy sees 
void Flyappy::baby_slam_update_map()
{   
    // Here we loop through the lidar ranges and check if there is a rock there or not 
    for(unsigned int i = 0; i < lidar_ranges_ .size(); i++)
    {   
        // We start by getting where the current lidar is on the map 
        double _point_location = state_.y + get_y_value_at_wall(i, lidar_ranges_ [i]);
        int _map_location = (int) (_point_location * 100);

        if(_map_location < 0)
        {
            _map_location = 0;
        } 
        else if (_map_location > 403)
        {
            _map_location = 403;
        }

        // Then we check if there is a rock or not, if there is none we fill the location on the map with a 0
        if(abs(get_x_value(i, lidar_ranges_ [i]) - distance_to_wall_) < 0.4)
        {
            if (map_[_map_location] != 0 || (i == 4) || (distance_to_wall_ < emergency_distance_))
            {   
                // Here we check if we thought something was a gap, but instead is a solid location and this location is inside our current gap, we reset that gap and find the new gap
                if(map_[_map_location] == 0 && _map_location >= lower_limit_ && _map_location <= upper_limit_)
                {
                    longest_sequence_ = 0;
                    map_[_map_location] = 1; 
                } 
                else 
                {
                    map_[_map_location] = 1; 
                }
            }
        } 
        else 
        {
            map_[_map_location] = 0;
        }
    }
}

// Function to find the longest_sequence of 0's aka our gaps 
void Flyappy::baby_slam_longest_sequence()
{   
    double _sequence = 0;
    // number of consecutive 1s allowed in a sequence, we dont want to ruin a correct gap if there is a "wrong" or not scanned 1 in the middle
    int _tolerance = 1;  
    int _consecutive_ones = 0;

    // We loop through the map to find the gap (i also solved the edge case by just not including it... hehe)
    for(unsigned int i = 1; i < map_.size() - 1; i++)
    {
        if(map_[i] == 0)
        {
            _sequence += 1;
            _consecutive_ones = 0;  
        }
        else if (map_[i] == 1 && _consecutive_ones < _tolerance) 
        {
            _sequence += 1;
            _consecutive_ones += 1;  
        }
        else 
        {
            if (_sequence > longest_sequence_) 
            {
                longest_sequence_ = _sequence; 

                if(longest_sequence_ >= min_gap_size_)
                {
                    upper_limit_ = i -1; 
                    lower_limit_ = i - _sequence; 
                }
            }
            _sequence = 0; 
            _consecutive_ones = 0;  
        }
    }
}

// Function to check if flyappy is in line with the gap (PS. I understand that the out_of_bounds_ and in_bounds is overconstrained and not elegant but I left it in for now)
void Flyappy::baby_slam_check_for_collision()
{
    top_of_bird_ = state_.y + 0.15;
    bottom_of_bird_ = state_.y - 0.15; 

    if((top_of_bird_*100 >= upper_limit_ || bottom_of_bird_*100 <= lower_limit_) && distance_to_wall_ <  0.45)
    {
        out_of_bounds_ = true;
        in_bounds_ = false; 
    } 
    else if ((top_of_bird_*100 <= upper_limit_ || bottom_of_bird_*100 >= lower_limit_))
    {
        in_bounds_ = true; 
        out_of_bounds_ = false; 
    } 
}

// start_visualization decides when we start our seperate thread to visualize what flyappy sees and where he currently thinks he is
void Flyappy::start_visualization() 
{
    std::thread vis_thread(&Flyappy::visualize_map, this);

    // thread so it runs independently
    vis_thread.detach();
}

// FUnction to visualize and start the popup 
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
            {
                img.at<cv::Vec3b>(y, 0) = cv::Vec3b(0, 0, 0); 
            }
            else if (map_[i] == 1)
            {
                img.at<cv::Vec3b>(y, 0) = cv::Vec3b(255, 255, 255); 
            }
            else if (map_[i] == 3)
            {
                img.at<cv::Vec3b>(y, 0) = cv::Vec3b(0, 0, 255); 
            }        
        }
        img.at<cv::Vec3b>(403 - state_.y*100, 0) = cv::Vec3b(0, 0, 255); 
        img.at<cv::Vec3b>(403 - top_of_bird_*100, 0) = cv::Vec3b(0, 0, 255); 
        img.at<cv::Vec3b>(403 - bottom_of_bird_*100, 0) = cv::Vec3b(0, 0, 255); 

        img.at<cv::Vec3b>(403 - upper_limit_, 0) = cv::Vec3b(255, 0, 255); 
        img.at<cv::Vec3b>(403 - lower_limit_, 0) = cv::Vec3b(255, 0, 255); 
        img.at<cv::Vec3b>(403 - requested_y_position_, 0) = cv::Vec3b(255, 255, 0); 

        // Resize the image to be more visible
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(width * 100, height * 2));

        // Display the image
        cv::imshow("Map", resized_img);
        cv::waitKey(1);  // non-blocking

        // Sleep for a while to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



// ------------------------------------------------------ Controller --------------------------------------------//

// FUnction to check if we need to start our emergency controller
void Flyappy::emergency_controller()
{   
    // It turns on its emergency controller when it realizes that its too close to the wall and the gap is not large enough 
    if(longest_sequence_ <= emergency_gap_size_ && distance_to_wall_ < 1.0 )
    {
        emergency_ = true; 
    } 
    // stops when the gap is larger= than 44
    else if (longest_sequence_ >= 44) 
    {
        emergency_ = false;
        going_down_ = false;
        going_up_ = false;
    }
}

// Controller for x acceleration, depending on the mode the pid controls different things, either speed or position 
void Flyappy::x_pid()
{   
    // When we are our "normal" mode, we base our requested velocity on a base velocity we request and on how confident we are that we have found a gap
    if(!steady_state_ && !emergency_ && !out_of_bounds_)
    {
        requested_x_velocity_ = wanted_x_velocity_ - (longest_sequence_ - emergency_gap_size_)/100;
        error_x_ = requested_x_velocity_ - velocity_.x; 
    } 
    // In an emergency we just want to stop 
    else if (emergency_ || out_of_bounds_) 
    {
        requested_x_velocity_ = 0; 
        error_x_ = -3 + (requested_x_velocity_ - velocity_.x)*10; 
    } 
    // In the steady state we are faster the closer we are to the middle of our gap and the further away we are from our end goal 
    else if (steady_state_)
    {
        requested_x_velocity_ = wanted_x_velocity_ + (steady_state_goal_position_x_ - state_.x)- abs(error_y_)*5;
        error_x_ = (requested_x_velocity_ - velocity_.x); 
    }

    integral_x_ += error_x_ * dt_; 
    derivative_x_ = (error_x_ - last_error_x_) / dt_; 

    // Derivatives are your best friend, till they are not : ) (especially here 0 noise :D)
    if(derivative_x_ > 100)
    {
        derivative_x_ = 0;
    }

    acceleration_.x = kp_x_ * error_x_ + ki_x_ * integral_x_ + kd_x_ * derivative_x_; 
    last_error_x_ = error_x_;
}

// Controller for y acceleration, similar to our x pid we have a seperate version for the 3 different cases 
void Flyappy::y_pid()
{   
    if(!steady_state_ && !emergency_ && !out_of_bounds_)
    {
        error_y_ = requested_y_position_ - state_.y; 
    } 
    // If there is an emergency, if we are in the norther hemisphere we go down and vice versa, till we find a gap large enough 
    else if (emergency_) 
    {
        if(state_.y < 2 && !going_down_)
        {   
            going_up_ = true;
            requested_y_position_ = 3; 
        } 
        else if(state_.y > 2 && !going_up_) 
        {
            going_down_ = true;
            requested_y_position_ = 1; 
        } 
        else if(state_.y < 1.1) 
        {
            going_down_ = false;
            going_up_ = true; 
        } 
        else if(state_.y > 2.9)
        {
            going_up_ = false;
            going_down_ = true;
        } 
        else 
        {
            requested_y_position_ = last_requested_y_position_;
        }
        last_requested_y_position_ = requested_y_position_;
        error_y_ = requested_y_position_ - state_.y; 
    } 
    else if (steady_state_) 
    {
        error_y_ = (steady_state_goal_position_y_ - state_.y);
    }

    integral_y_ += error_y_ * dt_; 
    derivative_y_ = (error_y_ - last_error_y_) / dt_; 

    // DOnt need to mention it again 
    if(derivative_y_ > 100){
        derivative_y_ = 0;
    }

    acceleration_.y = kp_y_ * error_y_ + ki_y_ * integral_y_ + kd_y_ * derivative_y_; 
    last_error_y_ = error_y_;
}

// Controller running at 120 hz, check flyappy_autonomy_code_node.cpp for more info 
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
        } 
        else 
        {
            update_dt();
            state_estimation(); 

            // If everything is calibrated and the state estimation is estimating, RUN THE BABY SLAM!!!!! 
            if(baby_slam_run())
            {
                baby_slam(); 
            } 
            else 
            {
                is_running_ = false; 
            }

            // my favorite part 
            y_pid();
            x_pid();

            last_distance_to_wall_ = distance_to_wall_; 
        }

    }
    // Update clock 
    previous_time_ = current_time_; 
}



