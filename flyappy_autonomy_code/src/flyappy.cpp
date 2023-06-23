#include "flyappy_autonomy_code/flyappy.hpp"



Flyappy::Flyappy()
{
    last_time_ = std::chrono::high_resolution_clock::now();
     
}

void Flyappy::set_lidar_data(float angle_min, float angle_max, float angle_increment, float time_increment,
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

void Flyappy::set_vx_data(float vel_x, float vel_y)
{
    vel_.x = vel_x; 
    vel_.y = vel_y; 
}

// Controller running at 60 hz
void Flyappy::threadloop()
{   
    track_velocity();
    if(!lidar_data_.ranges.empty() && test_) ///< Makes sure the controller does not start running before there is any lidar data -> would crash the code
    {   
        control_data_ = lidar_data_.ranges; 
        control_data_intens_ = lidar_data_.intensities; 
        test_ = true; 
        if((get_x_value(4, control_data_[4] <= 1.5) || (get_x_value(3, control_data_[3] <= 1.5) || (get_x_value(5, control_data_[5] <= 1.5)))))
        {
            start_slam_ = true;
        }
        if(start_slam_)
        {
            baby_slam();
        }
        find_the_gap();
        
        
    } 
}

void Flyappy::baby_slam_interpolation()
{
    int _rounded_height = (int) map_height_;
    std::cout << _rounded_height << std::endl;

    map_1D_ = std::vector<int>(map_accuracy_, 3); 
}

void Flyappy::baby_slam_distance_to_wall()
{
    double _total_x_distance = 0;
    double _counter = 0;
    for(unsigned int i = 0; i < control_data_.size(); i++)
    {
        if(get_x_value(i, control_data_[i]) <= 1.5)
        {
            _total_x_distance += get_x_value(i, control_data_[i]);
            _counter += 1; 
        }
    }
    distance_to_wall_ = _total_x_distance/_counter;
    // std::cout << "distance to the wall: " << distance_to_wall_ << std::endl; 
}

void Flyappy::slam_reset()
{   
    // std::cout << "dist to wall: " << distance_to_wall_ << std::endl;
    if(distance_to_wall_ < 0.3 && (abs(current_height_ - requested_y_pos_) < 0.1)){
        // std::cout << "IM RESETTING" << std::endl;
        // std::cout << "IM RESETTING" << std::endl;
        // std::cout << "IM RESETTING" << std::endl;
        map_1D_ = std::vector<int>(map_accuracy_, 3); //reset slam for next place
        // _longest_sequence = 0;
    }
}

void Flyappy::baby_slam_update_map()
{
    for(unsigned int i = 0; i < control_data_.size(); i++)
    {   
        // if(control_data_intens_[i]){
        double _point_location = current_height_ + get_y_value_at_wall(i, control_data_[i]);
        int _map_location = (int) (_point_location * 100);
        if(_map_location < 0){
            _map_location = 0;
        } else if (_map_location > 403){
            _map_location = 403;
        }

        if(abs(get_x_value(i, control_data_[i]) - distance_to_wall_) < 0.5){
            if (map_1D_[_map_location] != 0){
                map_1D_[_map_location] = 1; 
            }
            // if(i == 3)
            //     std::cout << "rock" << std::endl;
        } else {
            // std::cout << "do i even go in here" << std::endl;
            map_1D_[_map_location] = 0;
            // std::cout << "i got maybe" << std::endl;
            // if(i == 3)
            //     std::cout << "air" << std::endl;
        }
        // std::cout << "i got here" << std::endl;
        // if(i == 3){
        //     std::cout << get_x_value(i, control_data_[i]) << " x - distance " << std::endl;
        //     std::cout << current_height_ << " " << get_y_value_at_wall(i, control_data_[i]) << std::endl;
        //     std::cout << "i: " << i << " point location: " << _point_location << "map location: " << _map_location << std::endl;
        // }
        //     std::cout << current_height_ << " " << get_y_value(i, control_data_[i]) << std::endl;
        //     std::cout << "i: " << i << " point location: " << _point_location << "map location: " << _map_location << std::endl;
        // }
    }
    // for (int valuee : map_1D_) {
    //     std::cout << valuee << ' ';
    // }
    // std::cout << '\n';
    // std::cout << " " << std::endl;
}

void Flyappy::baby_slam()
{
    if((control_data_[0] < 3) && (control_data_[8] < 3) && get_height_){
        map_height_ = get_abs_y_value(0, control_data_[0]) + get_abs_y_value(8, control_data_[8]);
        baby_slam_interpolation();
        // std::cout << map_height_ << " test" << std::endl;
        get_height_ = false; 
    } 

    if(map_height_ != 0){
        baby_slam_distance_to_wall();
        baby_slam_update_map(); 
        slam_reset();
    }
}




void Flyappy::track_velocity()
{
    current_time_ = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = current_time_ - last_time_;
    double dt = diff.count();

    if((dt < 1.0/30.0 * 1/2) || (dt > 1.0/30.0 * 1.5)){
        dt = 1.0/30.0;
        // std::cout << "small initialization erro" << std::endl;
    }

    dt_ = dt;

    current_height_ += vel_.y * dt; 

    current_distance_ += vel_.x * dt;

    // std::cout << current_height_<< " <- current height " << track_this_height_ << std::endl;
    // std::cout << current_distance_ << " current distance " << std::endl;


    last_time_ = current_time_;


    // std::cout << "v_x_: " << v_x_ << " accel: " << send_command_x_ << " dt: " << dt << std::endl;
}



void Flyappy::find_the_gap()
{

    // control_data_ = lidar_data_.ranges; 

    if(!got_start_height_)
    {
        current_height_ = get_abs_y_value(0, control_data_[0]);
        // std::cout << current_height_<< " <- starting height " << std::endl;
        got_start_height_ = true;
    }

    controller_x_acceleration();
    controller_y_acceleration_slam(); 
    // controller_y_acceleration();
 
}



bool Flyappy::ready_to_zoom()
{
    // get_abs_y_value(0, get_acontrol_data_[0])
    if(((get_abs_y_value(0, control_data_[0]) < threashold_) && (get_abs_y_value(8, control_data_[8]) < threashold_)) && ((control_data_[0] > 0.2) && (control_data_[8] > 0.2)) && (control_data_[4] > threashold_) && !zoomer_)
    {
        track_this_height_ = current_height_;
        last_distance_ = current_distance_; 
        // map_1D_ = std::vector<int>(map_accuracy_, 3); //reset slam for next place
        return true;
    } else {
        return false;
    }
}

void Flyappy::controller_x_acceleration()
{   

    requested_v_x_ = (control_data_[3] + control_data_[4] + control_data_[5])/15;

    zoom_ = ready_to_zoom();

    if(zoom_)
    {
        // std::cout << "i entered zoom mode" << std::endl;
        // requested_v_x_ = 1; 
        zoom_ = false;
        zoomer_ = true;
        resetting_ = true; 
        zoom_timer_start_ = current_time_;
        // std::cout << v_y_ << std::endl;
    }

    std::chrono::duration<double> diff_gg = current_time_ - zoom_timer_start_;

    double zoom_timer_ = diff_gg.count();

    

    if((current_distance_ - last_distance_) < 0.8 && resetting_){
        // requested_v_x_ = 1; 
        // std::cout << "distance left till no more zoom " << current_distance_ - last_distance_ << std::endl;
        // std::cout << "still zooming" << std::endl;
        // std::cout << v_y_ << std::endl;
        // std::cout << current_height_<< " <- current height " << track_this_height_ << std::endl;
        // if((current_distance_ - last_distance_) < 1){
        //     // requested_v_x_ = 2;
        // }
        // std::cout << "i should be flying stright: " << current_distance_ - last_distance_ << std::endl;
    } else if (resetting_){
        std::cout << "end of reset im resetting all values!!! " << std::endl;
        zoomer_ = false;
        resetting_ = false; 
        decided_reset_value_ = false;
    }

    error_v_ = requested_v_x_ - vel_.x;

    // std::cout << "eroor_v " << error_v_ << std::endl;

    integral_v_ += error_v_ * dt_; 

    derivative_v_ = (error_v_ - previous_error_v_) / dt_; 

    send_command_x_ = kp_v_ * error_v_ + ki_v_ * integral_v_ + kd_v_ * derivative_v_; 

    if(send_command_x_ > 3){
        send_command_x_ = 3;
    }
    if(send_command_x_ < -3){
        send_command_x_ = -3;
    }

    // std::cout << "requested v_x " << requested_v_x_ << " current vel: " << v_x_ << " accel: " << send_command_x_ << std::endl;

    previous_error_v_ = error_v_; 
}

void Flyappy::longest_sequence()
{   
    double _sequence = 0; 
    double _longest_sequence = 0; 
    // double start_bottom_ = 0; 
    // double end_top_ = 0;

    for(unsigned int i = 0; i < map_1D_.size(); i++)
    {
        if(map_1D_[i] == 0)
        {
            _sequence +=1;      
        }
        else if ((map_1D_[i] == 1 || map_1D_[i] == 3) && i > 0 && i < map_1D_.size() - 1 && map_1D_[i-1] == 0 && map_1D_[i+1] == 0)
        {
            // Include a 1 or 3 if they are sandwiched between 0s
            _sequence += 1;
        }
        else 
        {
            if (_sequence > _longest_sequence) {
                _longest_sequence = _sequence; 
                end_top_ = i -1; 
                start_bottom_ = i - _sequence; 
            }
            _sequence = 0; // Sequence ended, so reset
        }
    }
    
    // Check for the longest sequence one last time after the loop ends
    if (_sequence > _longest_sequence) {
        _longest_sequence = _sequence; 
        end_top_ = map_1D_.size() - 1; 
        start_bottom_ = map_1D_.size() - _sequence;
    }

    // std::cout << "longest sequence " << _longest_sequence << std::endl; 
    // std::cout << "start and stop height: " << start_bottom_ << " " << end_top_ << std::endl;
}


// void Flyappy::longest_sequence()
// {   
//     double _sequence = 0; 
//     double _longest_sequence = 0; 

//     for(unsigned int i = 0; i < map_1D_.size(); i++)
//     {
//         // std::cout << "hello" << std::endl;
//         if(map_1D_[i] == 0)
//         {
//             _sequence +=1;      
//         }
//         else if ((i != 0) && (i != 402)){
//             if ((map_1D_[i] == 3 || map_1D_[i] == 1) && (map_1D_[i-1] == 0) && (map_1D_[i+1] == 0))
//                 _sequence += 1;
//         }
//         else 
//         {
//             _sequence = 0; // Sequence ended, so reset
//         }
        
        
//         if (_sequence > _longest_sequence) {
//             _longest_sequence = _sequence; 
//             end_top_ = i -1; 
//             start_bottom_ = i - 1 - _longest_sequence; 
//         }
        
//     }

//     // std::cout << "hello2" << std::endl;

//     // std::cout << "longest sequence " << _longest_sequence << std::endl; 
//     std::cout << "start and stop height: " << start_bottom_ << " " << end_top_ << std::endl;
// }

void Flyappy::controller_y_acceleration_slam()
{   
    longest_sequence(); 

    if(resetting_ && !decided_reset_value_){
        requested_y_pos_ = current_height_;
        std::cout << "i should be flying one height: " << requested_y_pos_ << std::endl;
        decided_reset_value_ = true; 
    } else if(!resetting_){
        requested_y_pos_ = (end_top_ + start_bottom_)/200.0;
    }
    
    if(requested_y_pos_ == 0){
        requested_y_pos_ = 2; 
    }
    std::cout << "current height: " << current_height_ << " wanted height not zooming " << requested_y_pos_ << std::endl;
    error_ = requested_y_pos_ - current_height_;
    // std::cout << requested_v_y_ << std::endl;
        
    integral_ += error_*dt_;


    derivative_ = (error_ - previous_error_) / dt_; 

    send_command_y_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;

    // send_command_y_ = WeightedMovingAverageFilter(WeightedMovingAverageFilterData_, weighted_y_acceleration_command_);

    previous_error_ = error_;

    previous_error_u_ = 0;

    
}


float Flyappy::get_y_value_at_wall(unsigned int num, float value)
{   
    double factor = distance_to_wall_/get_x_value(num, value); // factor to get correct y value for the "gap" as if i just take the normal y value its worong too far up or down
    return sin(start_angle_ + num*increment_angle_)*value*factor;
}

float Flyappy::get_abs_y_value(unsigned int num, float value)
{
    return abs(sin(start_angle_ + num*increment_angle_)*value);
}

float Flyappy::get_y_value(unsigned int num, float value)
{
    return sin(start_angle_ + num*increment_angle_)*value;
} 

float Flyappy::get_x_value(unsigned int num, float value)
{
    return cos(start_angle_ + num*increment_angle_)*value;
}

float Flyappy::get_squared_y_value(unsigned int num, float value)
{
    // if(value < 0.3 && (num == 3 || num == 4 || num == 5))
    // {
    //     std::cout << "im too close: " << num << " " << value << std::endl;
    //     return -((sin(start_angle_ + num*increment_angle_)*value));
    // } else {
    //     return pow(sin(start_angle_ + num*increment_angle_)*value,2);
    // }
    return pow(sin(start_angle_ + num*increment_angle_)*value,2);
}

float Flyappy::get_weighted_squared_y_value(unsigned int num, float value1, float value2, float value3)
{
    if(num == 0) {
        return pow(((sin(start_angle_ + (num)*increment_angle_)*value1) + (sin(start_angle_ + (num+1)*increment_angle_)*value2)/3 + (sin(start_angle_ + (num+2)*increment_angle_)*value3)/3),2)/3;
    } else if (num == 8) {
        return pow(((sin(start_angle_ + (num)*increment_angle_)*value1) + (sin(start_angle_ + (num-1)*increment_angle_)*value2)/3 + (sin(start_angle_ + (num-2)*increment_angle_)*value3)/3),2)/3;
    } else {
    return pow(((sin(start_angle_ + (num - 1)*increment_angle_)*value1)/3 + (sin(start_angle_ + num*increment_angle_)*value2) + (sin(start_angle_ + (num+1)*increment_angle_)*value3)/3),2)/3;
    }
}

float Flyappy::WeightedMovingAverageFilter(std::vector<double> vec, double value) // source singals & systems summary from HS 2022 & https://www.investopedia.com/ask/answers/071414/whats-difference-between-moving-average-and-weighted-moving-average.asp
{
    double weighted_moving_average = 0;
    double n = vec.size();

    vec.pop_back(); // remove last element of the list 
    vec.insert(vec.begin(), value);

    for (unsigned int i = 0; i < vec.size(); i++)
    {
        weighted_moving_average += (n - i)*vec[i];
    }
    return weighted_moving_average/((n*(n+1)/2));
}

double Flyappy::get_y_acceleration()
{
    // std::cout << send_command_y_ << std::endl;
    return send_command_y_; 
}

double Flyappy::get_x_acceleration()
{
    // if (send_command_x_ <= 0)
    // {
    //     send_command_x_ = 0.05;
    // }
    // std::cout << v_x_ << std::endl;
    return send_command_x_; 
}

