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

// Controller running at 60 hz
void Flyappy::threadloop()
{   
    track_velocity();
    if(!lidar_data_.ranges.empty() && test_) ///< Makes sure the controller does not start running before there is any lidar data -> would crash the code
    {
        test_ = true; 
        find_the_gap();
        
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

    v_x_ += send_command_x_*dt;

    last_time_ = current_time_;

    // std::cout << "v_x_: " << v_x_ << " accel: " << send_command_x_ << " dt: " << dt << std::endl;
}



void Flyappy::find_the_gap()
{

    control_data_ = lidar_data_.ranges; 

    controller_x_acceleration();
    controller_y_acceleration();
 
}

bool Flyappy::ready_to_zoom()
{
    if((control_data_[0] < threashold_) && (control_data_[8] < threashold_) && (control_data_[4] > threashold_))
    {
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
        std::cout << "i entered zoom mode" << std::endl;
        requested_v_x_ = 10; 
        zoom_ = false;
        zoom_timer_start_ = current_time_;
    }

    std::chrono::duration<double> diff_gg = current_time_ - zoom_timer_start_;

    double zoom_timer_ = diff_gg.count();

    if(zoom_timer_ < zoom_total_time_){
        requested_v_x_ = 10; 
        std::cout << "still zooming" << std::endl;
    }

    error_v_ = requested_v_x_ - v_x_;

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

    std::cout << "requested v_x " << requested_v_x_ << "accel: " << send_command_x_ << std::endl;

    previous_error_v_ = error_v_; 
}

void Flyappy::controller_y_acceleration()
{
    fly_down_value_ = 0;
    fly_up_value_ = 0;


    if(explore_){
        for(unsigned int i = 0; i < control_data_.size(); i++)
        {
            if(i < 4)
            {
                fly_down_value_ += get_squared_y_value(i, control_data_[i]);
            }
            if(i > 4)
            {
                fly_up_value_ += get_squared_y_value(i, control_data_[i]);
            }
        }
    }



    error_ = fly_up_value_ - fly_down_value_;

    // std::cout << "error: " << error_ << std::endl;

    integral_ += error_*dt_;

    // std::cout << "integral: " << integral_ << std::endl;

    derivative_ = (error_ - previous_error_) / dt_; 

    // std::cout << "derivative: " << derivative_ << " " << error_ - previous_error_ << " " << dt_ << std::endl;
    

    weighted_y_acceleration_command_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;

    send_command_y_ = WeightedMovingAverageFilter(WeightedMovingAverageFilterData_, weighted_y_acceleration_command_);

    // std::cout << weighted_y_acceleration_command_ << std::endl;

    previous_error_ = error_;

    // weighted_y_acceleration_command_ = error_;
    // weighted_y_acceleration_command_ = WeightedMovingAverageFilter(WeightedMovingAverageFilterData_, error_);

}

float Flyappy::get_squared_y_value(unsigned int num, float value)
{
    return pow(sin(start_angle_ + num*increment_angle_)*value,2);
    // return sin(start_angle_ + num*increment_angle_)*value;

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

