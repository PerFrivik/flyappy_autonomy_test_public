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
        test_ = true; 

        find_the_gap();
        
    } 
}

void Flyappy::baby_slam()
{
    if(control_data_[0] && )
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

// void Flyappy::track_height()
// {
//     current_height_ += v_y_ * 
// }



void Flyappy::find_the_gap()
{

    control_data_ = lidar_data_.ranges; 

    if(!got_start_height_)
    {
        current_height_ = get_abs_y_value(0, control_data_[0]);
        // std::cout << current_height_<< " <- starting height " << std::endl;
        got_start_height_ = true;
    }

    controller_x_acceleration();
    controller_y_acceleration();
 
}



bool Flyappy::ready_to_zoom()
{
    // get_abs_y_value(0, get_acontrol_data_[0])
    if(((get_abs_y_value(0, control_data_[0]) < threashold_) && (get_abs_y_value(8, control_data_[8]) < threashold_)) && ((control_data_[0] > 0.2) && (control_data_[8] > 0.2)) && (control_data_[4] > threashold_) && !zoomer_)
    {
        track_this_height_ = current_height_;
        last_distance_ = current_distance_; 
        return true;
    } else {
        return false;
    }
}

void Flyappy::controller_x_acceleration()
{   

    requested_v_x_ = (control_data_[3] + control_data_[4] + control_data_[5])/8;

    zoom_ = ready_to_zoom();

    if(zoom_)
    {
        std::cout << "i entered zoom mode" << std::endl;
        // requested_v_x_ = 1; 
        zoom_ = false;
        zoomer_ = true;
        zoom_timer_start_ = current_time_;
        // std::cout << v_y_ << std::endl;
    }

    std::chrono::duration<double> diff_gg = current_time_ - zoom_timer_start_;

    double zoom_timer_ = diff_gg.count();

    

    if((current_distance_ - last_distance_) < 0.8 && zoomer_){
        // requested_v_x_ = 1; 
        // std::cout << "distance left till no more zoom " << current_distance_ - last_distance_ << std::endl;
        // std::cout << "still zooming" << std::endl;
        // std::cout << v_y_ << std::endl;
        std::cout << current_height_<< " <- current height " << track_this_height_ << std::endl;
        // if((current_distance_ - last_distance_) < 1){
        //     // requested_v_x_ = 2;
        // }
    } else {
        zoomer_ = false;
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

void Flyappy::controller_y_acceleration()
{
    fly_down_value_ = 0;
    fly_up_value_ = 0;


    if(explore_){
        for(unsigned int i = 0; i < control_data_.size(); i++)
        {
            if(i < 4)
            {
                // fly_down_value_ += get_squared_y_value(i, control_data_[i]);
                if(i == 0){
                    fly_down_value_ += get_weighted_squared_y_value(i, control_data_[i], control_data_[i+1], control_data_[i+2]);
                } else {
                    fly_down_value_ += get_weighted_squared_y_value(i, control_data_[i-1], control_data_[i], control_data_[i+1]);
                }
            }
            if(i > 4)
            {
                // fly_up_value_ += get_squared_y_value(i, control_data_[i]);
                if(i == 8){
                    fly_up_value_ += get_weighted_squared_y_value(i, control_data_[i], control_data_[i-1], control_data_[i-2]);
                } else {
                    fly_up_value_ += get_weighted_squared_y_value(i, control_data_[i-1], control_data_[i], control_data_[i+1]);
                }
            }
        }
    }

    if(!zoomer_)
    {
        requested_v_y_ = fly_up_value_ - fly_down_value_;
        error_ = requested_v_y_ - vel_.y;
        // std::cout << requested_v_y_ << std::endl;
            
        integral_ += error_*dt_;

        // std::cout << "integral: " << integral_ << std::endl;

        derivative_ = (error_ - previous_error_) / dt_; 

        // std::cout << "derivative: " << derivative_ << " " << error_ - previous_error_ << " " << dt_ << std::endl;
        

        weighted_y_acceleration_command_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;

        send_command_y_ = WeightedMovingAverageFilter(WeightedMovingAverageFilterData_, weighted_y_acceleration_command_);

        std::cout << weighted_y_acceleration_command_ << std::endl;

        // std::cout << "not zooming" << std::endl;

        previous_error_ = error_;

        previous_error_u_ = 0;

    } else {
        // std::cout << "since im zooming im flying straight" << " " << v_y_ << std::endl;
        error_ = track_this_height_ - current_height_; 
        std::cout << "current height: " << current_height_ << " wanted height " << track_this_height_ << std::endl;
        // std::cout << "current error: " << error_ << std::endl;

        integral_ += error_*dt_;

        // std::cout << "integral: " << integral_ << std::endl;

        derivative_ = (error_ - previous_error_u_) / dt_; 

        previous_error_u_ = error_;

        std::cout << "derivative: " << derivative_ << " " << error_ - previous_error_u_ << " " << dt_ << std::endl;
        

        send_command_y_ = kp1_ * error_ + ki1_ * integral_ + kd1_ * derivative_;

        previous_error_ = 0;

        std::cout << send_command_y_ << " acceleration " << std::endl;
    }


}

float Flyappy::get_abs_y_value(unsigned int num, float value)
{
    return abs(sin(start_angle_ + num*increment_angle_)*value);
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

