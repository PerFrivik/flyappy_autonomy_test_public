#include "flyappy_autonomy_code/flyappy.hpp"



Flyappy::Flyappy()
{

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
    if(!lidar_data_.ranges.empty() && test_) ///< Makes sure the controller does not start running before there is any lidar data -> would crash the code
    {
        test_ = true; 
        find_the_gap();
    } 
}



void Flyappy::find_the_gap()
{

    control_data_ = lidar_data_.ranges; 

    controller_y_acceleration();
 

}

void Flyappy::controller_y_acceleration()
{
    fly_down_value_ = 0;
    fly_up_value_ = 0;

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

    y_acceleration_command_ = fly_up_value_ - fly_down_value_;
    // if(y_acceleration_command_ >= 0){
    //     std::cout << "fly up" << std::endl;
    // } else {
    //     std::cout << "fly down" << std::endl;
    // }
    if(y_acceleration_command_ > 1){
        y_acceleration_command_ = 1;
    }
    if(y_acceleration_command_ < -1){
        y_acceleration_command_ = -1;
    }

    weighted_y_acceleration_command_ = WeightedMovingAverageFilter(WeightedMovingAverageFilterData_, y_acceleration_command_);
    if(weighted_y_acceleration_command_ >= 0){
        std::cout << "fly up weighted "  << weighted_y_acceleration_command_ << std::endl;
    } else {
        std::cout << "fly down weighted " << weighted_y_acceleration_command_ << std::endl;
    }
    
    // std::cout << "down_value: " << fly_down_value_ << " up_value: " << fly_up_value_ << std::endl;
}

float Flyappy::get_squared_y_value(unsigned int num, float value)
{
    // return pow(sin(start_angle_ + num*increment_angle_)*value,2);
    return sin(start_angle_ + num*increment_angle_)*value;

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
    return weighted_y_acceleration_command_; 
}

