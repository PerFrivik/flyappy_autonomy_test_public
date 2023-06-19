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
    if(!lidar_data_.ranges.empty()) ///< Makes sure the controller does not start running before there is any lidar data -> would crash the code
    {
        find_the_gap();
    } 
}



void Flyappy::find_the_gap()
{

    std::cout << lidar_data_.scan_time << std::endl;
    std::cout << lidar_data_.ranges[0] << std::endl;
 

}