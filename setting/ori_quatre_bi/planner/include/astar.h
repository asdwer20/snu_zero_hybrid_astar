#ifndef ASTAR
#define ASTAR

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"

class astar {
    private:
    public:
        nav_msgs::OccupancyGridPtr 
        std::vector<std::vector<double>> generate_map(std::vector<double> start, std::vector<double> goal, nav_msgs::OccupancyGridConstPtr map);
};

#endif