#include <ros/ros.h>
#include "carsetupcomhandle.h"

#include <string>
#include <iostream>
#include "ompl/geometric/ScopedState.h"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

int main(int argc, char **argv){
    std::string node_name = "hybrid_astar_planner";

    //Basic ROS setup requirements
    ros::init(argc, argv);
    ros::NodeHandle node_handle;

    //Date input from Rviz
    CarSetupComHandle com_handle;

    ompl::base::StateSpacePtr state_space(
        new ompl::base::ReedsSheppStateSpace);
    ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace>
        state_type(state_space);


}
