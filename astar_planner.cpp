#include <ros/ros.h>
#include <string>
#include <iostream>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#icnlude "omple/geometric/SimpleSetup.h"
#include "ompl/geometric/ScopedState.h"
#include "ompl/geometric/planner.h"
#include "carsetupcomhandle.h"

namespace ompl_geo = omple::geometric;

int main(int argc, char **argv){
    std::string node_name = "hybrid_astar_planner";
    ros::init(argc, argv);
    ros::NodeHandle node_handle;
        
    //Setting up StateSpace using the OMPL Library
    //The space is a Reeds Shepps State Space with a custom planner setup
    ompl::base::StateSpacePtr state_space_type(new ompl::base::ReedsSheppStateSpace);
    ompl_geo::SimpleSetup state_space(state_space_type);
    
    //Get map date from Nodehandle
    std::string map_id;
    nh.getParam("/map_id", map_id);
    
    //Date input from Rviz
    CarSetupComHandle com_handle = CarSetupComHandle(argc, argv, node_name);
    com_handle.SimpleSetup();
    
    while(ros::ok()){
        //Check if the message was updated

    

}
