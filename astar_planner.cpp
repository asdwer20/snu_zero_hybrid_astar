#include <ros/ros.h>
#include <string>
#include <iostream>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
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

    int seq = 0;
    while(ros::ok()){
        //Check if the message was updated
        if(seq < CarSetupComHandle::GetLatestSeq()){
            //Clear the previous state space, update seq, acquire map
            state_space.clear();
            seq = CarSetupComHandle::GetLatestSeq();
            nav_msgs::OccupancyGridConstPtr input_map = CarSetupComHandle::GetMap(map_id, seq); 
            
            //Error exception for empty map
            if(input_map == NULL){
              ROS_ERROR_STREAM("map input is empty");
              break;
            } 
            
            //Set dimensions and bounds for the input map
            map_length = input_map.height;
            map_width = input_map.width;
            ompl::base::RealVectorBounds map_bounds(2);
            map_bounds.setLow(0, -map_length);
            map_bounds.setLow(1, -map_width);
            map_bounds.setHigh(0. map_length);
            map_bounds.setHigh(1, map_width);
            
            //Setup state validity checker using the isStateValid function within 
            //CarSetupComHandle header
            state_space_type.setBounds(map_bounds);
            ompl::base::SpaceInformation *space_info = state_space.getSpaceInformation().get();
            state_space.setStateValidityChecker([input_map, seq, space_info](const ompl::base::State *state)
            
            
            
              
            
        }


    

}
