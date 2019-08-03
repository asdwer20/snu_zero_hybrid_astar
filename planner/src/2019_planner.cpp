// 2019_planner.cpp

// libraries
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <cmath>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/ScopedState.h"
#include "ompl/geometric/planner.h"
#include "carsetupcomhandle.h"
#include "hybrid_astar.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// activeness
bool nodeactivation = true;
// callback for handling activeness
void activecb(core_msgs::ActiveNode::ConstPtr msg) {
    int lenth = msg->active_nodes.size();
    bool kill = true;
    std::string nn = "path_planner";
    std::string monitor = "zero_monitor";
    for (int i = 0; i < length ; i++) {
        if (nn.compare(msg->active_nodes[i]) == 0) {
            nodeactivation = true;
            std::cout << "node activated" << std::endl;
            return;
        }
        if (monitor.compare(msg->active_nodes[i]) == 0) {
            kill = false;
        }
    }
    nodeactivation = false;
    if (kill) {
        ros::shutdown();
    }
    std::cout << "node deactivated" << std::endl;
    return;
}

int main(int argc, char **argv){
    std::string node_name = "hybrid_astar_planner";
    ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());
    og::SimpleSetup ss(space);
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
        
    //Setting up StateSpace using the OMPL Library
    //The space is a Reeds Shepps State Space with a custom planner setup
    ob::SpaceInformation *space_info = space.getSpaceInformation().get();

    //Get activeness from active_nodes
    ros::Subscriber activenode = nh.subscribe("active_nodes", 1000, activecb);
    
    //Get map date from Nodehandle
    std::string map_id;
    nh.getParam("/map_id", map_id);
    
    //Date input from Rviz
    CarSetupComHandle com_handle = CarSetupComHandle(argc, argv, node_name);
    com_handle.SimpleSetup();
    comh.SetTopicPub<geometry_msgs::PoseArray>("/hybrid_astar");

    //setting up planner 
    ompl::hybridASTAR planner(std::make_shared<ompl::hybridASTAR>(si));

    int seq = 0;
    while(ros::ok()){
        //Check if the message was updated
        if(seq < CarSetupComHandle::GetLatestSeq()){
            //Clear the previous state space, update seq, acquire map
            ss.clear();
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
            ob::RealVectorBounds map_bounds(2);
            map_bounds.setLow(0, -map_length);
            map_bounds.setLow(1, -map_width);
            map_bounds.setHigh(0. map_length);
            map_bounds.setHigh(1, map_width);
            
            //Setup state validity checker using the isStateValid function within 
            //CarSetupComHandle header and bounds
            ss.setBounds(map_bounds);
            ss.setStateValidityChecker([input_map, seq, space_info](const ob::State *state));

            ob::OptimizationObjectivePtr obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
            
              
            
        }


    

}