// 2019_planner.cpp

// libraries
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <cmath>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/SimpleSetup.h"
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "carsetupcomhandle.h"
#include "hybrid_astar.h"
#include "core_msgs/ActiveNode.h"
#include "geometry_msgs/PoseArray.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// activeness
bool nodeactivation = true;
// callback for handling activeness
void activecb(core_msgs::ActiveNode::ConstPtr msg) {
    int length = msg->active_nodes.size();
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
    ob::StateSpacePtr space(std::make_shared<ob::SE2StateSpace>());
    og::SimpleSetup ss(space);
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
        
    //Setting up StateSpace using the OMPL Library
    //The space is a Reeds Shepps State Space with a custom planner setup
    ob::SpaceInformationPtr space_info(std::make_shared<ob::SpaceInformation>(space));

    //Get activeness from active_nodes
    ros::Subscriber activenode = nh.subscribe("active_nodes", 1000, activecb);

    //setup planner
    ompl::hybridASTARPtr planner(std::make_shared<ompl::hybridASTAR>(space_info));
    
    //Get map date from Nodehandle
    std::string map_id;
    nh.getParam("/map_id", map_id);
    
    //Date input from Rviz
    CarSetupComHandle comh = CarSetupComHandle(argc, argv, node_name);
    comh.SimpleSetup();
    comh.SetTopicPub<geometry_msgs::PoseArray>("/hybrid_astar");

    int seq = 0;
    while(ros::ok()){
        //Check if the message was updated
        if(CarSetupComHandle::isUpdatedMap()){
            //Clear the previous state space, update seq, acquire map
            ss.clear();
            int gseq = CarSetupComHandle::GetLatestGoalSeq();
            int mseq = CarSetupComHandle::GetLatestMapSeq();
            int sseq = CarSetupComHandle::GetLatestStartSeq();

            nav_msgs::OccupancyGridConstPtr input_map = CarSetupComHandle::GetMap(map_id, mseq); 
            
            //Error exception for empty map
            if(input_map == NULL){
              ROS_ERROR_STREAM("map input is empty");
              break;
            } 
            
            //Set dimensions and bounds for the input map
            int map_length = input_map->info.height;
            int map_width = input_map->info.width;
            ob::RealVectorBounds map_bounds(2);
            map_bounds.setLow(0, -map_length);
            map_bounds.setLow(1, -map_width);
            map_bounds.setHigh(0, map_length);
            map_bounds.setHigh(1, map_width);
            
            ob::ScopedStatePtr start = CarSetupComHandle::GetStart(space, map_id, sseq);
            ob::ScopedStatePtr goal = CarSetupComHandle::GetGoal(space, map_id, gseq);

            /*=================================== WORK ON FROM HERE ============================= */
            //Setup state validity checker using the isStateValid function within 
            //CarSetupComHandle header and bounds
            space->as<ob::SE2StateSpace>()->setBounds(map_bounds);
            ss.setStateValidityChecker([map_id, mseq, space_info](const ob::State *state) {return CarSetupComHandle::isStateValid(map_id, mseq, space, state);});
            
            //setting up rest of the planner and the goal points
            ob::OptimizationObjectivePtr obj = std::make_shared<ob::PathLengthOptimizationObjective>(space_info);
            ss.setOptimizationObjective(obj);
            planner->setup();
            ss.setPlanner(planner);
            ss.setStartAndGoalStates(*start, *goal);
            ss.setup();

            ob::PlannerStatus solved = ss.solve(1);
            //=======================================UNTIL HERE =====================================
            if(solved){
                std::cout << "Path found" << std::endl;
                ob::Path* path = ss.getSolutionPath();
                if(nodeactivation){
                    comh.PublishPath(map_id, mseq, path);
                }
            } else {
                std::cout << "No path found" << std::endl;
            }
        }
        
        ros::spinOnce();
    }

    return 0;
}
