// hybrid_astar.cpp

#include <ompl/base/Planner.h>
#include <string>
#include <cmath>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <iostream>
#include <vector>
#include <list>
#include <deque>

#include "carsetupcomhandle.h"
#include "hybrid_astar.h"
#include "costpath.h"
#include "binomial_heap.h"
#include "tf2_msgs/TFMessage.h"

#define RESOLUTION 0.03
#define MAXDIST 250
//#define MAX(a,b) (((a)>(b))?(a):(b))

std::string map_id = "car_frame";
namespace ompl{
    hybridASTAR::hybridASTAR(const base::SpaceInformationPtr &si) : base::Planner(si, "hybrid astar"){
      specs_.approximateSolutions = true;
      specs_.optimizingPaths = true;
      specs_.canReportIntermediateSolutions = false;
    }
    hybridASTAR::~hybridASTAR(void){
      hybridASTAR::freeMemory();
    }

    template<typename T> bool valid_state_check(std::vector<T> map, int width, int height, std::vector<int> state) {
      if (state[0] >= width || state[1] >= height) {
        return false;
      } else if (state[0] < 0 || state[1] < 0) {
        return false;
      } else if (map[width*state[1]+state[0]] != 0) {
        return false;
      } else {
        return true;
      } 
    }

    base::PlannerStatus hybridASTAR::solve(const base::PlannerTerminationCondition &ptc){
      checkValidity(); 
      std::vector<double> heading_changes = {-pi/4, -pi/6, -pi/8, 0, pi/8, pi/4};
      bool PATH_FOUND = false;        
      std::vector<std::vector<double>> closed;   
      list<Node*> costpath;
      std::vector<double> dis_goal;

      //Initialize Goal States
      base::State* goal = pdef_->getGoal().get()->as<base::GoalState>()->getState(); 
      if(pdef_->getStartStateCount() == 0){
        OMPL_ERROR("%s: There are no valid initial states", getName().c_str()); 
        return base::PlannerStatus::INVALID_START;
      } else if(pdef_->getStartStateCount() > 1){
        OMPL_ERROR("%s: There are too many valid initial states", getName().c_str());
        return base::PlannerStatus::INVALID_START;
      }

      //Initialize Start States
      base::State *start = pdef_->getStartState(pdef_->getStartStateCount()-1);

      double goal_x = goal->as<base::SE2StateSpace::StateType>()->getX();
      double goal_y = goal->as<base::SE2StateSpace::StateType>()->getY();
      double goal_theta = goal->as<base::SE2StateSpace::StateType>()->getYaw();

      dis_goal = return_discrete(goal_x, goal_y);
      
      double start_x = start->as<base::SE2StateSpace::StateType>()->getX();
      double start_y = start->as<base::SE2StateSpace::StateType>()->getY();
      double start_theta = start->as<base::SE2StateSpace::StateType>()->getYaw();

      //The open and closed states are slightly different from before as they are now complete 
      //paths instead of just single points
      base::OptimizationObjectivePtr obj = std::make_shared<base::PathLengthOptimizationObjective>(si_); 

      geometric::PathGeometric next_path(si_, start);
      geometric::PathGeometric current_path(si_, start);
      
      double path_cost = 0;
      costpath = insert(costpath, Costpath(current_path, path_cost));
      closed.push_back(dis_goal);

      base::State *current_state(si_->allocState());
      current_state = start;
      base::State *discrete_state(si_->allocState());
      base::State *next_state(si_->allocState());

      int mseq = CarSetupComHandle::GetLatestMapSeq();
      nav_msgs::OccupancyGridConstPtr input_map = CarSetupComHandle::GetMap(map_id, mseq);
      std::vector<signed char, std::allocator<signed char>> map1d = input_map->data;
      std::vector<int> map1d2(map1d.begin(), map1d.end());
      int map_width = input_map->info.width;
      int map_height = input_map->info.height;
      std::vector<int> dist(map_width*map_height, MAXDIST);
      std::vector<int> closed_map = map1d2;
      std::deque<std::vector<int>> open_map = {};

      double curr_x = current_state->as<base::SE2StateSpace::StateType>()->getX();
      double curr_y = current_state->as<base::SE2StateSpace::StateType>()->getY();
      double res = input_map -> info.resolution;
      double px = curr_x - input_map -> info.origin.position.x;
      double py = curr_y - input_map -> info.origin.position.y;
      tf2::Quaternion q(input_map -> info.origin.orientation.x, input_map -> info.origin.orientation.y, input_map -> info.origin.orientation.z, input_map -> info.origin.orientation.w);
      double yaw = q.getAngle();
      double fx = cos(yaw) * px + sin(yaw) * py;
      double fy = -sin(yaw) * py + cos(yaw) * py;
      int xj = map_width/2 + (int)(fx/res);
      int yi = map_height/2 + (int)(fy/res);

      std::vector<int> goal_state = { xj, yi };
      if (!valid_state_check(map1d2, map_width, map_height, goal_state)) {
        std::cout << "ERROR: Invalid Goal Pose" << std::endl;
      } else {
        std::cout << "GOOD: Valid Goal Pose" << std::endl;
        open_map.push_back(goal_state);
        dist[goal_state[0]*map_width+goal_state[1]] = 0;
        closed_map[goal_state[0]*map_width+goal_state[1]] = 1;
      }

      while (!open_map.empty()) {
        std::vector<int> state = open_map.front();
        open_map.pop_front();
        std::vector<int> stateL = { state[0]-1, state[1] };
        std::vector<int> stateR = { state[0]+1, state[1] };
        std::vector<int> stateD = { state[0], state[1]-1 };
        std::vector<int> stateU = { state[0], state[1]+1 };
      }

      std::cout << "For Debugging Only: " << dist[0] << " ;" << std::endl;

      //While termination condition is false, run the planner
      while(ptc.eval() == false){
        if(costpath.size() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_str());
          return base::PlannerStatus::ABORT;
        } 
         
        Costpath cpath = getMin(costpath)->data;
        current_path = cpath.path;
        current_state = current_path.getState(current_path.getStateCount()-1);
        
        //Condition of the current state examined is the goal state
        if(state_compare(current_state, goal)){ 
          std::cout << "path found" << std::endl;
          current_path.print(std::cout);
          auto cp(std::make_shared<geometric::PathGeometric>(si_));
          cp->append(current_path);
          base::PlannerSolution psol(cp);
          psol.setPlannerName(getName());
          pdef_->addSolutionPath(psol);
          std::cout << std::endl << std::endl << std::endl;
          return base::PlannerStatus::EXACT_SOLUTION;
        }

        costpath = extractMin(costpath);

        //Convert current state coordinates to discrete
        double current_x = current_state->as<base::SE2StateSpace::StateType>()->getX();
        double current_y = current_state->as<base::SE2StateSpace::StateType>()->getY();
        std::vector<double> disc_coord = return_discrete(current_x, current_y);

        
        double ds_X = disc_coord[0];
        double ds_Y = disc_coord[1];

        discrete_state->as<base::SE2StateSpace::StateType>()->setXY(ds_X, ds_Y); 
        closed.push_back(disc_coord);
        
        for(int i = 0; i < heading_changes.size(); i++){
          const auto cs = current_state->as<base::SE2StateSpace::StateType>();
          //std::cout << "=========================debug======================" << std::endl;
          //std::cout << "current state: " << cs->getX() <<", " << cs->getY() << ", " << cs->getYaw() << std::endl;
          double new_Yaw = cs->getYaw()+heading_changes[i];
          double new_X = cs->getX() + drive_distance*cos(new_Yaw);
          double new_Y = cs->getY() + drive_distance*sin(new_Yaw);
          
          //std::cout << "next possible state: " << new_X <<", " << new_Y << ", " << new_Yaw << std::endl;
          next_state->as<base::SE2StateSpace::StateType>()->setX(new_X);
          next_state->as<base::SE2StateSpace::StateType>()->setY(new_Y);
          next_state->as<base::SE2StateSpace::StateType>()->setYaw(new_Yaw);

          std::vector<double> discrete_next = return_discrete(new_X, new_Y);
        
          if(si_->isValid(next_state) && vector_contains(closed, discrete_next) == false){
            //std::cout << "VALID STATE" << std::endl;
            next_path = current_path; 
            next_path.append(next_state); 

            cost = next_path.length()*drive_distance + euclidean_distance(next_state, goal);
            //std::cout << "NEW COST: " << cost << std::endl;

            costpath = insert(costpath, Costpath(next_path, cost));
          }
          //std::cout << "===============================================" << std::endl;
        }
      }
    }

    double hybridASTAR::euclidean_distance(base::State *state, base::State *goal){
      double x1 = state->as<base::SE2StateSpace::StateType>()->getX();
      double y1 = state->as<base::SE2StateSpace::StateType>()->getY();

      double goalx = goal->as<base::SE2StateSpace::StateType>()->getX();
      double goaly = goal->as<base::SE2StateSpace::StateType>()->getY();

      return sqrt(pow(goalx-x1, 2) + pow(goaly- y1, 2));

    }

    std::vector<double> hybridASTAR::return_discrete(double x, double y){
      double round_x = RESOLUTION * round(x/RESOLUTION);
      double round_y = RESOLUTION * round(y/RESOLUTION);
      std::vector<double> discrete_coord = {round_x, round_y};
      return discrete_coord;
    }

    bool hybridASTAR::vector_contains(std::vector<std::vector<double>> closed, std::vector<double> input){
      bool FOUND = false;
      for(int i = 0; i< closed.size(); i++){
        if(closed[i] == input){
          FOUND = true;
          break;
        }
      }
      return FOUND;
    }

    bool hybridASTAR::state_compare(base::State *input, base::State *goal){
      double x1 = input->as<base::SE2StateSpace::StateType>()->getX();
      double y1 = input->as<base::SE2StateSpace::StateType>()->getY();
      std::cout << "input X: " << x1 << " Y: " << y1 << std::endl;

      double x_goal = goal->as<base::SE2StateSpace::StateType>()->getX();
      double y_goal = goal->as<base::SE2StateSpace::StateType>()->getY();
      //std::cout << "goal X: " << x_goal << " Y: " << y_goal << std::endl;

      if((std::abs(x1 - x_goal)<=0.1) and (std::abs(y1 - y_goal)<=0.1)){
        return true;
      } else {
        return false;
      }
    }

    void hybridASTAR::setup(void) {
        Planner::setup();
    }
    void hybridASTAR::clear(void) {

    }
    
    void hybridASTAR::freeMemory() {

    }
}
