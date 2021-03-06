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

#include "carsetupcomhandle.h"
#include "hybrid_astar.h"

//WORK ON: apparently when debugging, there is no definiton of pdef_ and checkValidity() function, which are both true, but for some reason
//I cannot find the equivalents in the past code 
namespace ompl{
    hybridASTAR::hybridASTAR(const base::SpaceInformationPtr &si) : base::Planner(si, "hybrid astar"){
      specs_.approximateSolutions = true;
      specs_.optimizingPaths = true;
      specs_.canReportIntermediateSolutions = false;
    }
    hybridASTAR::~hybridASTAR(void){
      hybridASTAR::freeMemory();
    }

    base::PlannerStatus hybridASTAR::solve(const base::PlannerTerminationCondition &ptc){
      checkValidity(); //not declared in this scope
      bool PATH_FOUND = false;        
      std::vector<base::Path *> open;
      std::vector<base::State *> closed;   
      std::vector<double> cost_vector;
      //Debugging open and closed
      // std::cout << "-------------------debug-----------------" << std::endl;
      // std::cout << "size of open = " << open.size() << std::endl;
      // std::cout << "size of closed = " << closed.size() << std::endl;
      // std::cout << "----------------debug end-----------------" << std::endl;

      //Initialize Goal States
      base::State* goal = pdef_->getGoal().get()->as<base::GoalState>()->getState(); //pdef_ not declared in this scope
      if(pdef_->getStartStateCount() == 0){
        OMPL_ERROR("%s: There are no valid initial states", getName().c_str()); //getName is not declared
        return base::PlannerStatus::INVALID_START;
      } else if(pdef_->getStartStateCount() > 1){
        OMPL_ERROR("%s: There are too many valid initial states", getName().c_str());
        return base::PlannerStatus::INVALID_START;
      }

      //Initialize Start States
      base::State *start = pdef_->getStartState(pdef_->getStartStateCount()-1);

      //DEBUG: Currently working fine
      double goal_x = goal->as<base::SE2StateSpace::StateType>()->getX();
      double goal_y = goal->as<base::SE2StateSpace::StateType>()->getY();
      double goal_theta = goal->as<base::SE2StateSpace::StateType>()->getYaw();
      
      double start_x = start->as<base::SE2StateSpace::StateType>()->getX();
      double start_y = start->as<base::SE2StateSpace::StateType>()->getY();
      double start_theta = start->as<base::SE2StateSpace::StateType>()->getYaw();

      std::cout << "-------------------debug-----------------" << std::endl;
      std::cout << "start state:" << start_x << ", " << start_y << ", " << start_theta << std::endl;
      std::cout << "goal state:" << goal_x << ", " << goal_y << ", " << goal_theta << std::endl;
      std::cout << "----------------debug end-----------------" << std::endl;


      //The open and closed states are slightly different from before as they are now complete 
      //paths instead of just single points
      base::OptimizationObjectivePtr obj = std::make_shared<base::PathLengthOptimizationObjective>(si_); 
      base::Path *current_path(si_);
      base::Path *next_path(si_);
      
      //Log appears not produce anything beyond this point
      //current_path->as<geometric::PathGeometric>()->append(start); // This line doesn't work
      //double l = current_path->length();
      
      double path_cost = calculate_cost(start, goal, 1);
      cost_vector.push_back(path_cost);
      open.push_back(current_path);
      double s = open.size();
       std::cout << "-------------------debug-----------------" << std::endl; 
       std::cout << "size of current cost after appending start: " << path_cost << std::endl;
      // std::cout << "cost of current path: " << c << std::endl;
       std::cout << "length of open after appending current path: "<< s << std::endl;
      base::State *current_state = 
      start;
      base::State *discrete_state;
      base::State *next_state;
      //While termination condition is false, run the planner
      while(ptc.eval() == false){
        if(open.size() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_str());
          ptc.terminate();
          break;
        } 
         
        int index = 0;
        index = return_lowest_cost_path(cost_vector);
        current_path = open[index];

        //DEBUGGING
        std::cout << "CHECKPOINT 1" << std::endl;
        current_state = current_path->as<geometric::PathGeometric>()->getState(current_path->as<geometric::PathGeometric>()->getStateCount()-1);
        
        open.erase(open.begin() + index);
        cost_vector.erase(cost_vector.begin() + index);
        double current_x = current_state->as<base::SE2StateSpace::StateType>()->getX();
        double current_y = current_state->as<base::SE2StateSpace::StateType>()->getY();
        std::vector<double> disc_coord = return_discrete(current_x, current_y);

        //ANOTHER PROBLEM LINE
        //discrete_state->as<base::SE2StateSpace::StateType>()->setXY(disc_coord[0], disc_coord[1]); //DOESNT WORK
        
        closed.push_back(discrete_state);

        //DEBUGGING
        std::cout << "CHECKPOINT 2" << std::endl;
        
        //Condition of the current state examined is the goal state
        if(state_compare(current_state, goal)){ 
          std::cout << "path found" << std::endl;
          current_path->print(std::cout);
          std::cout << "CHECKPOINT FIN" << std::endl;
          break;
        }
        for(int i = 0; i < heading_changes.size()-1; i++){
          next_state->as<base::SE2StateSpace::StateType>()->setX(
            current_state->as<base::SE2StateSpace::StateType>()->getX() + 
            drive_distance*cos(current_state->as<base::SE2StateSpace::StateType>()->getYaw())
          );
          std::cout << "CHECKPOINT 3.0" << std::endl;
          next_state->as<base::SE2StateSpace::StateType>()->setY(
            current_state->as<base::SE2StateSpace::StateType>()->getY() + 
            drive_distance*sin(current_state->as<base::SE2StateSpace::StateType>()->getYaw())
          );
          std::cout << "CHECKPOINT 3.5" << std::endl;
          next_state->as<base::SE2StateSpace::StateType>()->setYaw(
            current_state->as<base::SE2StateSpace::StateType>()->getYaw()+heading_changes[i]
          );
          
          if(si_->isValid(next_state) && vector_contains(closed, next_state) == false){
            cost = calculate_cost(next_state, goal, i);
            next_path = current_path; 
            std::cout << "CHECKPOINT 4" << std::endl;
            next_path->as<geometric::PathGeometric>()->append(next_state); 
            open.push_back(next_path);
            cost_vector.push_back(cost);
          }
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
      double dis_x = round(x);
      double dis_y = round(y);

      std::vector<double> discrete_coord = {dis_x, dis_y};

      return discrete_coord;
    }

    bool hybridASTAR::vector_contains(std::vector<base::State *> input, base::State *item){
      bool FOUND = false;
      for(int i = 0; i< input.size() -1; i++){
        if(input[i] == item){
          FOUND = true;
          break;
        }
      }
      return FOUND;
    }

    int hybridASTAR::return_lowest_cost_path(std::vector<double> input){
      int index = 0;
      double lowest_cost;
      for(int i = 0; i < input.size(); i++){
        if(i == 0){
          index = i;
          lowest_cost = input[i];
        } else {
          if(input[i] < lowest_cost){
            index = i;
            lowest_cost = input[i];
          }
        }
      }
      return index;
    }

    bool hybridASTAR::state_compare(base::State *input, base::State *goal){
      double x1 = input->as<base::SE2StateSpace::StateType>()->getX();
      double y1 = input->as<base::SE2StateSpace::StateType>()->getY();
      std::vector<double> dis_input = return_discrete(x1, y1);

      double x_goal = goal->as<base::SE2StateSpace::StateType>()->getX();
      double y_goal = goal->as<base::SE2StateSpace::StateType>()->getY();
      std::vector<double> dis_goal = return_discrete(x_goal, y_goal);

      if(dis_input == dis_goal){
        return true;
      } else {
        return false;
      }
    }

    double hybridASTAR::calculate_cost(base::State *start, base::State *goal, int turn_index){
      double distance = euclidean_distance(start, goal);
      double turn_weight = 10;
      if(turn_index != 1){
        distance = distance + turn_weight;
      }
      return distance + drive_distance;
    }

    void hybridASTAR::setup(void) {
        Planner::setup();
        //mtree_.freeMemory();
        //timeflags_.clear();
    }
    void hybridASTAR::clear(void) {

    }
    
    void hybridASTAR::freeMemory() {

    }
}
