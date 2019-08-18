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
      checkValidity(); 
      std::vector<double> heading_changes = {-pi/4, 0, pi/4};
      bool PATH_FOUND = false;        
      std::vector<geometric::PathGeometric> open;
      std::vector<base::State *> closed;   
      std::vector<double> cost_vector;

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

      //DEBUG: Currently working fine
      double goal_x = goal->as<base::SE2StateSpace::StateType>()->getX();
      double goal_y = goal->as<base::SE2StateSpace::StateType>()->getY();
      double goal_theta = goal->as<base::SE2StateSpace::StateType>()->getYaw();
      
      double start_x = start->as<base::SE2StateSpace::StateType>()->getX();
      double start_y = start->as<base::SE2StateSpace::StateType>()->getY();
      double start_theta = start->as<base::SE2StateSpace::StateType>()->getYaw();

      std::cout << "start state: " << start_x << ", " << start_y << ", " << start_theta << std::endl;
      std::cout << "goal state: " << goal_x << ", " << goal_y << ", " << goal_theta << std::endl;
      //The open and closed states are slightly different from before as they are now complete 
      //paths instead of just single points
      base::OptimizationObjectivePtr obj = std::make_shared<base::PathLengthOptimizationObjective>(si_); 

      geometric::PathGeometric next_path(si_, start);
      geometric::PathGeometric current_path(si_, start);
      
      double path_cost = calculate_cost(start, goal, 1);
      cost_vector.push_back(path_cost);
      open.push_back(current_path);
      double s = open.size();

      base::State *current_state(si_->allocState());
      current_state = start;
      base::State *discrete_state(si_->allocState());
      base::State *next_state(si_->allocState());

      //While termination condition is false, run the planner
      while(ptc.eval() == false){
        if(open.size() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_str());
          return base::PlannerStatus::ABORT;
        } 
         
        int index = 0;
        index = return_lowest_cost_path(cost_vector);
        current_path = open[index];
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

        open.erase(open.begin() + index);
        cost_vector.erase(cost_vector.begin() + index);

        //Convert current state coordinates to discrete
        double current_x = current_state->as<base::SE2StateSpace::StateType>()->getX();
        double current_y = current_state->as<base::SE2StateSpace::StateType>()->getY();
        std::vector<double> disc_coord = return_discrete(current_x, current_y);

        
        double ds_X = disc_coord[0];
        double ds_Y = disc_coord[1];

        discrete_state->as<base::SE2StateSpace::StateType>()->setXY(ds_X, ds_Y); 
        closed.push_back(discrete_state);
        
        
        for(int i = 0; i < heading_changes.size(); i++){
          const auto cs = current_state->as<base::SE2StateSpace::StateType>();
          std::cout << "=========================debug======================" << std::endl;
          std::cout << "current state: " << cs->getX() <<", " << cs->getY() << ", " << cs->getYaw() << std::endl;
          double new_Yaw = cs->getYaw()+heading_changes[i];
          double new_X = cs->getX() + drive_distance*cos(new_Yaw);
          double new_Y = cs->getY() + drive_distance*sin(new_Yaw);
          
          std::cout << "dx = " << drive_distance*cos(new_Yaw) << " dy = " << drive_distance*sin(new_Yaw) << std::endl;
          std::cout << "next possible state: " << new_X <<", " << new_Y << ", " << new_Yaw << std::endl;
          std::cout << "index: " << i << std::endl;

          next_state->as<base::SE2StateSpace::StateType>()->setX(new_X);
          next_state->as<base::SE2StateSpace::StateType>()->setY(new_Y);
          next_state->as<base::SE2StateSpace::StateType>()->setYaw(new_Yaw);
          
          if(si_->isValid(next_state) && vector_contains(closed, next_state) == false){
            cost = calculate_cost(next_state, goal, i);
            next_path = current_path; 
            std::cout << "CHECKPOINT 4" << std::endl;
            next_path.append(next_state); 
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
      for(int i = 0; i< input.size(); i++){
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
      std::cout << "input X: " << x1 << " Y: " << y1 << std::endl;

      double x_goal = goal->as<base::SE2StateSpace::StateType>()->getX();
      double y_goal = goal->as<base::SE2StateSpace::StateType>()->getY();
      std::cout << "goal X: " << x_goal << " Y: " << y_goal << std::endl;

      if(abs(x1 - x_goal) <= 0.5 && abs(y1 - y_goal) <= 0.5){
        return true;
      } else {
        return false;
      }
    }

    double hybridASTAR::calculate_cost(base::State *start, base::State *goal, int turn_index){
      double distance = euclidean_distance(start, goal);
      double turn_weight = 0;
      if(turn_index != 1){
        distance = distance + turn_weight;
      }
      return distance + drive_distance;
    }

    void hybridASTAR::setup(void) {
        Planner::setup();
    }
    void hybridASTAR::clear(void) {

    }
    
    void hybridASTAR::freeMemory() {

    }
}
