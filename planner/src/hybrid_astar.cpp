// hybrid_astar.cpp

#include <ompl/base/Planner.h>
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
#include <ompl/tools/config/SelfConfig.h>

#include "carsetupcomhandle.h"
#include "hybrid_astar.h"

//WORK ON: apparently when debugging, there is no definiton of pdef_ and checkValidity() function, which are both true, but for some reason
//I cannot find the equivalents in the past code 
namespace ompl{
    hybridASTAR::hybridASTAR(const base::SpaceInformationPtr &si) : base::Planner(si, "hybrid astar"){
      specs_.approximateSolutions = true;
      specs_.optimizingPaths = true;
      specs_.canReportIntermediateSolutions = false;

      open = {}; //a vector of vector<base::Path *> -> a path class contains cost
      closed = {}; //a vector of vector<base::State *>
    }
    hybridASTAR::~hybridASTAR(void){
      hybridASTAR::freeMemory();
    }

    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc){
      checkValidity(); //not declared in this scope
      bool PATH_FOUND = false;

      //Initialize Goal States
      base::State* goal = pdef_->getGoal().get()->as<base::GoalState>->getState(); //pdef_ not declared in this scope
      if(pdef_->getStartStateCount() == 0){
        OMPL_ERROR("%s: There are no valid initial states", getName().c_str()); //getName is not declared
        return base::PlannerStatus::INVALID_START;
      } else if(pdef_->getStartStateCount() > 1){
        OMPL_ERROR("%s: There are too many valid initial states", getName().c_star());
        return base::PlannerStatus::INVALID_START;
      }

      //Initialize Start States
      base::State *start = pdef_->getStartState(pdef_->getStartStateCount()-1);

      //The open and closed states are slightly different from before as they are now complete 
      //paths instead of just single points
      base::OptimizationObjectivePtr obj = std::make_shared<base::PathLengthOptimizationObjective>(si); 
      base::Path *current_path;
      base::Path *next_path;
      current_path->as<geometric::PathGeometric>()->append(start); 
      base::Cost path_cost = current_path->cost(obj);
      open.pushback(current_path);
      
      base::State *current_state = start;
      base::State *discrete_state;
      base::State *next_state;
      //While termination condition is false, run the planner
      while(ptc.eval() == false){
        if(open->size() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_star());
          ptc.terminate();
          break;
        } 
        sort_vectors(open); //sort path based on the heuristic distance 
        current_path = open.back();
        current_state = current_path->as<geometric::PathGeometric>()->getState(current_path->as<geometric::PathGeometric>()->getStateCount()-1);
        open.pop_back();
        
        std::vector<double> disc_coord = return_discrete(current_state->as<base::SE2StateSpace::StateType>()->getX(), current_state->as<base::SE2StateSpace::StateType>()->getY());

        discrete_state->as<base::SE2StateSpace::StateType>()->setXY(disc_coord[0], disc_coord[1]);

        closed->pushback(discrete_state);

        //Condition of the current state examined is the goal state
        if(state_compare(current_state, goal)){ 
          std::cout << "path found" << std::endl;
          current_path->print(std::ostream &out);
          break;
        }
        for(int i = 0; i < heading_changes.size()-1; i++){
          next_state->as<base::SE2StateSpace::StateType>()->setX(
            current_state->as<base::SE2StateSpace::StateType>()->getX() + 
            drive_distance*cos(current_state->as<base::SE2StateSpace::StateType>()->getYaw())
          );
          next_state->as<base::SE2StateSpace::StateType>()->setYaw(
            current_state->as<base::SE2StateSpace::StateType>()->getYaw()+heading_changes[i]
          );
        }
        if(si->isValid(next_state) && vector_contains(closed, next_state) == false){
          heuristic = euclidean_distance(next_state, goal) + drive_distance;
          *next_path = *current_path; 
          next_path->push_back(next_state); 
          open->push_back(next_path);
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

    bool compare_path_costs(const base::Path &v1, const base::Path &v2) {
	    return v1->as<geometric::PathGeometric>()->cost()->value() > v2->as<geometric::PathGeometric>()->cost()->value();
    }

    std::vector<base::Path *> hybridASTAR::sort_vectors(std::vector<base::Path *> input){
        std::sort(input.begin(), input.endl(), compare_path_costs);
        return input;
    }

    bool state_compare(base::State *input, base::State *goal){
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

     void hybridASTAR::freeMemory() {
        open.clear();
        closed.clear();
    }
  }
}
