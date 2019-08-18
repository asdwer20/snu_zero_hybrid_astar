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
      std::vector<geometric::PathGeometric> open;
      std::vector<base::State *> closed;   
      std::vector<double> cost_vector;

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

      //The open and closed states are slightly different from before as they are now complete 
      //paths instead of just single points
      base::OptimizationObjectivePtr obj = std::make_shared<base::PathLengthOptimizationObjective>(si_); 

      auto next_path(std::make_shared<geometric::PathGeometric>(si_));
      auto current_path(std::make_shared<geometric::PathGeometric>(si_));
      
      double path_cost = calculate_cost(start, goal, 1);
      cost_vector.push_back(path_cost);
      open.push_back(current_path);
      double s = open.size();
      
      base::State *current_state(si_->allocState());
      current_state = start;
      base::State *discrete_state(si_->allocState());
      base::State *next_state(si_->allocState());

      std::cout << "CHECKPOINT 1" << std::endl;
      //While termination condition is false, run the planner
      while(ptc.eval() == false){
        if(open.size() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_str());
          ptc.terminate();
          return base::PlannerStatus(true, false);
        } 
         
        int index = 0;
        index = return_lowest_cost_path(cost_vector);
        std::cout << "CHECKPOINT 1.5" << std::endl;
        current_path = open[index];
        std::cout << "CHECKPOINT 2" << std::endl;

        current_state = current_path->as<geometric::PathGeometric>()->getState(current_path->as<geometric::PathGeometric>()->getStateCount()-1);
        
        open.erase(open.begin() + index);
        cost_vector.erase(cost_vector.begin() + index);
       
        double current_x = current_state->as<base::SE2StateSpace::StateType>()->getX();
        double current_y = current_state->as<base::SE2StateSpace::StateType>()->getY();
        std::vector<double> disc_coord = return_discrete(current_x, current_y);
        std::cout << "CHECKPOINT 2.5" << std::endl;

        //This line doesnt work 
        const auto ds = discrete_state->as<base::SE2StateSpace::StateType>();
    
        double ds_X = disc_coord[0];
        double ds_Y = disc_coord[1];
        ds->setXY(ds_X, ds_Y); //THIS DOESNT WORK
        closed.push_back(discrete_state);
        
        //Condition of the current state examined is the goal state
        if(state_compare(current_state, goal)){ 
          std::cout << "path found" << std::endl;
          std::cout << "CHECKPOINT FIN" << std::endl;
          current_path->print(std::cout);

          base::PlannerSolution psol(current_path);
          psol.setPlannerName(getName());
          pdef_->addSolutionPath(psol);

          std::cout << std::endl << std::endl << std::endl;
          
          return base::PlannerStatus(true,true);
        }
        for(int i = 0; i < heading_changes.size()-1; i++){
          std::cout << "CHECKPOINT 3" << std::endl;
          const auto ns = next_state->as<base::SE2StateSpace::StateType>();
          const auto cs = current_state->as<base::SE2StateSpace::StateType>();
          
          double new_X = cs->getX() + drive_distance*cos(cs->getYaw());
          double new_Y = cs->getY() + drive_distance*sin(cs->getYaw());
          double new_Yaw = cs->getYaw()+heading_changes[i];
          ns->setX(new_X);
          std::cout << "CHECKPOINT 3.5" << std::endl;
          ns->setY(new_Y);
          ns->setYaw(new_Yaw);

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
