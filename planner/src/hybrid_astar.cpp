// hybrid_astar.cpp

#include <ompl/base/Planner.h>
#include <cmath>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/Path.h>

#include "carsetupcomhandle.h"
#include "hybrid_astar.h"

//WORK ON: apparently when debugging, there is no definiton of pdef_ and checkValidity() function, which are both true, but for some reason
//I cannot find the equivalents in the past code 
namespace ompl{
    hybridASTAR::hybridASTAR(const base::SpaceInformationPtr &si) : base::Planner(si, "hybrid astar"){
      specs_.approximateSolutions = true;
      specs_.optimizingPaths = true;
      specs_.canReportIntermediateSolutions = false;

      heading_changes = {-pi/4, 0, pi/4};
      open = {}; //a vector of vector<base::Path *> -> a path class contains cost
      closed = {}; //a vector of vector<base::State *>
      drive_distance = sqrt(2);
    }
    hybridASTAR::~hybridASTAR(void){
      hybridASTAR::freeMemory();
    }

    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc){
      checkValidity();
      bool PATH_FOUND = false;

      //Initialize Goal States
      base::State* goal = pdef_->getGoal().get()->as<base::GoalState>()->getState(); //WORKON: apparently the declaration in the line isnt corrent i.e. the <> ,(), placements
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
      base::Path *current_path;
      base::Path *next_path;
      current_path->as<geometric::PathGeometric>()->append(start); //WORKON: append function does not exist?
      current_path->cost = euclidean_distance(*start) + current_path->length;

      open.pushback(start); //WORKON: open not declared in this scope
      cost = current_path->cost; //should we use optimizationobjective??

      base::State *current_state = start;
      base::State *discrete_state;
      base::State *next_state;
      //While termination condition is false, run the planner
      while(ptc() == false){
        if(open->size() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_star());
          ptc() = true;
          break;
        } 
        sort_vectors(open); //sort path based on the heuristic distance 
        current_path = open.back();
        current_state = current_path->as<geometric::PathGeometric>()->getState(current_path->getStateCount()-1);
        open.pop_back();
        
        std::vector<double> disc_coord = return_discrete(
          current_state->as<base::SE2StateSpace::StateType>()->getX(), 
          current_state->as<base::SE2StateSpace::StateType>()->getY()
        );

        discrete_state->as<base::SE2StateSpace::StateType>()->setXY(disc_coord[0], disc_coord[1]);

        closed->append(discrete_state);

        //Condition of the current state examined is the goal state
        if(*current_state == *discrete_state){ //the specific details of this condition must be defined
          //i.e. if the definition of the goal state is an area, how that area will be definied 
          //can result in the change of the comparator here
          current_path->print(std::ostream &out);
          ptc() = true;
          std::cout << "path found";
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
        if(isValid(next_state) && vector_contains(closed, next_state) == false){
          heuristic = euclidean_distance(next_state, goal) + drive_distance;
          *next_path = *current_path; //this is done because the function passes by reference
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
      for(int i = 0; i< input->size() -1; i++){
        if(input[i] == item){
          FOUND = true;
          break;
        }
      }
      return FOUND;
    }

     void hybridASTAR::freeMemory() {
        current_path->clear();
        next_path->clear();
        open->clear();
        closed->clear();
    }
  }
}
