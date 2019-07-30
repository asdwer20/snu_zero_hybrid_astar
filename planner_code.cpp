#include <ompl/base/Planner.h>

#include "CarSetupComHandle.h"
#include "planner_code.h"

namepsace ompl{
    planner_code::hybrid_astar(const base::SpaceInformationPtr &si) : base::Planner(si, "hybrid astar"){
      specs_.approximateSolutions = true;
      specs_.optimizingPaths = true;
      specs_.canReportIntermediateSolutions = false;

      heading_changes = {-pi/4, 0, pi/4};
      open = {}; //a vector of vector<base::Path*> -> a path class contains cost
      closed = {} //same as above
      drive_distance = sqrt(2);
    }
    planner_code::~hybrid_astar(void){
      hybrid_astar::destructor();
    }

    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc){
      checkValidity();
      bool PATH_FOUND = false;

      //Initialize Goal States
      base::Goal *goal = pdef_->getGoal().get()->as<base::GoalState()->getState();
      if(pdef_->getStartStateCount() == 0){
        OMPL_ERROR("%s: There are no valid initial states", getName().c_str());
        return base::PlannerStatus::INVALID_START;
      } else if(pdef_->getStartStateCount() > 1){
        OMPL_ERROR("%s: There are too many valid initial states", getName().c_star());
        return base::PlannerStatus::INVALID_START;
      }

      //Initialize Start States
      base::State *start = pdef_->getStartState(pdef_->getStartStateCount()-1);
      si->setStateValidityChecker(const validity_checker) //implement a state validity checker


      //The open and closed states are slightly different from before as they are now complete 
      //paths instead of just single points
      base::Path *current_path;
      current_path->append(start);
      current_path->cost = euclidian_distance(start) + current_path->length;

      open.pushback(start);
      cost = current_path->cost; //should we use optimizationobjective??

      base::State *current_state = start;
      base::State *discrete_state;

      //While termination condition is false, run the planner
      while(ptc() == false){
        if(open->getStateCount() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_star());
          ptc() = true;
          break;
        } 
        sort_vectors(open); //sort path based on the heuristic distance 
        open.pop_back();
        discrete_state->setXY() = return_discrete(current_state->getX(), current_state->getY());
        closed->append(discrete_state);

        //Condition of the current state examined is the goal state
        if(*current_state == *discrete_state){ //the specific details of this condition must be defined
          //i.e. if the definition of the goal state is an area, how that area will be definied 
          //can result in the change of the comparator here

        }
      }
    }

  }
}
