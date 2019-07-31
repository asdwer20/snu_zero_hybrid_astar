#include <ompl/base/Planner.h>

#include "CarSetupComHandle.h"
#include "hybrid_astar.h"

namepsace ompl{
    hybrid_astar::hybrid_astar(const base::SpaceInformationPtr &si) : base::Planner(si, "hybrid astar"){
      specs_.approximateSolutions = true;
      specs_.optimizingPaths = true;
      specs_.canReportIntermediateSolutions = false;

      heading_changes = {-pi/4, 0, pi/4};
      open = {}; //a vector of vector<base::Path*> -> a path class contains cost
      closed = {} //same as above
      drive_distance = sqrt(2);
    }
    hybrid_astar::~hybrid_astar(void){
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
      current_path->cost = euclidean_distance(start) + current_path->length;

      open.pushback(start);
      cost = current_path->cost; //should we use optimizationobjective??

      base::State *current_state = start;
      base::State *discrete_state;
      base::State *next_state;
      //While termination condition is false, run the planner
      while(ptc() == false){
        if(open->getStateCount() == 0){
          OMPL_ERROR("%s: There are no possible paths", getName().c_star());
          ptc() = true;
          break;
        } 
        sort_vectors(open); //sort path based on the heuristic distance 
        current_path = open.back();
        current_state = current_path->getState(current_path->getStateCount()-1);
        open.pop_back();
        discrete_state->setXY(return_discrete(current_state->getX(), current_state->getY()));
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
          next_state->setX(current_state->getX() + drive_distance*cos(current_state->getYaw()));
          next_state->setYaw(current_state->getYaw()+heading_changes[i]);
        }
        if(isValid(next_state) && vector_contains(closed, next_state) == false){
          heuristic = euclidean_distance(next_state, goal) + drive_distance;
          open->push_back(next_state);
        }
      }
    }

  }
}
