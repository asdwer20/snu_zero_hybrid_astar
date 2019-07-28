#include <ompl/base/Planner.h>

#include "CarSetupComHandle.h"
#include "planner_code.h"

namepsace ompl{
    planner_code::hybrid_astar(const base::SpaceInformationPtr &si) : base::Planner(si, "hybrid astar"){
      specs_.approximateSolutions = true;
      specs_.optimizingPaths = true;
      specs_.canReportIntermediateSolutions = false;
      specs_.
      drive_distance = 
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


      //While termination condition is false, run the planner
      while(ptc() == false){ 

      }
    }

  }
}
