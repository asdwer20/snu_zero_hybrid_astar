#include <ompl/base/Planner.h>

#include "CarSetupComHandle.h"

namepsace ompl;
{
  class hybrid_astar : public   base::Planner
  {
  public:
    virtual ~hybrid_astar(void){
    }

    virtual base::PlannerStatus solve(cost base::PlannerTerminationCondition &ptc){
      checkValidity();
      base::Goal *goal = pdef_->getGoal().get();
      //While termination condition is false, run the planner
      while(ptc() == false){

      }
    }

  }
}
