#ifndef HYBRIDASTAR
#define HYBRIDASTAR

#include <ompl/base/Planner.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <vector>
#include <cmath>


namespace ompl {
    
    class hybridASTAR : public base::Planner {
        public :
        hybridASTAR(const control::SpaceInformationPtr &si);
        ~hybridASTAR(void) override;
        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        void clear(void) override;
        void setup(void) override;
        void freeMemory();

        double euclidean_distance(base::State *state, base::State *goal);
        double return_discrete(double x, double y);

        std::vector<base::Path *> open;
        std::vector<base::State *> closed;   
        std::vector<double> heading_changes;

        bool vector_contains(std::vector<base::Sate *> input, base::State *item);

        double drive_distance;
        double cost;
    
        float heuristic;
       
        
    };
    typedef std::shared_ptr<hybridASTAR> hybridASTARPtr;
}

#endif
