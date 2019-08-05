// hybrid_astar.h

#ifndef HYBRIDASTAR
#define HYBRIDASTAR

#include <ompl/base/Planner.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <vector>
#include <cmath>


namespace ompl {
    
    class hybridASTAR : public base::Planner {
        public :

        hybridASTAR(const base::SpaceInformationPtr &si);
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

        bool vector_contains(std::vector<base::State *> input, base::State *item);

        double drive_distance;
        double cost;
        double pi = 3.14159265;
    
        float heuristic;
       
        
    };
    typedef std::shared_ptr<hybridASTAR> hybridASTARPtr;
}

#endif
