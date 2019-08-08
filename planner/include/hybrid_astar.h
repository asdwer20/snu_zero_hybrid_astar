// hybrid_astar.h

#ifndef HYBRIDASTAR
#define HYBRIDASTAR

#include <ompl/base/Planner.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/State.h>
#include <vector>
#include <cmath>


namespace ompl {
    
    class hybridASTAR : public base::Planner {
        public :

        hybridASTAR(const base::SpaceInformationPtr &si);
        ~hybridASTAR(void) override;
        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        double drive_distance = sqrt(2);
        double cost;
        double pi = 3.14159265;
        
        void clear(void) override;
        void setup(void) override;
        void freeMemory();

        double euclidean_distance(base::State *state, base::State *goal);
        
        std::vector<double> return_discrete(double x, double y);
        std::vector<double> heading_changes = {-pi/4, 0, pi/4};
        std::vector<base::Path *> sort_vectors(std::vector<base::Path *> input);
        
        bool compare_path_costs(const base::Path &v1, const base::Path &v2);
        bool vector_contains(std::vector<base::State *> input, base::State *item);
        bool state_compare(base::State* input, base::State* goal);
    
        float heuristic;
       
        
    };
    typedef std::shared_ptr<hybridASTAR> hybridASTARPtr;
}

#endif
