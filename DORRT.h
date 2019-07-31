#ifndef DORRT
#define DORRT

#include <ompl/base/Planner.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <chrono>
#include <ctime>
#include "magneticmodel.h"
#include "potentialmodel.h"

namespace ompl {
    
    class doRRT : public base::Planner {
        public :
        doRRT(const control::SpaceInformationPtr &si, const MagneticModelPtr &mm, const PotentialModelPtr &pm);
        ~doRRT(void) override;
        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        void setPotMax(double max) {
            potmax_ = max;
        }
        double getPotMax(void) {
            return potmax_;
        }
        void setResolution(double r) {
            res_ = r;
        }
        double getResolution(void) {
            return res_;
        }
        void setFieldNorm(double n) {
            fieldnorm_ = n;
        }
        double getFieldNorm(void) {
            return fieldnorm_;
        }
        void setMaxSample(int n) {
            maxsample_ = n;
        }
        int getMaxSample(void) {
            return maxsample_;
        }
        void setExtendLength(double l) {
            extendlength_ = l;
        }
        double getExtendLength(void) {
            return extendlength_;
        }
        void setApproxLength(double l) {
            approxlength_ = l;
        }
        double getApproxLength(void) {
            return approxlength_;
        }
        void setInterpolate(bool in) {
            interpolate_ = in;
        }
        bool getInterpolate(void) {
            return interpolate_;
        }
        void setR_RRT(double r) {
            r_rrt_ = r;
        }
        double getR_RRT(void) {
            return r_rrt_;
        }
        void setCheckTime(bool c) {
            checktime_ = c;
        }
        bool getCheckTime(void) {
            return checktime_;
        }
        void setReedsSheppRadius(double r) {
            reedssheppradius_ = r;
            sp_ = std::make_shared<base::ReedsSheppStateSpace>(reedssheppradius_);
        }
        double getReedsSheppRadius(void) {
            return reedssheppradius_;
        }
        void clear(void) override;
        void setup(void) override;
        void freeMemory();
        void showTree(void);
        void showCalcTime(void);
        std::vector<base::State *> getTree(void);

        protected :
        class Motion {
            public :
            Motion(const control::SpaceInformation *csi) : si(csi), state(csi->allocState()), parent(nullptr), inGoal(false), cost(0), incCost(0), potential(0) {}
            ~Motion() {
                si->freeState(state);
            }
            const control::SpaceInformation *si;
            base::State *state;
            Motion *parent;
            bool inGoal;
            double cost;
            double incCost;
            std::vector<Motion *> children;
            double potential;
            Eigen::Vector3d pvector;
            Eigen::Vector3d fvector;
        };
        class Tree {
            public :
            Tree() : head(nullptr), size(0) {};
            ~Tree() = default;
            Motion * head;
            std::vector<Motion *> members;
            int size;
            void freeMemory() {
                for (int i = 0 ; i < members.size() ; i++) {
                    if(members[i]->state) {
                        members[i]->si->freeState(members[i]->state);
                    }
                }
                members.clear();
            }
        };
        class TimeFlag {
            public :
            TimeFlag(std::string n) : name(n) {
                tp = std::chrono::system_clock::now();
            }
            double dur(TimeFlag init, TimeFlag fin) {
                std::chrono::duration<double> d = fin.tp-init.tp;
                return d.count();
            }
            std::chrono::time_point<std::chrono::system_clock> tp;
            std::string name;
        };
        void addFlag(std::string name);
        bool DOSampling(Motion* node);
        double fieldNormMag(Eigen::Vector3d field);
        bool DOExtend(Motion* nearest, Motion* rand, Motion* node);
        double DOMoveLength(double potential, Eigen::Vector3d pvector);
        double distanceMotion(const Motion *a, const Motion *b);
        double distanceMotionEuc(const Motion *a, const Motion *b);
        double combineCosts(double c1, double c2);
        void getNeighbors(Motion *motion, std::vector<int> &nbh);
        void removeFromParent(Motion *m);
        void updateChildCosts(Motion *m);
        bool collisionFree(Motion *m1, Motion *m2);

        void changeParent(Motion *np, Motion *m);
        int nearest(Motion *from);
        void addMotion(Motion *p, Motion *c);
        void setRoot(Motion *m);
        void showMotion(const Motion *m, const std::string tag);
        

        Tree mtree_;
        std::vector<TimeFlag> timeflags_;
        double potmax_;
        double fieldnorm_;
        double res_;
        double extendlength_;
        int maxsample_;
        const control::SpaceInformation *siC_;
        base::StateSamplerPtr sampler_;
        control::DirectedControlSamplerPtr controlSampler_;
        MagneticModelPtr mm_;
        PotentialModelPtr pm_;
        double r_rrt_;
        double approxlength_;
        bool checktime_;
        bool interpolate_;
        double reedssheppradius_;
        base::StateSpacePtr sp_;
        
    };
    typedef std::shared_ptr<doRRT> doRRTPtr;
}

#endif
