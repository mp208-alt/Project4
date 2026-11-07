#ifndef RG_RRT_H
#define RG_RRT_H

#include "ompl/base/Planner.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/RandomNumbers.h"
#include <vector>

namespace ompl
{
    namespace control
    {
        class RG_RRT : public base::Planner
        {
        public:
            RG_RRT(const SpaceInformationPtr &si);
            ~RG_RRT() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void clear() override;
            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;

            // Goal bias
            void setGoalBias(double goalBias) { goalBias_ = goalBias; }
            double getGoalBias() const { return goalBias_; }

            // Intermediate states
            void setIntermediateStates(bool addIntermediateStates) { intermediateStates_ = addIntermediateStates; }
            bool getIntermediateStates() const { return intermediateStates_; }

            // Reachability control count
            void setControlNumber(int n) { controlNumber_ = n; }
            int getControlNumber() const { return controlNumber_; }

            // Time step
            void setTimeStep(unsigned int step) { timeStep_ = step; }
            unsigned int getTimeStep() const { return timeStep_; }

            class Motion
            {
            public:
                Motion() = default;
                Motion(const SpaceInformation *si)
                    : state(si->allocState()), control(si->allocControl()) {}
                ~Motion() = default;

                base::State *state{nullptr};
                Control *control{nullptr};
                Motion *parent{nullptr};
                // Store motion step duration
                unsigned int steps{0};
                // Approximate reachable set
                std::vector<Motion *> reachability_set;
            };

        protected:
            void freeMemory();
            void computeReachableSet(Motion *m);
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            const SpaceInformation *siC_{nullptr};
            std::shared_ptr<NearestNeighbors<Motion *>> nearest_neighbors;

            base::StateSamplerPtr sampler_;
            DirectedControlSamplerPtr controlSampler_;

            double goalBias_{0.05};
            bool intermediateStates_{true};
            int controlNumber_{11};
            unsigned int timeStep_{1};

            RNG rng_;
            Motion *lastGoalMotion_{nullptr};
        };
    }
}
#include "RG-RRT.cpp"
#endif