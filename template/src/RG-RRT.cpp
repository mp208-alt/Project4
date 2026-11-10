///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Timothy Slater and Maya Parnes
//////////////////////////////////////

#include "RG-RRT.h"
/*********************************************************************
- Software License Agreement (BSD License)
 *
-  Copyright (c) 2008, Willow Garage, Inc.
-  All rights reserved.
 *
-  Redistribution and use in source and binary forms, with or without
-  modification, are permitted provided that the following conditions
-  are met:
 *
- Redistributions of source code must retain the above copyright
-     notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above
-     copyright notice, this list of conditions and the following
-     disclaimer in the documentation and/or other materials provided
-     with the distribution.
- Neither the name of the Willow Garage nor the names of its
-     contributors may be used to endorse or promote products derived
-     from this software without specific prior written permission.
 *
-  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
-  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
-  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
-  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
-  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
-  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/control/PathControl.h"
#include "ompl/control/PlannerData.h"
#include <limits>

ompl::control::RG_RRT::RG_RRT(const SpaceInformationPtr &si) : base::Planner(si, "RG_RRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    // Binding parameter to control count functions
    Planner::declareParam<double>("goal_bias", this, &RG_RRT::setGoalBias, &RG_RRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("control_number", this, &RG_RRT::setControlNumber, &RG_RRT::getControlNumber);
    Planner::declareParam<double>("time_step", this, &RG_RRT::setTimeStep, &RG_RRT::getTimeStep);
    Planner::declareParam<int>("reachability_control_count", this, &RG_RRT::setControlNumber, &RG_RRT::getControlNumber);
    Planner::declareParam<bool>("intermediate_states", this, &RG_RRT::setIntermediateStates, &RG_RRT::getIntermediateStates, "0,1");
}

ompl::control::RG_RRT::~RG_RRT()
{
    freeMemory();
}

void ompl::control::RG_RRT::setup()
{
    base::Planner::setup();
    if (!nearest_neighbors)
        nearest_neighbors.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nearest_neighbors->setDistanceFunction([this](const Motion *a, const Motion *b)
                                           { return distanceFunction(a, b); });
}

void ompl::control::RG_RRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nearest_neighbors)
        nearest_neighbors->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RG_RRT::freeMemory()
{
    if (nearest_neighbors)
    {
        std::vector<Motion *> motions;
        nearest_neighbors->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            for (auto &rm : motion->reachability_set)
            {
                if (rm->state)
                    si_->freeState(rm->state);
                if (rm->control)
                    siC_->freeControl(rm->control);
                delete rm;
            }
            delete motion;
        }
    }
}

// Define the reachable set of a state node m
void ompl::control::RG_RRT::computeReachableSet(Motion *m)
{
    auto *cspace = siC_->getControlSpace()->as<ompl::control::RealVectorControlSpace>();
    const ompl::base::RealVectorBounds &bounds = cspace->getBounds();

    int n = getControlNumber();
    // Uniform sampling (making sure controls are evenly spaced across range)
    double step = (n > 1) ? (bounds.high[0] - bounds.low[0]) / (n - 1) : 0.0;

    for (int i = 0; i < n; ++i)
    {
        double u_val = bounds.low[0] + (step * i);

        auto *reachMotion = new Motion(siC_);
        auto *reachControl = siC_->allocControl()->as<ompl::control::RealVectorControlSpace::ControlType>();
        siC_->nullControl(reachControl);
        reachControl->values[0] = u_val;

        // Propagate state for timestep duration
        unsigned int cd = siC_->propagateWhileValid(m->state, reachControl, timeStep_, reachMotion->state);

        // Check state validity, if valid add to reachability set, and if invalid delete to save memory
        if (cd == timeStep_ && si_->isValid(reachMotion->state))
        {
            siC_->copyControl(reachMotion->control, reachControl);
            reachMotion->steps = cd;
            m->reachability_set.push_back(reachMotion);
        }
        else
        {
            if (reachMotion->state)
                si_->freeState(reachMotion->state);
            if (reachMotion->control)
                siC_->freeControl(reachMotion->control);
            delete reachMotion;
        }
        siC_->freeControl(reachControl);
    }
}

ompl::base::PlannerStatus ompl::control::RG_RRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Compute reachability set of the root, add to tree
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        computeReachableSet(motion);
        nearest_neighbors->add(motion);
    }

    if (nearest_neighbors->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nearest_neighbors->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *q_rand = new Motion(siC_);
    base::State *rstate = q_rand->state;
    auto *control = siC_->allocControl();

    while (!ptc)
    {
        // Sample random goal state (with goal biasing if chosen)
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Find the nearest existing node to the current state in the tree
        Motion *q_near = nearest_neighbors->nearest(q_rand);

        // Check if any state is closer to current state than q_near
        bool reachable_is_closer = false;
        double distance_to_near = distanceFunction(q_near, q_rand);

        for (Motion *rm : q_near->reachability_set)
        {
            if (distanceFunction(rm, q_rand) < distance_to_near)
            {
                reachable_is_closer = true;
                break;
            }
        }

        // If q_near is closer than all reachable states --> there is a constraint preventing motion there and tree stops expansion
        if (!reachable_is_closer)
            continue;

        // If not, tree expands from q_near toward q_rand
        unsigned int cd = controlSampler_->sampleTo(control, q_near->control, q_near->state, q_rand->state);

        // Produce a new state and compute its reachability set (if new state is valid)
        if (cd > 0)
        {
            auto *motion = new Motion(siC_);
            siC_->copyControl(motion->control, control);
            motion->steps = cd;
            motion->parent = q_near;

            siC_->propagate(q_near->state, motion->control, cd, motion->state);

            // Check if state is valid
            if (si_->isValid(motion->state))
            {
                computeReachableSet(motion);
                nearest_neighbors->add(motion);

                double dist = 0.0;
                bool goal_reached = goal->isSatisfied(motion->state, &dist);
                if (goal_reached)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }
            else
            {
                if (motion->state)
                    si_->freeState(motion->state);
                if (motion->control)
                    siC_->freeControl(motion->control);
                delete motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        std::vector<Motion *> solution_nodes;
        while (solution != nullptr)
        {
            solution_nodes.push_back(solution);
            solution = solution->parent;
        }

        auto path(std::make_shared<PathControl>(si_));
        for (int i = solution_nodes.size() - 1; i >= 0; --i)
        {
            if (solution_nodes[i]->parent)
                path->append(solution_nodes[i]->state, solution_nodes[i]->control, solution_nodes[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(solution_nodes[i]->state);
        }
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (q_rand->state)
        si_->freeState(q_rand->state);
    if (q_rand->control)
        siC_->freeControl(q_rand->control);
    delete q_rand;
    siC_->freeControl(control);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nearest_neighbors->size());

    return {solved, approximate};
}

void ompl::control::RG_RRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nearest_neighbors)
        nearest_neighbors->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}