///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Maya Parnes
//////////////////////////////////////

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include "RG-RRT.h"
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/config.h>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <valarray>
#include <limits>
#include <cmath>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

class PendulumProjection : public ob::ProjectionEvaluator
{
public:
    PendulumProjection(const ob::StateSpace *space) : ob::ProjectionEvaluator(space) {}

    unsigned int getDimension() const override { return 2; }

    void defaultCellSizes() override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.5;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *rstate = state->as<ob::RealVectorStateSpace::StateType>();
        projection[0] = rstate->values[0];
        projection[1] = rstate->values[1];
    }
};

// Definitions of variables and functions
void PendulumODE(const oc::ODESolver::StateType &q, const oc::Control *u, oc::ODESolver::StateType &qdot)
{
    qdot.resize(q.size(), 0.0);

    double torque = 0.0;
    if (u)
    {
        const auto *control = u->as<oc::RealVectorControlSpace::ControlType>();
        if (control)
        {
            torque = control->values[0];
        }
    }

    double rot_velocity = q[1];
    double theta = q[0];
    float g = 9.81;

    // Functions of theta dot and omega dot using defined variables
    qdot[0] = rot_velocity;
    qdot[1] = -g * std::cos(theta) + torque;
}

int main(int argc, char **argv)
{
    float pi = 3.14159265f;

    // Defining the upper and lower bounds for the 2 dimensions in the state space (theta, rot_velocity)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -pi / 2.0);
    bounds.setHigh(0, pi / 2.0);
    bounds.setLow(1, -10.0);
    bounds.setHigh(1, 10.0);
    space->setBounds(bounds);

    space->registerProjection("PendulumProjection", std::make_shared<PendulumProjection>(space.get()));

    // Defining the upper and lower bounds for torque (in the control space)
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 1);
    ob::RealVectorBounds cbounds(1);
    int change_of_toque = (argc > 1) ? std::atoi(argv[1]) : 1;
    std::string planner_choice = (argc > 2) ? argv[2] : "rrt";
    double max_torque = 5.0;

    if (change_of_toque == 1)
    {
        max_torque = 3.0; // Low torque
    }
    else if (change_of_toque == 2)
    {
        max_torque = 5.0; // Medium torque
    }
    else if (change_of_toque == 3)
    {
        max_torque = 10.0; // High torque
    }

    cbounds.setLow(0, -max_torque);
    cbounds.setHigh(0, max_torque);
    cspace->setBounds(cbounds);

    // Setting up the ODE function
    auto setup = std::make_shared<oc::SimpleSetup>(cspace);

    // Implement KPIECE1, RRT, and RG-RRT
    if (planner_choice == "kpiece1" || planner_choice == "kpiece")
    {
        setup->setPlanner(std::make_shared<oc::KPIECE1>(setup->getSpaceInformation()));
    }
    else if (planner_choice == "rrt")
    {
        setup->setPlanner(std::make_shared<oc::RRT>(setup->getSpaceInformation()));
    }
    else if (planner_choice == "rg-rrt" || planner_choice == "rgrrt")
    {
        setup->setPlanner(std::make_shared<oc::RG_RRT>(setup->getSpaceInformation()));
    }

    setup->setStatePropagator([&](const ob::State *state, const oc::Control *control, double duration, ob::State *result)
                              {
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        auto *f = result->as<ob::RealVectorStateSpace::StateType>();
        oc::ODESolver::StateType q = {s->values[0], s->values[1]};
        oc::ODESolver::StateType qdot(2, 0.0);
        // Calculate the rates of change with ODE
        PendulumODE(q, control, qdot);
        // Find final state f and velocity using the calculated rates of change (Euler integration)
        f->values[0] = s->values[0] + duration * qdot[0];
        f->values[1] = s->values[1] + duration * qdot[1]; });

    // Assigning validity to states (only valid for upper and lower bounds of the angular velocity)
    setup->setStateValidityChecker([](const ob::State *state) -> bool
                                   {
        if (!state) return false;
        const auto *rstate = state->as<ob::RealVectorStateSpace::StateType>();
        return rstate && rstate->values[1] >= -10.0 && rstate->values[1] <= 10.0; });

    // Setting start and goal configurations
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    start[0] = -pi / 2.0;
    goal[0] = pi / 2.0;
    start[1] = 0.0;
    goal[1] = 0.0;

    // Setting a threshold error bound of 0.05
    double threshold = 0.05;
    setup->setStartAndGoalStates(start, goal, threshold);

    if (planner_choice == "benchmark")
    {
        setup->setup();
        ompl::tools::Benchmark b(*setup, "Pendulum Control Experiment");

        b.addPlanner(std::make_shared<oc::RRT>(setup->getSpaceInformation()));
        b.addPlanner(std::make_shared<oc::KPIECE1>(setup->getSpaceInformation()));
        b.addPlanner(std::make_shared<oc::RG_RRT>(setup->getSpaceInformation()));

        ompl::tools::Benchmark::Request req;
        req.maxTime = 10.0;
        req.maxMem = 4096.0;
        req.runCount = 20;
        req.displayProgress = true;

        b.benchmark(req);
        b.saveResultsToFile("pendulum_benchmark.log");
    }
    else
    {
        ob::PlannerStatus solved = setup->solve(10.0);

        if (solved)
        {
            std::cout << "Found solution:\n";
            og::PathGeometric geoPath = setup->getSolutionPath().asGeometric();
            geoPath.printAsMatrix(std::cout);
        }
        else
        {
            std::cout << "No solution found.\n";
        }
    }

    return 0;
}