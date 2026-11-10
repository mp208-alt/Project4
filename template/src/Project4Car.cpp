///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Timothy Slater
//////////////////////////////////////

#include <iostream>
#include <string>
#include <cmath>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/PathGeometric.h>
#include <Eigen/Core>
#include <ompl/tools/benchmark/Benchmark.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


// Your Projection for the car
class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override { return 2; }

    void defaultCellSizes() override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.5;
        cellSizes_[1] = 0.5;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *rstate = state->as<ob::RealVectorStateSpace::StateType>();
        projection[0] = rstate->values[0]; // x
        projection[1] = rstate->values[1]; // y
    }
};


void carODE(const oc::ODESolver::StateType &q, const oc::Control *u,
            oc::ODESolver::StateType &qdot)
{
    qdot.resize(q.size(), 0.0);

    double omega = 0.0, v_dot = 0.0;
    if (u)
    {
        const auto *control = u->as<oc::RealVectorControlSpace::ControlType>();
        if (control)
        {
            omega = control->values[0];
            v_dot = control->values[1];
        }
    }

    double theta = q[2];
    double v = q[3];

    qdot[0] = v * std::cos(theta);
    qdot[1] = v * std::sin(theta);
    qdot[2] = omega;
    qdot[3] = v_dot;

}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // Street with narrow winding horizontal cooridor that opens to a wider turn before the goal
    obstacles.push_back({0.0, 0.0, 9.0, 0.5}); 
    obstacles.push_back({0.0, 1.5, 9.0, 3.0}); 
    obstacles.push_back({4.0, 6.0, 7.0, 1.0}); 
    obstacles.push_back({0.0, 8.0, 7.0, 2.0}); 
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    const double pi = M_PI;
    auto space = std::make_shared<ob::RealVectorStateSpace>(4);

    //Set upper an lower bounds for car state
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, 0.0); 
    bounds.setHigh(0, 10.0); // x

    bounds.setLow(1, 0.0);
    bounds.setHigh(1, 10.0); // y

    bounds.setLow(2,-pi);
    bounds.setHigh(2, pi);  // theta    

    bounds.setLow(3, -3.0); 
    bounds.setHigh(3, 3.0); // v 

    space->setBounds(bounds);

    space->registerProjection("CarProjection", std::make_shared<CarProjection>(space.get()));
    
    //Set upper and lower bounds for car control
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -pi / 2.0); 
    cbounds.setHigh(0, pi / 2.0); // omega
    cbounds.setLow(1, -2.0); 
    cbounds.setHigh(1, 2.0); // vdot

    cspace->setBounds(cbounds);

    auto ss = std::make_shared<oc::SimpleSetup>(cspace);
    ss->setStatePropagator([](const ob::State *state, const oc::Control *control, double duration, ob::State *result)
    {
        const double pi = M_PI;
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        auto *f = result->as<ob::RealVectorStateSpace::StateType>();
        oc::ODESolver::StateType q = {s->values[0], s->values[1], s->values[2], s->values[3]};
        oc::ODESolver::StateType qdot(4, 0.0);
        carODE(q, control, qdot);

        f->values[0] = s->values[0] + duration * qdot[0];
        f->values[1] = s->values[1] + duration * qdot[1];

        // Clamp theta to [-pi, pi] to model car-like range
        double thetaClamped = s->values[2] + duration * qdot[2];
        while (thetaClamped >  pi) thetaClamped -= 2.0 * pi;
        while (thetaClamped < -pi) thetaClamped += 2.0 * pi;
        f->values[2] = thetaClamped;


        f->values[3] = s->values[3] + duration * qdot[3];
    });

    ss->setStateValidityChecker([&obstacles](const ob::State *state) -> bool
    {
        if (!state) return false;
        const auto *rstate = state->as<ob::RealVectorStateSpace::StateType>();
        if (!rstate) return false;
        double x = rstate->values[0];
        double y = rstate->values[1];
        double v = rstate->values[3];

        if (x < 0.0 || x > 10.0) return false;
        if (y < 0.0 || y > 10.0) return false;
        if (v < -3.0 || v > 3.0) return false;

        // Approximating car-like robot as point for collision checking
        return isValidPoint(x, y, obstacles);
    });

    // start: car is bottom-left, pointing right, stationary
    // goal: car is top-right, pointing right, stationary
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    start[0] = 1.0; start[1] = 1.0; start[2] = 0.0; start[3] = 0.0;
    goal[0]  = 9.0; goal[1]  = 9.0; goal[2]  = 0.0; goal[3]  = 0.0;


    ss->setStartAndGoalStates(start, goal, 0.5); 
    // threshold error bound of 0.5 since car might struggle to get into closer state


    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    if (choice == 1)
        ss->setPlanner(std::make_shared<oc::RRT>(ss->getSpaceInformation()));
    else if (choice == 2)
        ss->setPlanner(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()));
    else if (choice == 3 )
        ss->setPlanner(std::make_shared<oc::RG_RRT>(ss->getSpaceInformation()));

    ob::PlannerStatus solved = ss->solve(30.0);

    if (solved)
    {
        std::cout << "Found solution:\n";
        og::PathGeometric geoPath = ss->getSolutionPath().asGeometric();

        geoPath.printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No solution found.\n";
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
        ss->setup();
        ompl::tools::Benchmark b(*ss, "Car Control Experiment");

        b.addPlanner(std::make_shared<oc::RRT>(ss->getSpaceInformation()));
        b.addPlanner(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()));

        // RG-RRT with timestep 1 and 11 evenly-spaced control values (default)
        auto rgrrt1 = std::make_shared<oc::RG_RRT>(ss->getSpaceInformation());
        b.addPlanner(std::make_shared<oc::RG_RRT>(ss->getSpaceInformation()));
        rgrrt1->setName("RG_RRT (Controls = 11, TimeStep = 1)");
        b.addPlanner(rgrrt1);

        // RG-RRT with higher number of controls within the range
        auto rgrrt2 = std::make_shared<oc::RG_RRT>(ss->getSpaceInformation());
        rgrrt2->params().setParam("control_number", "110");
        rgrrt2->setName("RG_RRT (Controls = 110)");
        b.addPlanner(rgrrt2);

        // RG-RRT with larger time period
        auto rgrrt3 = std::make_shared<oc::RG_RRT>(ss->getSpaceInformation());
        rgrrt3->params().setParam("time_step", "10");
        rgrrt3->setName("RG_RRT (TimeStep = 10)");
        b.addPlanner(rgrrt3);

        ompl::tools::Benchmark::Request req;
        req.maxTime = 10.0;
        req.maxMem = 4096.0;
        req.runCount = 20;
        req.displayProgress = true;

        b.benchmark(req);
        b.saveResultsToFile("car_benchmark.log");
}

int main(int argc, char **argv)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);
    int choice = 1;
    int planner = -1;

    if (argc > 1)
        planner = std::atoi(argv[1]);

    if (planner < 1 || planner > 3)
    {
        do
        {
            std::cout << "Plan or Benchmark? " << std::endl;
            std::cout << " (1) Plan" << std::endl;
            std::cout << " (2) Benchmark" << std::endl;

            std::cin >> choice;
        } while (choice < 1 || choice > 2);
    }


    ompl::control::SimpleSetupPtr ss = createCar(obstacles);
    // Planning
    if (choice == 1)
    {
        if (planner < 1 || planner > 3)
        {
            do
            {
                std::cout << "What Planner? " << std::endl;
                std::cout << " (1) RRT" << std::endl;
                std::cout << " (2) KPIECE1" << std::endl;
                std::cout << " (3) RG-RRT" << std::endl;

                std::cin >> planner;
            } while (planner < 1 || planner > 3);
        }

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
