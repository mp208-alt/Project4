micromamba activate ompl_env

To build:
    cd template
    make

To run:
    ./Project4Pendulum <torque_mode> <planner_type>
        where torque_mode is 1,2, or 3 (corresponding to low (-3, 3), medium (-5, 5), and high (-10, 10) torque values), and planner_type is either kpiece1 or rrt
            for example (low torque mode on KPIECE1): ./Project4Pendulum 1 kpiece1

    all run commands:
    ./Project4Pendulum 1 kpiece1
    ./Project4Pendulum 2 kpiece1
    ./Project4Pendulum 3 kpiece1

    ./Project4Pendulum 1 rrt
    ./Project4Pendulum 2 rrt
    ./Project4Pendulum 3 rrt

To export the solution paths for visualization:
    ./Project4Pendulum 1 rrt > path_rrt_torque3.txt
    ./Project4Pendulum 1 kpiece1 > path_kpiece_torque3.txt