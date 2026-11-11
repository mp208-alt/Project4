micromamba activate ompl_env

To build:
```
    cd template
    make
```

To run:
    ./Project4Car <planner_type>
        where planner_type is 1 (RRT) or 2 (KPIECE1)
            for example (RRT): ./Project4Car 1

    all run commands:
    ./Project4Car 1
    ./Project4Car 2

To export the solution paths for visualization:
    ./Project4Car 1 > path_car_rrt.txt
    ./Project4Car 2 > path_car_kpiece.txt
