///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
//////////////////////////////////////

#include "CollisionChecking.h"

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (Rectangle obstacle : obstacles) {
        double x_min = obstacle.x;
        double x_max = x_min + obstacle.width;

        double y_min = obstacle.y;
        double y_max = y_min + obstacle.height;

        if (x_min <= x && x <= x_max && y_min <= y && y <= y_max) {
            return false;
        }
    }

    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (Rectangle obstacle : obstacles) {
        double x_min = obstacle.x;
        double x_max = x_min + obstacle.width;

        double y_min = obstacle.y;
        double y_max = y_min + obstacle.height;

        std::array<Point, 4> vertices = {{
            {x_min, y_min},
            {x_min, y_max},
            {x_max, y_min},
            {x_max, y_max}
        }};

        if ((x_min - radius <= x && x <= x_max + radius) && (y_min <= y && y <= y_max)
            || (x_min <= x && x <= x_max) && (y_min - radius <= y && y <= y_max + radius)
            ) {
                return false;
        }

        for (Point vertex : vertices) {
            double magnitude = std::sqrt(std::pow(x - vertex.x, 2) + std::pow(y - vertex.y, 2));
            if (magnitude <= radius) {
                return false;
            }
        }
    }

    return true;
}

// Determine oritentation of a triangle given three points in 2d space. If the 
// triangle is oriented positively, return true.
double triangleOrientation(Point a, Point b, Point c) {
    return (b.x * c.y - b.y * c.x) - (a.x * c.y - a.y * c.x) + (a.x * b.y - a.y * b.x);
}

// Rotate q by theta. Return qp (q prime), the point rotated in the robot's local coordinate frame.
void transformPoint(Point &q, Point v, double theta) {
    q = {
        {q.x * std::cos(theta) - q.y * std::sin(theta) + v.x},
        {q.x * std::sin(theta) + q.y * std::cos(theta) + v.y}
    };
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{   
    Point robot_center = {x, y}; // World Frame
    std::array<Point, 4> robot_vertices = {{ // Robot Frame
            {-(sideLength / 2), -(sideLength / 2)},
            {(sideLength / 2), -(sideLength / 2)},
            {(sideLength / 2), (sideLength / 2)},
            {-(sideLength / 2), (sideLength / 2)}
    }};

    for (Point &vertex : robot_vertices)  transformPoint(vertex, robot_center, theta);// Convert each point to World Frame

    for (Rectangle obstacle : obstacles) {
        double x_min = obstacle.x;
        double x_max = x_min + obstacle.width;

        double y_min = obstacle.y;
        double y_max = y_min + obstacle.height;

        std::array<Point, 4> obstacle_vertices = {{
            {x_min, y_min},
            {x_max, y_min},
            {x_max, y_max},
            {x_min, y_max}
        }};

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                Point robot1 = {robot_vertices[i].x, robot_vertices[i].y};
                Point robot2 = {robot_vertices[(i == 3) ? 0 : i + 1].x, robot_vertices[(i == 3) ? 0 : i + 1].y};

                Point obstacle1 = {obstacle_vertices[j].x, obstacle_vertices[j].y};
                Point obstacle2 = {obstacle_vertices[(j == 3) ? 0 : j + 1].x, obstacle_vertices[(j == 3) ? 0 : j + 1].y};

                bool is_intersecting = triangleOrientation(robot1, robot2, obstacle1) * triangleOrientation(robot1, robot2, obstacle2) < 0 
                                        && triangleOrientation(obstacle1, obstacle2, robot1) * triangleOrientation(obstacle1, obstacle2, robot2) < 0;
                is_intersecting = is_intersecting || !isValidPoint(robot_center.x, robot_center.y, {obstacle,});
                
                if (is_intersecting) {
                    return false;
                }


            }
        }
    }
    return true;

   
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> & /*robots*/, const std::vector<Rectangle> & /*obstacles*/,
               const std::vector<bool> & /*valid*/)
{
}
