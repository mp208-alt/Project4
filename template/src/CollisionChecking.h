#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>
#include <iostream>
#include <array>

struct Point {
    double x;
    double y;
};

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

// Definition of our robot.
struct Robot
{
    // Type = {c,s,p}.  Circle, square, or point robot
    char type;
    // The location of the robot in the environment
    double x, y;
    // The orientation of the square robot.  Undefined for point or circle robot
    double theta;
    // The length of a side of the square robot or the radius of the circle robot
    // Undefined for the point robot.
    double length;
};

// Intersect the point (x,y) with the set of rectangles. If the point lies
// outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles);

// Intersect a circle with center (x,y) and given radius with the set of
// rectangles. If the circle lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);

// Determine oritentation of a triangle given three points in 2d space. If the 
// triangle is oriented positively, return true.
double triangleOrientation(Point r, Point o1, Point o2);

// Rotate q by theta. Return q_prime, the point rotated in the robot's local coordinate frame.
void transformPoint(Point q, double theta);

// Intersect a square with center at (x,y), orientation theta, and the given
// side length with the set of rectangles. If the square lies outside of all
// obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles);

// Custom debugging/development code. Takes the list of robots, the list of
// obstacles, and whether or not each configuation should be valid or not.
void debugMode(const std::vector<Robot> &robots, const std::vector<Rectangle> &obstacles,
               const std::vector<bool> &valid);

#endif
