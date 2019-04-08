#pragma once
#include <vector>

using std::vector;


/**
 * Holds the x- and y-coordinate of a 2D point
 */
class Point
{
public:
   
    /**
     * Creates a point from two double values
     * @param x x-coordinate
     * @param y y-coordinate
     */
    Point(double x, double y);
    
    /**
     * Creates a point from a vector of two double values
     * @param p Vector of two double values (x, y)
     */
    Point(vector<double> p);

    /**
     * Destructor
     */
    ~Point();

    /**
     * x-coordinate 
     */
    double X;
    
    /**
     * y-coordinate
     */
    double Y;
};
