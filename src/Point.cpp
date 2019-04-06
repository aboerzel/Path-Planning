#include "Point.h"

Point::Point(double x, double y)
{
    X = x;
    Y = y;
}

Point::Point(vector<double> p)
{
    X = p[0];
    Y = p[1];
}

Point::~Point()
= default;
