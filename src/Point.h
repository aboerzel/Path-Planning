#pragma once
#include <vector>

using std::vector;

class Point
{
public:
    Point(double x, double y);
    Point(vector<double> p);
    ~Point();

    double X;
    double Y;
};
