#pragma once
#include "Point.h"

class PointConverter
{
public:
    static Point map_to_vehicle_coordinates(const Point& p, const Point& ref_p, const double ref_yaw);

    static Point vehicle_to_map_coordinates(const Point& p, const Point& ref_p, double ref_yaw);
};

