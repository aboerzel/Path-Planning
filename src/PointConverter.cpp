#include "Point.h"
#include "PointConverter.h"
#include <math.h>

Point PointConverter::map_to_vehicle_coordinates(const Point& p, const Point& ref_p, const double ref_yaw)
{
    const auto shift_x = p.X - ref_p.X;
    const auto shift_y = p.Y - ref_p.Y;

    const auto x = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    const auto y = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

    return Point(x, y);
}

Point PointConverter::vehicle_to_map_coordinates(const Point& p, const Point& ref_p, const double ref_yaw)
{
    auto x = p.X * cos(ref_yaw) - p.Y * sin(ref_yaw);
    auto y = p.X * sin(ref_yaw) + p.Y * cos(ref_yaw);

    x += ref_p.X;
    y += ref_p.Y;

    return Point(x, y);
}
