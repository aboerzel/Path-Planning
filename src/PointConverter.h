#pragma once
#include "Point.h"

/**
 * Converts points between map coordinates and vehicle coordinates
 */
class PointConverter
{
public:
    
    /**
     * Converts a point on the map to a point relative to the vehicle position
     * @param p Waypoint on the map
     * @param ref_p Current vehicle position on the map
     * @param ref_yaw Current angle of the vehicle relative to the map's cornet system
     * @return Point p relative to the vehicle position
     */
    static Point map_to_vehicle_coordinates(const Point& p, const Point& ref_p, const double ref_yaw);

    /**
     * Converts a point relative from the vehicle position to a point on the map
     * @param p Path point relative to the vehicle position
     * @param ref_p Current vehicle position on the map
     * @param ref_yaw Current angle of the vehicle relative to the map's cornet system
     * @return Point p on the map
     */
    static Point vehicle_to_map_coordinates(const Point& p, const Point& ref_p, double ref_yaw);
};

