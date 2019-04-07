#pragma once
#include "Point.h"

/**
 * Converts points between map coordinates and vehicle coordinates
 */
class PointConverter
{
public:
    
    /**
     * brief 
     * @param p 
     * @param ref_p 
     * @param ref_yaw 
     * @return 
     */
    static Point map_to_vehicle_coordinates(const Point& p, const Point& ref_p, const double ref_yaw);

    /**
     * \brief 
     * @param p 
     * @param ref_p 
     * @param ref_yaw 
     * @return 
     */
    static Point vehicle_to_map_coordinates(const Point& p, const Point& ref_p, double ref_yaw);
};

