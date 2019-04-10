#pragma once
#include "HighwayDrivingBehavior.h"
#include "Point.h"

/**
 * Calculates the next path that the vehicle should drive, 
 * considering the traffic rules for highways and the current traffic situation.
 */
class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();

    /**
     * Calculates the next path from the previous path, the current vehicle position as well as the waypoints and sensor fusion data.
     * @param previous_path_x X coordinates of the previous path (map coordinate system)
     * @param previous_path_y Y coordinates of the previous path (map coordinate system)
     * @param map_waypoints_s Frenet s positions of map waypoints
     * @param map_waypoints_x X coordinates of the map waypoints
     * @param map_waypoints_y Y coordinates of the map waypoints
     * @param sensor_fusion Sensor Fusion Data
     * @param car_x X coordinate of the current vehicle position (map coordinate system)
     * @param car_y Y coordinate of the current vehicle position (map coordinate system)
     * @param car_yaw Yaw angle of the vehicle (map coordinate system)
     * @param car_speed Current vehicle speed [mph]
     * @param car_s Current Frenet s position of the vehicle
     * @return Next path the vehicle should drive
     */
    vector<Point> calculate_path(const vector<double>& previous_path_x, const vector<double>& previous_path_y,
                                 const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y, 
                                 const vector<vector<double>>& sensor_fusion, double car_x, double car_y, double car_yaw, double car_speed, double car_s);

private:

    /**
     * Driving behavior for highways 
     */
    HighwayDrivingBehavior highway_driving_behavior;

    /**
     * Reference speed [mph]
     */
    double ref_speed = 0.0; };

