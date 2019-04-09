#pragma once
#include "HighwayDrivingBehavior.h"
#include "Point.h"

class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();

    vector<Point> calculate_path(const vector<double>& previous_path_x, const vector<double>& previous_path_y,
                                 const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x,
                                 const vector<double>& map_waypoints_y, const vector<vector<double>>& sensor_fusion,
                                 double car_x, double car_y, double car_yaw, double car_speed, double car_s);

    HighwayDrivingBehavior highway_driving_behavior;

    double ref_speed = 0.0; // reference speed [mph]
};

