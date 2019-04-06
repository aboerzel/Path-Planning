#pragma once
#include <vector>

using std::vector;

class BehaviorPlanner
{
public:
    BehaviorPlanner();
    ~BehaviorPlanner();

    int get_target_lane(double s, int current_lane, vector<vector<double>> sensor_fusion);

    vector<double> find_closest_vehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);

    int get_lane_score(double s, int lane, vector<vector<double>> sensor_fusion);

    int current_lane;
    double target_vehicle_speed;
    double lead_vehicle_speed;


    static int calculate_lane(double d);
};

