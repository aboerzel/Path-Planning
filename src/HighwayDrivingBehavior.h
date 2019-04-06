#pragma once
#include <vector>

using std::vector;

class HighwayDrivingBehavior
{
public:
    HighwayDrivingBehavior();
    ~HighwayDrivingBehavior();

    int get_target_lane(double s, int current_lane, const vector<vector<double>>& sensor_fusion);

    double target_vehicle_speed;

private:

    vector<double> find_closest_vehicle(double s, int lane, const vector<vector<double>>& sensor_fusion, bool forward);

    int get_lane_score(double s, int lane, const vector<vector<double>>& sensor_fusion);

    vector<double> avg_scores = { 0,0,0 };
};

