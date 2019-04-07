#pragma once
#include <vector>

using std::vector;

struct DrivingAction 
{
    int lane;
    double speed;
};  

class HighwayDrivingBehavior
{
public:
    HighwayDrivingBehavior();
    ~HighwayDrivingBehavior();

    DrivingAction get_driving_action(double s, int current_lane, const vector<vector<double>>& sensor_fusion);

private:

    vector<double> find_closest_vehicle(double s, int lane, const vector<vector<double>>& sensor_fusion, bool forward);

    int get_best_lane(double s, int lane, const vector<vector<double>>& sensor_fusion);

    vector<double> avg_scores = { 0,0,0 };
};

