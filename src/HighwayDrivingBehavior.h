#pragma once
#include <vector>

using std::vector;

/**
 * Driving action to be performed
 */
struct DrivingAction 
{
    int lane;       // target lane
    double speed;   // target speed [m/s]
};  

/**
 * Determines the best driving action for driving on a highway, 
 * considering the traffic rules and the current traffic situation. 
 */
class HighwayDrivingBehavior
{
public:
    HighwayDrivingBehavior();
    ~HighwayDrivingBehavior();

    /**
     * Determines the best driving action considering the traffic rules and other vehicles.
     * @param s The vehicle’s s position in frenet coordinates.
     * @param current_lane Lane in which the vehicle is currently located.
     * @param sensor_fusion Sensor data regarding other vehicles.
     * @return Driving action (target lane and speed) to be performed
     */
    DrivingAction get_driving_action(double s, int current_lane, const vector<vector<double>>& sensor_fusion);

private:

    /**
     * Determines the distance and the speed of the closest vehicle in the specified lane and direction. 
     * @param s The vehicle’s s position in frenet coordinates.
     * @param lane Lane in which the vehicle is currently located.
     * @param sensor_fusion Sensor data regarding other vehicles.
     * @param forward Direction in, which is searched for vehicles (true = forward, false = backwards)
     * @return Distance and speed of the closest vehicle in the given direction
     */
    vector<double> find_closest_vehicle(double s, int lane, const vector<vector<double>>& sensor_fusion, bool forward);

    /**
     * Determines the best lane considering the current traffic situation.
     * @param s The vehicle’s s position in frenet coordinates
     * @param lane Lane in which the vehicle is currently located.
     * @param sensor_fusion Sensor data regarding other vehicles.
     * @return Best lane
     */
    int get_best_lane(double s, int lane, const vector<vector<double>>& sensor_fusion);

    vector<double> avg_scores = { 0,0,0 };
};


