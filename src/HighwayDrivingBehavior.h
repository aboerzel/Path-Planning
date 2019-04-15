#pragma once
#include <vector>
#include <string>

using namespace std;
using std::vector;

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
     * @param current_speed TODO
     * @param sensor_fusion Sensor data regarding other vehicles.
     * @return Driving action (target lane and speed) to be performed
     */
    void update(double s, int current_lane, double current_speed, const vector<vector<double>>& sensor_fusion);

    double target_speed; // target speed [m/s]

    int target_lane;

private:

    enum BehaviorState { KeepLane, PrepareLaneChangeLeft, PrepareLaneChangeRight, LaneChangeLeft, LaneChangeRight };

    /**
     * Determines the distance and the speed of the closest vehicle in the specified lane and direction. 
     * @param s The vehicle’s s position in frenet coordinates.
     * @param current_lane Lane in which the vehicle is currently located.
     * @param sensor_fusion Sensor data regarding other vehicles.
     * @param forward Direction in, which is searched for vehicles (true = forward, false = backwards)
     * @return Distance and speed of the closest vehicle in the given direction
     */
    vector<double> find_closest_vehicle(double s, int current_lane, const vector<vector<double>>& sensor_fusion,
                                        bool forward);

    /**
     * Determines the best lane considering the current traffic situation.
     * @param s The vehicle’s s position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located.
     * @param current_speed TODO
     * @param sensor_fusion Sensor data regarding other vehicles.
     * @return Best lane
     */
    int get_best_lane(double s, int current_lane, double current_speed, const vector<vector<double>>& sensor_fusion);

    /**
     * Calculates a safe distance to the leading vehicle given by the leading vehicle speed
     * \param lead_vehicle_speed Speed of the leading vehicle [m/s]
     * \return Safety distance to the leading vehicle [m]
     */
    double get_dynamic_safety_distance(double lead_vehicle_speed);

    /**
     * Calculates a  
     * \param lead_vehicle_distance 
     * \return 
     */
    double get_dynamic_speed_from_distance(double lead_vehicle_distance);

    void follow_lead_vehicle(double s, int current_lane, const vector<vector<double>>& sensor_fusion);

    void adapt_target_lane_speed(double s, const vector<vector<double>>& sensor_fusion);

    BehaviorState keep_lane(double s, const int current_lane, double current_speed,
                            const vector<vector<double>>& sensor_fusion);

    BehaviorState prepare_lane_change_left(double s, const int current_lane,
                                           const vector<vector<double>>& sensor_fusion);

    BehaviorState prepare_lane_change_right(double s, const int current_lane,
                                            const vector<vector<double>>& sensor_fusion);

    BehaviorState lane_change_left(double s, const int current_lane, const vector<vector<double>>& sensor_fusion);

    BehaviorState lane_change_right(double s, const int current_lane, const vector<vector<double>>& sensor_fusion);

    static string state_to_string(BehaviorState state);

    double reference_speed_; // target speed [m/s]

    BehaviorState current_state;
};
