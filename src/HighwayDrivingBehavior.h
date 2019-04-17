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
     * Updates driving action (target lane, speed and acceleration) considering the traffic rules and other vehicles
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     */
    void update(double current_s, int current_lane, const vector<vector<double>>& sensor_fusion);

    /**
     * Target speed [m/s]
     */
    double target_speed;

    /**
     * Target acceleration [m/s²] 
     * Currently a constant acceleration is used. 
     * This can later be adapted dynamically (e.g. to brake more quickly in emergency situations).
     */
    double target_accel;

    /**
     * Target lane id 
     */
    int target_lane;

private:

    enum BehaviorState { KeepLane, PrepareLaneChangeLeft, PrepareLaneChangeRight, LaneChangeLeft, LaneChangeRight };

    /**
     * Keeps the vehicle in the current lane at a safe distance from the vehicle ahead.
     * Check if a lane change makes sense.
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return New behaviour state
     */
    BehaviorState keep_lane(double current_s, int current_lane, const vector<vector<double>>& sensor_fusion);

    /**
     * Waits for a gap for the upcoming lane change to the left lane
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return New behaviour state
     */
    BehaviorState prepare_lane_change_left(double current_s, int current_lane,
                                           const vector<vector<double>>& sensor_fusion);

    /**
     * Waits for a gap for the upcoming lane change to the right lane
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return New behaviour state
     */
    BehaviorState prepare_lane_change_right(double current_s, int current_lane,
                                            const vector<vector<double>>& sensor_fusion);

    /**
     * Moves to the left lane
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return New behaviour state
     */
    BehaviorState lane_change_left(double current_s, int current_lane, const vector<vector<double>>& sensor_fusion);

    /**
     * Moves to the right lane
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return New behaviour state
     */
    BehaviorState lane_change_right(double current_s, int current_lane, const vector<vector<double>>& sensor_fusion);

    /**
     * Checks whether a change to the specified lane can be carried out safely
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return bool value that indicates whether switching to the specified lane is safe
     */
    bool is_lane_safe_for_change(double current_s, int lane, const vector<vector<double>>& sensor_fusion);

    /**
     * Adjusts the vehicle speed to the speed of the specified lane
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     */
    void adapt_target_speed_to_lane_speed(double current_s, int lane, const vector<vector<double>>& sensor_fusion);

    /**
     * Determines the distance and the speed of the closest vehicle in the specified lane and direction
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @param forward Direction in, which is searched for vehicles (true = forward, false = backwards)
     * @return Distance [m] and speed [m/s] of the closest vehicle in the given direction
     */
    vector<double> find_closest_vehicle(double current_s, int current_lane, const vector<vector<double>>& sensor_fusion,
                                        bool forward);

    /**
     * Calculates the average speed of all vehicles ahead in lane
     * @param current_s The vehicle’s longitudinal position in frenet coordinates 
     * @param current_lane Lane in which the vehicle is currently located
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return Average lane speed [m/s] and number of vehicles ahead in lane
     */
    vector<double> get_average_lane_speed(double current_s, int current_lane, const vector<vector<double>>& sensor_fusion);

    /**
     * Determines the best lane considering the current traffic situation
     * @param current_s The vehicle’s longitudinal position in frenet coordinates
     * @param sensor_fusion Sensor data regarding other vehicles
     * @return Best lane Id
     */
    int get_best_lane(double current_s, const vector<vector<double>>& sensor_fusion);

    /**
     * Calculates a safe distance to the leading vehicle given by the leading vehicle speed
     * @param lead_vehicle_speed Speed of the leading vehicle [m/s]
     * @return Safety distance to the leading vehicle [m]
     */
    double get_dynamic_safety_distance(double lead_vehicle_speed);

    /**
     * Calculates a safe speed from the distance to the vehicle ahead
     * @param lead_vehicle_distance Distance to the leading vehicle [m]
     * @return Safety speed [m/s]
     */
    double get_dynamic_speed_from_distance(double lead_vehicle_distance);


    static string state_to_string(BehaviorState state);

    /**
     * Reference speed [m/s]
     * Target speed to be met if the traffic situation allows it
     */
    double reference_speed_;

    /**
     * Current behavior state
     * Used for the finite state machine
     */
    BehaviorState current_state_;

    /**
     * Id of the preferred lane 
     */
    int preferred_lane_;

    /**
     * Duration of the current lane change preparation [s]
     */
    double prepare_lane_change_time_;
};
