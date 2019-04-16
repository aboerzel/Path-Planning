#include "HighwayDrivingBehavior.h"
#include <math.h>
#include <algorithm>
#include "LaneConverter.h"
#include <string>
#include "SpeedConverter.h"
#include <iostream>
#include <exception>

using namespace std;

// parameter to adjust driving behavior
#define MAX_SPEED 22.12848                      // maximum speed in [m/sec] => 49.5 MPS
#define MAX_ACCEL (9.0)                         // max acceleration [m/s²] => 10 m/s² * 0.90 => 9.0 m/s²
#define MIN_LANE_CHANGE_DISTANCE_AHEAD 10.0     // minimum distance [m] to the vehicle ahead on the target lane, needed for lane change
#define MIN_LANE_CHANGE_DISTANCE_BEHIND 30.0    // minimum distance [m] to the vehicle behind on the target lane, needed for lane change
#define MIN_DISTANCE 5.0                        // minimum distance [m] to the vehicle ahead regardless of the speed

HighwayDrivingBehavior::HighwayDrivingBehavior()
{
    reference_speed_ = MAX_SPEED; // our reference speed is the maximum speed allowed on highways

    target_speed = 0.0; // target speed [m/s]
    target_accel = MAX_ACCEL; // here we use a simple constant acceleration [m/s²]
    target_lane = -1;

    current_state = KeepLane; // initial behavior state
}

HighwayDrivingBehavior::~HighwayDrivingBehavior()
= default;

void HighwayDrivingBehavior::update(const double s, const int current_lane, const double current_speed,
                                    const vector<vector<double>>& sensor_fusion)
{
    printf("%-22s: %s\n", "current state:", state_to_string(current_state).c_str());

    switch (current_state)
    {
    case KeepLane:
        {
            current_state = keep_lane(s, current_lane, current_speed, sensor_fusion);
            break;
        }
    case PrepareLaneChangeLeft:
        {
            current_state = prepare_lane_change_left(s, current_lane, sensor_fusion);
            break;
        }
    case PrepareLaneChangeRight:
        {
            current_state = prepare_lane_change_right(s, current_lane, sensor_fusion);
            break;
        }
    case LaneChangeLeft:
        {
            current_state = lane_change_left(s, current_lane, sensor_fusion);
            break;
        }
    case LaneChangeRight:
        {
            current_state = lane_change_right(s, current_lane, sensor_fusion);
            break;
        }
    default:
        throw std::range_error("Unexpected behavior state");
    }
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::keep_lane(
    const double s, const int current_lane, const double current_speed, const vector<vector<double>>& sensor_fusion)
{
    target_lane = current_lane;

    adapt_target_speed_to_lane_speed(s, target_lane, sensor_fusion);

    // search for the best lane
    const auto new_lane = get_best_lane(s, target_lane, current_speed, sensor_fusion);

    if (new_lane < target_lane)
    {
        return PrepareLaneChangeLeft;
    }

    if (new_lane > target_lane)
    {
        return PrepareLaneChangeRight;
    }

    return KeepLane;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::prepare_lane_change_left(
    const double s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    const auto left_lane = current_lane - 1;

    // check if left lane is safe for lane change
    if (is_lane_safe_for_change(s, left_lane, sensor_fusion))
    {
        // safe to change lane 
        target_lane = left_lane;
        adapt_target_speed_to_lane_speed(s, target_lane, sensor_fusion);
        return LaneChangeLeft;
    }

    // check min distance to leading vehicle on current lane
    const auto lead_vehicle = find_closest_vehicle(s, current_lane, sensor_fusion, true);
    if (lead_vehicle[0] < MIN_LANE_CHANGE_DISTANCE_AHEAD)
    {
        // discard the overtaking maneuver
        adapt_target_speed_to_lane_speed(s, current_lane, sensor_fusion);
        return KeepLane;
    }

    // adapt speed to left lane speed
    adapt_target_speed_to_lane_speed(s, left_lane, sensor_fusion);
    return PrepareLaneChangeLeft;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::prepare_lane_change_right(
    const double s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    const auto right_lane = current_lane + 1;

    // check if right lane is a safe for lane change
    if (is_lane_safe_for_change(s, right_lane, sensor_fusion))
    {
        // discard the overtaking maneuver
        target_lane = right_lane;
        adapt_target_speed_to_lane_speed(s, target_lane, sensor_fusion);
        return LaneChangeRight;
    }

    // check min distance to leading vehicle on current lane
    const auto lead_vehicle = find_closest_vehicle(s, current_lane, sensor_fusion, true);
    if (lead_vehicle[0] < MIN_LANE_CHANGE_DISTANCE_AHEAD)
    {
        // discard the overtaking maneuver
        adapt_target_speed_to_lane_speed(s, current_lane, sensor_fusion);
        return KeepLane;
    }

    // adapt speed to right lane speed
    adapt_target_speed_to_lane_speed(s, right_lane, sensor_fusion);
    return PrepareLaneChangeRight;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::lane_change_left(
    const double s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    adapt_target_speed_to_lane_speed(s, target_lane, sensor_fusion);

    if (current_lane == target_lane)
    {
        return KeepLane;
    }

    return LaneChangeLeft;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::lane_change_right(
    const double s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    adapt_target_speed_to_lane_speed(s, target_lane, sensor_fusion);

    if (current_lane == target_lane)
    {
        return KeepLane;
    }

    return LaneChangeRight;
}

bool HighwayDrivingBehavior::is_lane_safe_for_change(
    const double s, const int lane, const vector<vector<double>>& sensor_fusion)
{
    auto new_vehicle_ahead = find_closest_vehicle(s, lane, sensor_fusion, true);
    auto new_vehicle_behind = find_closest_vehicle(s, lane, sensor_fusion, false);

    // check if there is a safe gap for lane change
    return new_vehicle_ahead[0] > MIN_LANE_CHANGE_DISTANCE_AHEAD && new_vehicle_behind[0] >
        MIN_LANE_CHANGE_DISTANCE_BEHIND;
}

void HighwayDrivingBehavior::adapt_target_speed_to_lane_speed(
    const double s, const int lane, const vector<vector<double>>& sensor_fusion)
{
    // find leading vehicle on target_lane lane
    const auto lead_vehicle = find_closest_vehicle(s, lane, sensor_fusion, true);
    const auto lead_vehicle_distance = lead_vehicle[0];
    const auto lead_vehicle_speed = lead_vehicle[1];

    printf("%-22s: %4.2f m\n", "lead vehicle distance", lead_vehicle_distance);
    printf("%-22s: %4.2f m/s (%4.2f MPS)\n", "lead vehicle speed", lead_vehicle_speed,
           SpeedConverter::km_per_sec_to_miles_per_hour(lead_vehicle_speed));

    // ensure safety distance to the leading vehicle on the current lane
    // adjust the speed to the speed of the leading vehicle on the current lane
    if (lead_vehicle_distance < get_dynamic_safety_distance(lead_vehicle_speed))
        target_speed = min(lead_vehicle_speed, reference_speed_);
    else
        // drive with reference speed if the distance to leading vehicle is big enough
        target_speed = get_dynamic_speed_from_distance(lead_vehicle_distance);
}

vector<double> HighwayDrivingBehavior::find_closest_vehicle(
    const double s, const int current_lane, const vector<vector<double>>& sensor_fusion, const bool forward)
{
    double distance = 10000; // distance out of range
    auto speed = reference_speed_; // use target speed if no vehicle in lane

    // check vehicles in sensor range
    for (auto& vehicle : sensor_fusion)
    {
        const auto vehicle_lane = LaneConverter::d_to_lane(vehicle[6]);

        // check only vehicles on the same lane
        if (vehicle_lane != current_lane)
            continue;

        const auto vehicle_s = vehicle[5];

        if (forward)
        {
            // check vehicle ahead
            const auto vehicle_distance = vehicle_s - s;
            if (vehicle_s > s && vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
            }
        }
        else
        {
            // check vehicle behind 
            const auto vehicle_distance = s - vehicle_s;
            if (s >= vehicle_s && vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
            }
        }
    }

    // avoid division by zero
    return {max(1.0, distance), speed};
}

int HighwayDrivingBehavior::get_best_lane(
    const double s, const int current_lane, const double current_speed, const vector<vector<double>>& sensor_fusion)
{
    vector<double> costs = {0, 0, 0};

    auto vehicle_ahead_in_current_lane = find_closest_vehicle(s, current_lane, sensor_fusion, true);

    for (auto i = 0; i < 3; i++)
    {
        auto vehicle_ahead = find_closest_vehicle(s, i, sensor_fusion, true);
        auto vehicle_behind = find_closest_vehicle(s, i, sensor_fusion, false);

        if (vehicle_ahead[0] < 1000)
        {
            costs[i] += 25; // vehicle far ahead 
        }

        if (vehicle_ahead[0] < 150)
        {
            costs[i] += 50; // vehicle further ahead 
        }

        if (vehicle_ahead[0] < 50)
        {
            costs[i] += 100; // vehicle near ahead 
        }

        // if (i != current_lane && vehicle_ahead[0] < MIN_LANE_CHANGE_DISTANCE_AHEAD)
        // {
        // costs[i] += 1000; // vehicle near behind in other lane => not enough space for lane change
        // }

        // if (i != current_lane && vehicle_behind[0] < MIN_LANE_CHANGE_DISTANCE_BEHIND)
        // {
        // costs[i] += 1000; // vehicle near behind in other lane => not enough space for lane change
        // }

        if (vehicle_ahead_in_current_lane[1] < vehicle_ahead[1])
        {
            costs[i] += 200 - i * 200 / 2; // current lane speed slower than other lane speed
        }

        costs[i] += 10 + i * 10 / 2;
    }

    if (current_lane == 0)
    {
        // minimum cost for lanes 0 and 1
        return min_element(costs.begin(), costs.end() - 1) - costs.begin();
    }

    if (current_lane == 1)
    {
        // minimum cost for lanes 0 to 2
        return min_element(costs.begin(), costs.end()) - costs.begin();
    }

    // minimum cost for lanes 1 and 2
    return min_element(costs.begin() + 1, costs.end()) - costs.begin();
}

double HighwayDrivingBehavior::get_dynamic_safety_distance(const double lead_vehicle_speed)
{
    // rule of thumb: velocity [km/h] * 0.5 <=> safety distance [m]
    // velocity [m/s] * 3.6 = velocity [km/h] => factor = 3.6 * 0.5 = 1.8
    return max(lead_vehicle_speed * 1.8, MIN_DISTANCE); // ensure min distance
}

double HighwayDrivingBehavior::get_dynamic_speed_from_distance(const double lead_vehicle_distance)
{
    // rule of thumb: velocity [km/h] * 0,5 <=> safety distance [m]
    // distance [m/s] * 2 <=> velocity [km/h] => velocity [km/h] / 3.6 = velocity [m/s] => factor = 2 / 3.6 = 0.555
    return min(lead_vehicle_distance * 0.555, reference_speed_); // ensure max speed
}

string HighwayDrivingBehavior::state_to_string(const BehaviorState state)
{
    switch (state)
    {
    case KeepLane:
        return "Keep Lane";
    case PrepareLaneChangeLeft:
        return "Prepare Lane Change Left";
    case PrepareLaneChangeRight:
        return "Prepare Lane Change Right";
    case LaneChangeLeft:
        return "Lane Change Left";
    case LaneChangeRight:
        return "Lane Change Right";
    }
    return {};
}
