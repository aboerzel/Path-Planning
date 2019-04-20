#include "HighwayDrivingBehavior.h"
#include <math.h>
#include <algorithm>
#include "LaneConverter.h"
#include <string>
#include "SpeedConverter.h"
#include <iostream>
#include <exception>
#include "GlobalSettings.h"

using namespace std;

// parameter to adjust driving behavior
#define MAX_SPEED 22.12848                      // maximum speed in [m/sec] => 49.5 MPS
#define MAX_ACCEL 8.4                           // max acceleration [m/s²]
#define MIN_LANE_CHANGE_DISTANCE 25.0           // minimum distance [m] to the vehicle ahead on the target lane, needed for lane change
#define MIN_DISTANCE 5.0                        // minimum distance [m] to the vehicle ahead regardless of the speed
#define MAX_PREPARE_LANE_CHANGE_DURATION 6	    // maximum duration to prepare a lane change [s]
#define MIN_WAIT_TIME_BETWEEN_LANE_CHANGES 3    // minimum wait time between two lane changes [s]

HighwayDrivingBehavior::HighwayDrivingBehavior()
{
    reference_speed_ = MAX_SPEED; // our reference speed is the maximum speed allowed on highways

    target_speed = 0.0; // target speed [m/s]
    target_accel = MAX_ACCEL; // here we use a simple constant acceleration [m/s?]
    target_lane = -1;
    preferred_lane_ = -1;
    prepare_lane_change_time_ = 0;
    duration_since_last_lane_change_ = 0;

    current_state_ = KeepLane; // initial behavior state
}

HighwayDrivingBehavior::~HighwayDrivingBehavior()
= default;

void HighwayDrivingBehavior::update(const double current_s, const int current_lane, const double current_speed,
                                    const vector<vector<double>>& sensor_fusion)
{
    printf("%-32s: %s\n", "current state", state_to_string(current_state_).c_str());
    printf("%-32s: %d\n", "preferred lane", preferred_lane_);
    printf("%-32s: %4.2f s\n", "prepare lane change duration", prepare_lane_change_time_);
    printf("%-32s: %4.2f s\n", "duration since last lane change", duration_since_last_lane_change_);

    switch (current_state_)
    {
    case KeepLane:
        {
            current_state_ = keep_lane(current_s, current_lane, sensor_fusion);
            break;
        }
    case PrepareLaneChangeLeft:
        {
            current_state_ = prepare_lane_change_left(current_s, current_lane, current_speed, sensor_fusion);
            break;
        }
    case PrepareLaneChangeRight:
        {
            current_state_ = prepare_lane_change_right(current_s, current_lane, current_speed, sensor_fusion);
            break;
        }
    case LaneChangeLeft:
        {
            current_state_ = lane_change_left(current_s, current_lane, sensor_fusion);
            break;
        }
    case LaneChangeRight:
        {
            current_state_ = lane_change_right(current_s, current_lane, sensor_fusion);
            break;
        }
    default:
        throw std::range_error("Unexpected behavior state");
    }

    printf("%-32s: %s\n", "new state", state_to_string(current_state_).c_str());

    duration_since_last_lane_change_ += DT;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::keep_lane(
    const double current_s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    prepare_lane_change_time_ = 0;
    target_lane = current_lane;

    adapt_target_speed_to_lane_speed(current_s, target_lane, sensor_fusion);

    if (duration_since_last_lane_change_ >= MIN_WAIT_TIME_BETWEEN_LANE_CHANGES)
    {
        // search for the best lane
        preferred_lane_ = get_best_lane(current_s, sensor_fusion);

        if (preferred_lane_ < target_lane)
        {
            return PrepareLaneChangeLeft;
        }

        if (preferred_lane_ > target_lane)
        {
            return PrepareLaneChangeRight;
        }
    }

    return KeepLane;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::prepare_lane_change_left(
    const double current_s, const int current_lane, const double current_speed,
    const vector<vector<double>>& sensor_fusion)
{
    const auto left_lane = current_lane - 1;

    // check if left lane is safe for lane change
    if (is_lane_safe_for_change(current_s, current_lane, left_lane, current_speed, sensor_fusion))
    {
        // change lane
        target_lane = left_lane;
        adapt_target_speed_to_lane_speed(current_s, target_lane, sensor_fusion);
        return LaneChangeLeft;
    }

    // remain on current lane
    adapt_target_speed_to_lane_speed(current_s, current_lane, sensor_fusion);

    // check min distance to leading vehicle on current lane
    const auto lead_vehicle = find_closest_vehicle(current_s, current_lane, sensor_fusion, true);
    if (lead_vehicle[0] < MIN_LANE_CHANGE_DISTANCE)
    {
        // discard the overtaking maneuver
        return KeepLane;
    }

    // check max prepare lane change duration
    if (prepare_lane_change_time_ > MAX_PREPARE_LANE_CHANGE_DURATION)
    {
        // discard the overtaking maneuver
        return KeepLane;
    }

    prepare_lane_change_time_ += DT;

    return PrepareLaneChangeLeft;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::prepare_lane_change_right(
    const double current_s, const int current_lane, const double current_speed,
    const vector<vector<double>>& sensor_fusion)
{
    const auto right_lane = current_lane + 1;

    // check if right lane is a safe for lane change
    if (is_lane_safe_for_change(current_s, current_lane, right_lane, current_speed, sensor_fusion))
    {
        // change lane
        target_lane = right_lane;
        adapt_target_speed_to_lane_speed(current_s, target_lane, sensor_fusion);
        return LaneChangeRight;
    }

    // remain on current lane
    adapt_target_speed_to_lane_speed(current_s, current_lane, sensor_fusion);

    // check min distance to leading vehicle on current lane
    const auto vehicle_ahead = find_closest_vehicle(current_s, current_lane, sensor_fusion, true);
    if (vehicle_ahead[0] < MIN_LANE_CHANGE_DISTANCE)
    {
        // discard the overtaking maneuver
        return KeepLane;
    }

    // check max prepare lane change duration
    if (prepare_lane_change_time_ > MAX_PREPARE_LANE_CHANGE_DURATION)
    {
        // discard the overtaking maneuver
        return KeepLane;
    }

    prepare_lane_change_time_ += DT;

    return PrepareLaneChangeRight;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::lane_change_left(
    const double current_s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    prepare_lane_change_time_ = 0;

    adapt_target_speed_to_lane_speed(current_s, target_lane, sensor_fusion);

    if (current_lane == target_lane)
    {
        duration_since_last_lane_change_ = 0;
        return KeepLane;
    }

    return LaneChangeLeft;
}

HighwayDrivingBehavior::BehaviorState HighwayDrivingBehavior::lane_change_right(
    const double current_s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    prepare_lane_change_time_ = 0;

    adapt_target_speed_to_lane_speed(current_s, target_lane, sensor_fusion);

    if (current_lane == target_lane)
    {
        duration_since_last_lane_change_ = 0;
        return KeepLane;
    }

    return LaneChangeRight;
}

bool HighwayDrivingBehavior::is_lane_safe_for_change(
    const double current_s, const int current_lane, const int lane, const double current_speed, const vector<vector<double>>& sensor_fusion)
{
    // check distance to leading vehicle on current lane
    auto lead_vehicle = find_closest_vehicle(current_s, current_lane, sensor_fusion, true);
    // leading vehicle on current lane to close
    const auto min_distance = max(current_speed * 0.5, 15.0);
    if (lead_vehicle[0] < min_distance)
    {
        printf("BLOCKING: Distance %4.2f m to vehicle ahead on lane %d < %4.2f m\n", lead_vehicle[0], current_lane, min_distance);
        return false;
    }

    // check vehicles in sensor range
    for (auto& vehicle : sensor_fusion)
    {
        const auto vehicle_lane = LaneConverter::d_to_lane(vehicle[6]);
     
        // check only vehicles on target lane
        if (vehicle_lane != lane)
            continue;

        const auto vehicle_s = vehicle[5];
        const auto vehicle_distance = vehicle_s - current_s;

        // check vehicle ahead
        if (vehicle_distance >= 0 && vehicle_distance < MIN_LANE_CHANGE_DISTANCE + 20)
        {
            // vehicle too close 
            if (vehicle_distance < MIN_LANE_CHANGE_DISTANCE + 10)
            {
                printf("BLOCKING: Distance %4.2f m to vehicle ahead on lane %d < %4.2f m\n", vehicle_distance, lane, MIN_LANE_CHANGE_DISTANCE + 10);
                return false;
            }

            const auto vehicle_speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
            // vehicle ahead drives slower
            if (vehicle_speed < current_speed)
            {
                printf("BLOCKING: Speed %4.2f m/s of vehicle ahead on lane %d < current speed %4.2f m/s\n", vehicle_speed, lane, current_speed);
                return false;
            }         
        }

        // check vehicle behind
        if (vehicle_distance <= 0 && abs(vehicle_distance) < MIN_LANE_CHANGE_DISTANCE + 20)
        {
            // vehicle too close 
            if (abs(vehicle_distance) < MIN_LANE_CHANGE_DISTANCE - 5)
            {
                printf("BLOCKING: Distance %4.2f m to vehicle behind on lane %d < %4.2f m\n", vehicle_distance, lane, MIN_LANE_CHANGE_DISTANCE - 5);
                return false;
            }

            const auto vehicle_speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
            // vehicle behind drives faster
            if (vehicle_speed > current_speed)
            {
                printf("BLOCKING: Speed %4.2f m/s of vehicle ahead on lane %d < current speed %4.2f m/s\n", vehicle_speed, lane, current_speed);
                return false;
            }
        }
    }

    return true;
}

void HighwayDrivingBehavior::adapt_target_speed_to_lane_speed(
    const double current_s, const int lane, const vector<vector<double>>& sensor_fusion)
{
    // find leading vehicle on target_lane lane
    const auto lead_vehicle = find_closest_vehicle(current_s, lane, sensor_fusion, true);
    const auto lead_vehicle_distance = lead_vehicle[0];
    const auto lead_vehicle_speed = lead_vehicle[1];

    printf("%-32s: %4.2f m\n", ("lead vehicle distance [lane " + to_string(lane) + "]").c_str(), lead_vehicle_distance);
    printf("%-32s: %4.2f m/s (%4.2f MPS)\n", ("lead vehicle speed    [lane " + to_string(lane) + "]").c_str(), lead_vehicle_speed,
           SpeedConverter::km_per_sec_to_miles_per_hour(lead_vehicle_speed));

    // ensure safety distance to the leading vehicle on the current lane
    // adjust the speed to the speed of the leading vehicle on the current lane
    if (lead_vehicle_distance < get_dynamic_safety_distance(lead_vehicle_speed) && lead_vehicle_distance > 15)
        target_speed = min(lead_vehicle_speed, reference_speed_);
    else
        // drive with reference speed if the distance to leading vehicle is big enough
        target_speed = get_dynamic_speed_from_distance(lead_vehicle_distance);
}

vector<double> HighwayDrivingBehavior::find_closest_vehicle(
    const double current_s, const int current_lane, const vector<vector<double>>& sensor_fusion,
    const bool forward)
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
            const auto vehicle_distance = vehicle_s - current_s;
            if (vehicle_distance >= 0.0 && vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
            }
        }
        else
        {
            // check vehicle behind
            const auto vehicle_distance = current_s - vehicle_s;
            if (vehicle_distance > 0.0 && vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
            }
        }
    }

    // avoid division by zero
    return {max(0.1, distance), speed};
}

vector<double> HighwayDrivingBehavior::get_average_lane_speed(
    const double current_s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    auto lane_speed_sum = 0.0;
    auto num_vehicles = 0.0;

    // check vehicles in sensor range
    for (auto& vehicle : sensor_fusion)
    {
        const auto vehicle_lane = LaneConverter::d_to_lane(vehicle[6]);

        // check only vehicles on the same lane
        if (vehicle_lane != current_lane)
            continue;

        const auto vehicle_s = vehicle[5];

        // check vehicle ahead
        const auto vehicle_distance = vehicle_s - current_s;
        if (vehicle_distance > 0.0)
        {
            lane_speed_sum += sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
            num_vehicles++;
        }
    }

    return {num_vehicles != 0 ? lane_speed_sum / num_vehicles : -1, num_vehicles};
}

int HighwayDrivingBehavior::get_best_lane(const double current_s, const vector<vector<double>>& sensor_fusion)
{
    vector<double> costs = {0, 0, 0};

    for (auto i = 0; i < 3; i++)
    {
        auto vehicle_ahead = find_closest_vehicle(current_s, i, sensor_fusion, true);
        auto avg_lane_speed_result = get_average_lane_speed(current_s, i, sensor_fusion);

        // punish the more vehicles drive ahead in lane
        if (avg_lane_speed_result[1] > 0)
        {
            costs[i] += avg_lane_speed_result[1] / 10;
        }

        // punish the closeness to the vehicle ahead in lane
        if (vehicle_ahead[0] != -1 && vehicle_ahead[0] <= 500)
        {
            costs[i] += (500 - vehicle_ahead[0]) / 500 * 20;
        }

        // punish low lane speed
        if (avg_lane_speed_result[1] > 0)
        {
            costs[i] += (85 - avg_lane_speed_result[0]) / 85 * 30;
        }

        // punish left lanes => prefer right lanes
        costs[i] += 2 - (2 - i);
    }

    // return lane with minimum cost
    return min_element(costs.begin(), costs.end()) - costs.begin();
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
