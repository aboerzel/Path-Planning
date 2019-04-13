#include "HighwayDrivingBehavior.h"
#include <math.h>
#include <algorithm>
#include "LaneConverter.h"
#include <string>
#include "SpeedConverter.h"

using namespace std;

// parameter to adjust driving behavior
#define MAX_SPEED 22.12848                      // maximum speed in [m/sec] => 49.5 MPS
#define MIN_LANE_CHANGE_DISTANCE_AHEAD 30.0     // minimum distance [m] to the vehicle ahead on the target lane, needed for lane change
#define MIN_LANE_CHANGE_DISTANCE_BEHIND 30.0    // minimum distance [m] to the vehicle behind on the target lane, needed for lane change
#define MIN_DISTANCE 5.0                        // minimum distance [m] to the vehicle ahead regardless of the speed

HighwayDrivingBehavior::HighwayDrivingBehavior()
{
    // our speed target is the maximum speed allowed on highways
    target_speed = MAX_SPEED;
}

HighwayDrivingBehavior::~HighwayDrivingBehavior()
= default;

DrivingAction HighwayDrivingBehavior::get_driving_action(const double s, const int current_lane,
                                                         const vector<vector<double>>& sensor_fusion)
{
    // find leading vehicle on current lane
    const auto lead_vehicle = find_closest_vehicle(s, current_lane, sensor_fusion, true);
    const auto lead_vehicle_distance = lead_vehicle[0];
    const auto lead_vehicle_speed = lead_vehicle[1];

    // check if it's better to change the lane
    const auto new_lane = get_best_lane(s, current_lane, sensor_fusion);

    // check if change to the new lane is possible
    auto new_vehicle_ahead = find_closest_vehicle(s, new_lane, sensor_fusion, true);
    auto new_vehicle_behind = find_closest_vehicle(s, new_lane, sensor_fusion, false);

    // check if there is enough room to change the lane
    if (new_vehicle_ahead[0] < MIN_LANE_CHANGE_DISTANCE_AHEAD || new_vehicle_behind[0] < MIN_LANE_CHANGE_DISTANCE_BEHIND)
    {
        // it's not possible to change the lane => remain on current lane
        printf("%-22s: %4.2f m\n", "lead vehicle distance", lead_vehicle_distance);
        printf("%-22s: %4.2f m/s (%4.2f MPS)\n", "lead vehicle speed", lead_vehicle_speed,
               SpeedConverter::km_per_sec_to_miles_per_hour(lead_vehicle_speed));

        // ensure safety distance to the leading vehicle on the current lane
        // adjust the speed to the speed of the leading vehicle on the current lane
        if (lead_vehicle_distance < get_dynamic_safety_distance(lead_vehicle_speed))
            return DrivingAction{current_lane, min(lead_vehicle_speed, target_speed)};

        // drive with max speed if the distance to leading vehicle is big enough
        return DrivingAction{current_lane, get_dynamic_speed_from_distance(lead_vehicle_distance)};
    }

    // it's possible to change the lane => change to the new lane
    printf("%-22s: %4.2f m\n", "lead vehicle distance", new_vehicle_ahead[0]);
    printf("%-22s: %4.2f m/s (%4.2f MPS)\n", "lead vehicle speed", new_vehicle_ahead[1],
           SpeedConverter::km_per_sec_to_miles_per_hour(new_vehicle_ahead[1]));

    // ensure safety distance to the leading vehicle on the new lane
    // adjust the speed to the speed of the leading vehicle on the new lane
    if (new_vehicle_ahead[0] < get_dynamic_safety_distance(new_vehicle_ahead[1]))
        return DrivingAction{new_lane, min(new_vehicle_ahead[1], target_speed)};

    // drive with max speed if the distance to leading vehicle is big enough
    return DrivingAction{new_lane, get_dynamic_speed_from_distance(new_vehicle_ahead[0])};
}

vector<double> HighwayDrivingBehavior::find_closest_vehicle(const double s, const int lane,
                                                            const vector<vector<double>>& sensor_fusion,
                                                            const bool forward)
{
    double distance = 10000; // distance out of range
    auto speed = target_speed; // use target speed if no vehicle in lane

    // check vehicles in sensor range
    for (auto& vehicle : sensor_fusion)
    {
        const auto vehicle_lane = LaneConverter::d_to_lane(vehicle[6]);

        // check only vehicles on the same lane
        if (vehicle_lane != lane)
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

int HighwayDrivingBehavior::get_best_lane(const double s, const int lane, const vector<vector<double>>& sensor_fusion)
{
    vector<double> scores = {0, 0, 0};

    auto vehicle_ahead_in_lane = find_closest_vehicle(s, lane, sensor_fusion, true);

    for (auto i = 0; i < 3; i++)
    {
        auto vehicle_ahead = find_closest_vehicle(s, i, sensor_fusion, true);
        auto vehicle_behind = find_closest_vehicle(s, i, sensor_fusion, false);

        if (i == lane && vehicle_ahead[0] > 1000)
        {
            scores[i] += 40; // no vehicle ahead in current lane => keep current lane
        }
        else if (i != lane && vehicle_ahead[0] > 1000 && vehicle_behind[0] > MIN_LANE_CHANGE_DISTANCE_BEHIND)
        {
            scores[i] += 35; // wide open lane => change to that lane
        }

        if (vehicle_ahead[0] > MIN_LANE_CHANGE_DISTANCE_AHEAD)
        {
            scores[i] += 5; // distance to the vehicle ahead is too close, negative score
        }

        if (vehicle_behind[0] > MIN_LANE_CHANGE_DISTANCE_BEHIND)
        {
            scores[i] += 5; // distance to the vehicle behind is too close, negative score
        }

        if (vehicle_ahead[0] > 1000)
        {
            scores[i] += 10; // large distance to the vehicle ahead
        }

        if (vehicle_behind[0] > 1000)
        {
            scores[i] += 10; // large distance to the vehicle behind
        }

        if (vehicle_ahead[1] > vehicle_ahead_in_lane[1])
        {
            scores[i] += 10; // faster speed ahead
        }

        // Use the average of the last 10 scores to avoid massive changing driving actions
        avg_scores[i] = (avg_scores[i] * 10) - avg_scores[i];
        avg_scores[i] += scores[i];
        avg_scores[i] /= 10;
    }

    if (lane == 0)
    {
        // best score for lanes 0 and 1
        return max_element(avg_scores.begin(), avg_scores.end() - 1) - avg_scores.begin();
    }

    if (lane == 1)
    {
        // best score for lanes 0 to 2
        return max_element(avg_scores.begin(), avg_scores.end()) - avg_scores.begin();
    }

    // best score for lanes 1 and 2
    return max_element(avg_scores.begin() + 1, avg_scores.end()) - avg_scores.begin();
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
    return min(lead_vehicle_distance * 0.555, target_speed); // ensure max speed
}
