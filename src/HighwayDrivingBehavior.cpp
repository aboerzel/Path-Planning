#include "HighwayDrivingBehavior.h"
#include <math.h>
#include <algorithm>
#include "LaneConverter.h"
#include <string>

using namespace std;

#define MAX_SPEED 49.5
#define MIN_DISTANCE_TO_LEAD_VEHICLE 30.0
#define MIN_LANE_CHANGE_DISTANCE 5.0

HighwayDrivingBehavior::HighwayDrivingBehavior()
= default;

HighwayDrivingBehavior::~HighwayDrivingBehavior()
= default;

DrivingAction HighwayDrivingBehavior::get_driving_action(const double s, const int current_lane,
                                                      const vector<vector<double>>& sensor_fusion)
{
    // find leading vehicle on current lane
    const auto lead_vehicle = find_closest_vehicle(s, current_lane, sensor_fusion, true);
    const auto lead_vehicle_distance = lead_vehicle[0];
    const auto lead_vehicle_speed = lead_vehicle[1];

    // we don't need to change the lane if the distance to leading car is big enough
    if (lead_vehicle_distance > MIN_DISTANCE_TO_LEAD_VEHICLE)
    {
        printf("%-22s: %4.2f\n", "lead vehicle distance", lead_vehicle_distance);
        printf("%-22s: %4.2f\n", "lead vehicle speed", lead_vehicle_speed);

        avg_scores = {0, 0, 0}; // reset average score

        // drive at maximum speed in this case
        return DrivingAction{current_lane, MAX_SPEED};
    }

    // check if it's better to change the lane
    const auto new_lane = get_best_lane(s, current_lane, sensor_fusion);

    // check if change to the new lane is possible
    auto new_vehicle_ahead = find_closest_vehicle(s, new_lane, sensor_fusion, true);
    auto new_vehicle_behind = find_closest_vehicle(s, new_lane, sensor_fusion, false);

    // check if there is enough room to change the lane
    if (new_vehicle_ahead[0] < MIN_LANE_CHANGE_DISTANCE || new_vehicle_behind[0] < MIN_LANE_CHANGE_DISTANCE ||
        avg_scores[new_lane] <= -5)
    {
        // it's not possible to change the lane => remain on current lane
        printf("%-22s: %4.2f\n", "lead vehicle distance", lead_vehicle_distance);
        printf("%-22s: %4.2f\n", "lead vehicle speed", lead_vehicle_speed);

        // adjust the speed to the speed of the leading car on the current lane
        if (lead_vehicle_distance < MIN_DISTANCE_TO_LEAD_VEHICLE)
            return DrivingAction{current_lane, min(lead_vehicle_speed, MAX_SPEED)};

        // drive with max speed if the distance to leading car is big enough
        return DrivingAction{current_lane, MAX_SPEED};
    }

    // it's possible to change the lane => change to the new lane
    printf("%-22s: %4.2f\n", "lead vehicle distance", new_vehicle_ahead[0]);
    printf("%-22s: %4.2f\n", "lead vehicle speed", new_vehicle_ahead[1]);

    // adjust the speed to the speed of the leading car on the new lane
    if (new_vehicle_ahead[0] < MIN_DISTANCE_TO_LEAD_VEHICLE)
        return DrivingAction{new_lane, min(new_vehicle_ahead[1], MAX_SPEED)};

    // drive with max speed if the distance to leading car is big enough
    return DrivingAction{new_lane, MAX_SPEED};
}

vector<double> HighwayDrivingBehavior::find_closest_vehicle(const double s, const int lane,
                                                            const vector<vector<double>>& sensor_fusion,
                                                            const bool forward)
{
    double distance = 10000;
    auto speed = MAX_SPEED;

    // check vehicles in sensor range
    for (auto& vehicle : sensor_fusion)
    {
        const auto vehicle_s = vehicle[5];
        const auto vehicle_speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
        const auto vehicle_lane = LaneConverter::d_to_lane(vehicle[6]);

        // check only vehicles on the same lane
        if (vehicle_lane != lane)
            continue;

        if (forward)
        {
            // check vehicle ahead
            const auto vehicle_distance = vehicle_s - s;
            if (vehicle_s > s && vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = vehicle_speed;
            }
        }
        else
        {
            // check vehicle behind 
            const auto vehicle_distance = s - vehicle_s;
            if (s >= vehicle_s && vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = vehicle_speed;
            }
        }
    }

    // avoid dividing by zero
    return {max(1.0, distance), speed};
}

int HighwayDrivingBehavior::get_best_lane(const double s, const int lane, const vector<vector<double>>& sensor_fusion)
{
    vector<double> scores = {0, 0, 0};

    for (auto i = 0; i < 3; i++)
    {
        if (i == lane)
        {
            scores[i] += 0.5; // benefit to keeping lane
        }

        auto front_vehicle = find_closest_vehicle(s, i, sensor_fusion, true);
        auto back_vehicle = find_closest_vehicle(s, i, sensor_fusion, false);

        if (front_vehicle[0] > 100 && back_vehicle[0] > 100)
        {
            scores[i] += 5; // if wide open lane, move into that lane
        }
        else
        {
            if (front_vehicle[0] < 10)
            {
                scores[i] -= 5; // if car too close in front, negative score
            }

            if (back_vehicle[0] < 10)
            {
                scores[i] -= 5; // if car too close in back, negative score
            }

            scores[i] += 1 - (10 / (front_vehicle[0] / 3)); // benefit for large open distance in lane in front

            scores[i] += 1 - (10 / (back_vehicle[0] / 3)); // benefit for large open distance in lane in back

            scores[i] += 1 - (10 / (front_vehicle[1] / 2)); // benefit for faster car speed in lane in front

            scores[i] += 1 / (back_vehicle[1] / 2); // benefit for slower car speed in lane in back
        }

        // simple average calculation for scores over the last 10 iterations
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
