#include "BehaviorPlanner.h"
#include <iso646.h>
#include <math.h>

#define MAX_SPEED 22.352
#define SPEED_BUFFER 0.5
#define MIN_DISTANCE_TO_LEAD 20

BehaviorPlanner::BehaviorPlanner()
{
    target_vehicle_speed = 0.0;
}

BehaviorPlanner::~BehaviorPlanner()
{
}

int BehaviorPlanner::get_target_lane(const double s, const int current_lane, vector<vector<double>> sensor_fusion)
{
    const auto lead_vehicle = find_closest_vehicle(s, current_lane, sensor_fusion, true);
    const auto lead_vehicle_distance = lead_vehicle[0];
    const auto leed_vehicle_speed = lead_vehicle[1];

    // check if blocked, i.e. car is within 20 meters
    if (lead_vehicle_distance > MIN_DISTANCE_TO_LEAD)
    {
        // if lots of space, stay in lane and go near the speed limit
        auto new_lane = current_lane;
        target_vehicle_speed = MAX_SPEED - SPEED_BUFFER;
        avg_scores = {0, 0, 0}; // Reset average scores for laneScore()
        return new_lane;
    }

    auto new_lane = get_lane_score(s, current_lane, sensor_fusion);

    // check that the car has not incorrectly chose a blocked lane
    auto front_vehicle = find_closest_vehicle(s, new_lane, sensor_fusion, true);
    auto back_vehicle = find_closest_vehicle(s, new_lane, sensor_fusion, false);

    // Reset to current lane and leading vehicle if not enough room
    if (front_vehicle[0] < 10 or back_vehicle[0] < 10 or avg_scores[new_lane] <= -5)
    {
        new_lane = current_lane;
        target_vehicle_speed = leed_vehicle_speed;
    }

    return new_lane;
}

vector<double> BehaviorPlanner::find_closest_vehicle(const double s, const int lane, vector<vector<double>> sensor_fusion, const bool direction)
{
    double distance = 10000;
    auto speed = 22.352 - 0.5; // Set in case of no cars

    // Check each vehicle in sensor range
    for (auto& vehicle : sensor_fusion)
    {
        const auto vehicle_s = vehicle[5];
        const auto vehicle_speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
        const auto vehicle_lane = calculate_lane(vehicle[6]);

        if (vehicle_lane == lane)
        {
            // if same lane
            if (direction)
            {
                const auto vehicle_distance = vehicle_s - s;
                if (vehicle_s > s and vehicle_distance < distance)
                {
                    // and ahead of my vehicle
                    distance = vehicle_distance;
                    speed = vehicle_speed;
                }
            }
            else
            {
                const auto vehicle_distance = s - vehicle_s;
                if (s >= vehicle_s and vehicle_distance < distance)
                {
                    // if behind my vehicle
                    distance = vehicle_distance;
                    speed = vehicle_speed;
                }
            }
        }
    }

    if (distance <= 0)
    {
        // Avoid dividing by zero
        distance = 1.0;
    }

    return {distance, speed};
}

int BehaviorPlanner::get_lane_score(double s, int lane, vector<vector<double>> sensor_fusion)
{
    return 0;
}

int BehaviorPlanner::calculate_lane(double d)
{
    // Check which lane the d-value comes from
    // Left is 0, middle is 1, right is 2

    if (d < 4)
    {
        return 0;
    }

    if (d < 8)
    {
        return 1;
    }

    return 2;
}
