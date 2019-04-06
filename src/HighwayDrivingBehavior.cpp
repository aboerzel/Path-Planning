#include "HighwayDrivingBehavior.h"
#include <iso646.h>
#include <math.h>
#include <algorithm>
#include "LaneDConverter.h"
#include <iostream>
#include <string>

using namespace std;

#define MAX_ALLOWED_SPEED 50.0                          
#define SPEED_BUFFER 5.0                                
#define MAX_SPEED (MAX_ALLOWED_SPEED - SPEED_BUFFER)   
#define MIN_DISTANCE_TO_LEAD 5.0                      

HighwayDrivingBehavior::HighwayDrivingBehavior()
{
    target_vehicle_speed = 0.0;
}

HighwayDrivingBehavior::~HighwayDrivingBehavior()
= default;

int HighwayDrivingBehavior::get_target_lane(const double s, const int current_lane, const vector<vector<double>>& sensor_fusion)
{
    const auto lead_vehicle = find_closest_vehicle(s, current_lane, sensor_fusion, true);
    const auto lead_vehicle_distance = lead_vehicle[0];
    const auto leed_vehicle_speed = lead_vehicle[1];

    // check if blocked, i.e. car is within 20 meters
    if (lead_vehicle_distance > MIN_DISTANCE_TO_LEAD)
    {
        // if lots of space, stay in lane and go near the speed limit
        auto new_lane = current_lane;
        target_vehicle_speed = MAX_SPEED;
        avg_scores = {0, 0, 0}; // reset average score
        return new_lane;
    }

    auto new_lane = get_lane_score(s, current_lane, sensor_fusion);
  
    cout << "new_lane: " << new_lane << endl;

    // check that the car has not incorrectly chose a blocked lane
    auto front_vehicle = find_closest_vehicle(s, new_lane, sensor_fusion, true);
    auto back_vehicle = find_closest_vehicle(s, new_lane, sensor_fusion, false);

    // stay in current lane if not enough room for lane change
    if (front_vehicle[0] < 5 or back_vehicle[0] < 5 or avg_scores[new_lane] <= -5)
    {
        new_lane = current_lane;
        
        if (lead_vehicle[0] < 5)
            target_vehicle_speed = min(leed_vehicle_speed, MAX_SPEED);
        else
            target_vehicle_speed = MAX_SPEED;
    }
    else
    {
        if (front_vehicle[0] < 5)
            target_vehicle_speed = min(front_vehicle[1], MAX_SPEED);
        else
            target_vehicle_speed = MAX_SPEED;
    }

    return new_lane;
}

vector<double> HighwayDrivingBehavior::find_closest_vehicle(const double s, const int lane, const vector<vector<double>>& sensor_fusion, const bool forward)
{
    double distance = 10000;
    auto speed = MAX_SPEED; 

    // check vehicles in sensor range
    for (auto& vehicle : sensor_fusion)
    {
        const auto vehicle_s = vehicle[5];
        const auto vehicle_speed = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
        const auto vehicle_lane = LaneDConverter::d_to_lane(vehicle[6]);

        // check only vehicles on the same lane
        if (vehicle_lane != lane)
            continue;
 
        if (forward)
        {
            // check vehicle ahead
            const auto vehicle_distance = vehicle_s - s;
            if (vehicle_s > s and vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = vehicle_speed;
            }
        }
        else
        {
            // check behind 
            const auto vehicle_distance = s - vehicle_s;
            if (s >= vehicle_s and vehicle_distance < distance)
            {
                distance = vehicle_distance;
                speed = vehicle_speed;
            }
        }
        
    }

    // avoid dividing by zero
    return {min(1.0, distance), speed};
}

int HighwayDrivingBehavior::get_lane_score(const double s, const int lane, const vector<vector<double>>& sensor_fusion)
{
    vector <double> scores = { 0,0,0 };

    for (auto i = 0; i < 3; i++) 
    {
        if (i == lane) 
        {  
            scores[i] += 0.5; // benefit to keeping lane
        }

        auto front_vehicle = find_closest_vehicle(s, i, sensor_fusion, true);
        auto back_vehicle = find_closest_vehicle(s, i, sensor_fusion, false);

        if (front_vehicle[0] > 100 and back_vehicle[0] > 100) 
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

        // simple average calculation for scores over the last ten iterations
        avg_scores[i] = (avg_scores[i] * 10) - avg_scores[i];
        avg_scores[i] += scores[i];
        avg_scores[i] /= 10;

       // cout << "score " << i << " : " << to_string(avg_scores[i]) << endl;
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
