#include "TrajectoryPlanner.h"
#include "Point.h"
#include "LaneConverter.h"
#include "helpers.h"
#include "PointConverter.h"
#include "spline.h"
#include "SpeedConverter.h"
#include "GlobalSettings.h"

// parameter to adjust path planning
#define WAYPOINT_DISTANCE 48        // distance between the waypoints of the calculated path
#define WAYPOINT_COUNT 4            // number of new waypoints to calculate
#define PATH_LENGTH 50              // number of waypoints of the path

TrajectoryPlanner::TrajectoryPlanner()
{
    ref_speed = 0.0; // reference speed [m/s]
}

TrajectoryPlanner::~TrajectoryPlanner()
= default;

vector<Point> TrajectoryPlanner::generate_trajectory(const vector<double>& previous_path_x,
                                                     const vector<double>& previous_path_y,
                                                     const vector<double>& map_waypoints_s,
                                                     const vector<double>& map_waypoints_x,
                                                     const vector<double>& map_waypoints_y,
                                                     const vector<vector<double>>& sensor_fusion,
                                                     double car_x, double car_y, double car_yaw, double car_speed,
                                                     double car_s, double car_d)
{
    vector<Point> trajectory;

    int prev_path_size = previous_path_x.size();

    // start with previous path
    for (auto i = 0; i < prev_path_size; i++)
    {
        trajectory.emplace_back(previous_path_x[i], previous_path_y[i]);
    }

    double ref_x;
    double ref_y;
    double ref_yaw;

    vector<Point> path_points;

    if (prev_path_size < 2)
    {
        ref_x = car_x;
        ref_y = car_y;
        ref_yaw = deg2rad(car_yaw);

        auto prev_car_x = car_x - cos(car_yaw);
        auto prev_car_y = car_y - sin(car_yaw);

        path_points.emplace_back(prev_car_x, prev_car_y);
        path_points.emplace_back(car_x, car_y);

        // initialize reference speed [m/s] with current car speed [MPS], to get a smooth velocity
        ref_speed = SpeedConverter::miles_per_hour_to_km_per_sec(car_speed);
    }
    else
    {
        ref_y = previous_path_y[prev_path_size - 1];
        ref_x = previous_path_x[prev_path_size - 1];

        auto ref_x_prev = previous_path_x[prev_path_size - 2];
        auto ref_y_prev = previous_path_y[prev_path_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        path_points.emplace_back(ref_x_prev, ref_y_prev);
        path_points.emplace_back(ref_x, ref_y);
    }

    auto current_lane = LaneConverter::d_to_lane(car_d);

    printf("%-32s: %4.2f m/s (%4.2f MPS)\n", "current speed", ref_speed,
           SpeedConverter::km_per_sec_to_miles_per_hour(ref_speed));
    printf("%-32s: %d\n", "current lane", current_lane);
    printf("%-32s: %4.2f\n", "current d", car_d);

    highway_driving_behavior.update(car_s, current_lane, car_speed, sensor_fusion);

    auto target_d = LaneConverter::lane_to_d(highway_driving_behavior.target_lane);

    printf("%-32s: %4.2f m/s (%4.2f MPS)\n", "target speed", highway_driving_behavior.target_speed,
           SpeedConverter::km_per_sec_to_miles_per_hour(highway_driving_behavior.target_speed));
    printf("%-32s: %d\n", "target lane", highway_driving_behavior.target_lane);
    printf("%-32s: %4.2f\n", "target d", target_d);

    // add new waypoints to following the desired lane
    for (auto i = 1; i <= WAYPOINT_COUNT; i++)
    {
        auto wp = getXY(car_s + (i * WAYPOINT_DISTANCE), target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        path_points.emplace_back(wp);
    }

    // convert path points from map coordinates to vehicle coordinates
    for (auto& path_point : path_points)
    {
        path_point = PointConverter::map_to_vehicle_coordinates(path_point, Point(ref_x, ref_y), ref_yaw);
    }

    // create spline from path points
    tk::spline spline;
    spline.set_points(get_x_values(path_points), get_y_values(path_points));

    auto target_x = WAYPOINT_DISTANCE;
    auto target_y = spline(target_x);
    auto target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

    auto x_offset = 0.0;
    auto speed_diff = highway_driving_behavior.target_accel * DT; // speed difference per time interval DT [m/s]

    // create new path points and add them to the new path
    for (auto i = 0; i < PATH_LENGTH - prev_path_size; i++)
    {
        // accelerate / decelerate; ensure target speed
        if (ref_speed < highway_driving_behavior.target_speed - speed_diff)
        {
            ref_speed += speed_diff;
        }
        else if (ref_speed > highway_driving_behavior.target_speed + speed_diff)
        {
            ref_speed -= speed_diff;
        }

        // calculate points along new path
        auto delta_s = ref_speed * DT; // distance per time interval DT [m] 
        auto x = x_offset + target_x * delta_s / target_dist;
        auto y = spline(x);

        x_offset = x;

        // convert back to map coordinates
        auto p = PointConverter::vehicle_to_map_coordinates(Point(x, y), Point(ref_x, ref_y), ref_yaw);

        trajectory.push_back(p);
    }

    return trajectory;
}
