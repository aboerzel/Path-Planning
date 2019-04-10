#include "PathPlanner.h"
#include "Point.h"
#include "LaneConverter.h"
#include "helpers.h"
#include "PointConverter.h"
#include "spline.h"

// parameter to adjust path planning
#define WAYPOINT_DISTANCE 50.0  // distance between the waypoints of the calculated path
#define WAYPOINT_COUNT 3        // number of new waypoints to calculate
#define DT .02                  // time interval in seconds in which the path planning is called
#define PATH_LENGTH 50          // number of waypoints of the path
#define ACCELERATION .224       // acceleration

PathPlanner::PathPlanner()
{
    ref_speed = 0.0;
}

PathPlanner::~PathPlanner()
= default;

vector<Point> PathPlanner::calculate_path(const vector<double>& previous_path_x, const vector<double>& previous_path_y,
                                          const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x,
                                          const vector<double>& map_waypoints_y, const vector<vector<double>>& sensor_fusion,
                                          double car_x, double car_y, double car_yaw, double car_speed, double car_s)
{
    vector<Point> new_path;

    int prev_path_size = previous_path_x.size();

    // start with previous path
    for (auto i = 0; i < prev_path_size; i++)
    {
        new_path.push_back(Point(previous_path_x[i], previous_path_y[i]));
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

        path_points.push_back(Point(prev_car_x, prev_car_y));
        path_points.push_back(Point(car_x, car_y));

        // initialize reference speed with current car speed, to get a smooth velocity
        ref_speed = car_speed;
    }
    else
    {
        ref_y = previous_path_y[prev_path_size - 1];
        ref_x = previous_path_x[prev_path_size - 1];

        auto ref_x_prev = previous_path_x[prev_path_size - 2];
        auto ref_y_prev = previous_path_y[prev_path_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        path_points.push_back(Point(ref_x_prev, ref_y_prev));
        path_points.push_back(Point(ref_x, ref_y));
    }

    auto frenet_vec = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

    auto current_lane = LaneConverter::d_to_lane(frenet_vec[1]);

    printf("%-22s: %4.2f\n", "current speed", ref_speed);
    printf("%-22s: %d\n", "current lane", current_lane);
    printf("%-22s: %4.2f\n", "current d", frenet_vec[1]);

    auto driving_action = highway_driving_behavior.get_driving_action(frenet_vec[0], current_lane, sensor_fusion);
    auto target_d = LaneConverter::lane_to_d(driving_action.lane);

    printf("%-22s: %4.2f\n", "target speed", driving_action.speed);
    printf("%-22s: %d\n", "target lane", driving_action.lane);
    printf("%-22s: %4.2f\n", "target d", target_d);

    // add new waypoints to following the desired lane
    for (auto i = 1; i <= WAYPOINT_COUNT; i++)
    {
        auto wp = getXY(car_s + (i * WAYPOINT_DISTANCE), target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        path_points.push_back(wp);
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

    double x_offset = 0;

    for (auto i = 0; i < PATH_LENGTH - prev_path_size; i++)
    {
        // accelerate / decelerate; ensure min/max speed
        if (ref_speed < driving_action.speed - ACCELERATION)
        {
            ref_speed += ACCELERATION;
        }
        else if (ref_speed > driving_action.speed + ACCELERATION)
        {
            ref_speed -= ACCELERATION;
        }

        // calculate points along new path
        auto N = target_dist / (DT * ref_speed / (10 * ACCELERATION));
        auto x = x_offset + target_x / N;
        auto y = spline(x);

        x_offset = x;

        // convert back to map coordinates
        auto p = PointConverter::vehicle_to_map_coordinates(Point(x, y), Point(ref_x, ref_y), ref_yaw);

        new_path.push_back(p);
    }

    return new_path;
}
