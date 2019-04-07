#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "Point.h"
#include "HighwayDrivingBehavior.h"
#include "LaneConverter.h"
#include "PointConverter.h"

// for convenience
using nlohmann::json;
using namespace std;

HighwayDrivingBehavior highway_driving_behavior;

#define WAYPOINT_DISTANCE 50.0
#define WAYPOINT_COUNT 3
#define DT .02
#define PATH_LENGTH 50
#define MAX_ACC .224

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    auto ref_speed = 0.0; // reference speed [mph]

    h.onMessage([&ref_speed, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx,&map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode)
        {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            if (length && length > 2 && data[0] == '4' && data[1] == '2')
            {
                auto s = hasData(data);

                if (s != "")
                {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                        // j[1] is the data JSON object

                        // Main car's localization Data
                        double car_x = j[1]["x"];
                        double car_y = j[1]["y"];
                        double car_s = j[1]["s"];
                        double car_d = j[1]["d"];
                        double car_yaw = j[1]["yaw"];
                        double car_speed = j[1]["speed"];

                        // Previous path data given to the Planner
                        auto previous_path_x = j[1]["previous_path_x"];
                        auto previous_path_y = j[1]["previous_path_y"];
                        // Previous path's end s and d values 
                        double end_path_s = j[1]["end_path_s"];
                        double end_path_d = j[1]["end_path_d"];

                        // Sensor Fusion Data, a list of all other cars on the same side 
                        //   of the road.
                        auto sensor_fusion = j[1]["sensor_fusion"];

                        json msgJson;

                        vector<double> next_x_vals;
                        vector<double> next_y_vals;

                        /**
                         * TODO: define a path made up of (x,y) points that the car will visit
                         *   sequentially every .02 seconds
                         */

                        int prev_path_size = previous_path_x.size();
              
                        // start with previous path
                        for (auto i = 0; i < prev_path_size; i++)
                        {
                            next_x_vals.push_back(previous_path_x[i]);
                            next_y_vals.push_back(previous_path_y[i]);
                        }
             
                        auto ref_x = car_x;
                        auto ref_y = car_y;
                        auto ref_yaw = deg2rad(car_yaw);
               
                        vector<Point> path_points;

                        if (prev_path_size < 2) 
                        {                       
                            auto prev_car_x = car_x - cos(car_yaw);
                            auto prev_car_y = car_y - sin(car_yaw);

                            path_points.push_back(Point(prev_car_x, prev_car_y));
                            path_points.push_back(Point(car_x, car_y));

                            // initialize reference speed with current car speed
                            ref_speed = car_speed;
                        }
                        else 
                        {
                            ref_y = previous_path_y[prev_path_size - 1];

                            ref_x = previous_path_x[prev_path_size - 1];
                            double ref_x_prev = previous_path_x[prev_path_size - 2];
                            double ref_y_prev = previous_path_y[prev_path_size - 2];

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
                        for (auto i=1; i <= WAYPOINT_COUNT; i++)
                        {
                            auto wp = getXY(car_s + (i * WAYPOINT_DISTANCE), target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                            path_points.push_back(wp);
                        }

                        // convert spline points from map coordinates to vehicle coordinates
                        for (auto i = 0; i < path_points.size(); i++)
                        {
                            path_points[i] = PointConverter::map_to_vehicle_coordinates(path_points[i], Point(ref_x, ref_y), ref_yaw);
                        }

                        // create spline
                        tk::spline spline;
                        spline.set_points(get_x_values(path_points), get_y_values(path_points));

                        auto target_x = WAYPOINT_DISTANCE;
                        auto target_y = spline(target_x);
                        auto target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

                        double x_offset = 0;
                       
                        for (auto i = 0; i < PATH_LENGTH - prev_path_size; i++)
                        {
                            // accelerate / decelerate; ensure min/max speed
                            if (ref_speed < driving_action.speed - MAX_ACC)
                            {
                                ref_speed += MAX_ACC;
                            }
                            else if (ref_speed > driving_action.speed + MAX_ACC)
                            {
                                ref_speed -= MAX_ACC;
                            }

                            // calculate points along new path
                            auto N = target_dist / (DT * ref_speed / (10 * MAX_ACC));
                            auto x = x_offset + target_x / N;
                            auto y = spline(x);

                            x_offset = x;

                            // convert back to map coordinates
                            auto p = PointConverter::vehicle_to_map_coordinates(Point(x, y), Point(ref_x, ref_y), ref_yaw);

                            next_x_vals.push_back(p.X);
                            next_y_vals.push_back(p.Y);
                        }

                        msgJson["next_x"] = next_x_vals;
                        msgJson["next_y"] = next_y_vals;

                        auto msg = "42[\"control\"," + msgJson.dump() + "]";

                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    } // end "telemetry" if
                }
                else
                {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } // end websocket if
        }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char* message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
