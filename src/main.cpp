#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

int main() {
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
  while (getline(in_map_, line)) {
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
  
  // In simulator car starts at middle lane
  int lane = 1;
  double initial_vel = 0.0; // in mph
  h.onMessage([&lane,&initial_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
            
            // Core code starts here //
            
            // Step 1: Prediction
            
            // Get other car's position
            bool front_car = false;
            bool leftlane_car = false;
            bool rightlane_car = false;
            
            for (int i = 0; i < sensor_fusion.size(); i++) {
                double d = sensor_fusion[i][6];
                int car_lane = -1;
                
                // Check whether there is car in the ego vehicle lane
                if (d > 0 && d < 4) {
                    car_lane = 0;
                }
                else if (d > 4 && d < 8) {
                    car_lane = 1;
                }
                else if (d > 8 && d < 12) {
                    car_lane = 2;
                }
                if (car_lane < 0) {
                    continue;
                }
                double v_x = sensor_fusion[i][3];
                double v_y = sensor_fusion[i][4];
                double v_speed = sqrt(v_x * v_x + v_y * v_y);
                double v_car_s = sensor_fusion[i][5];
                
                v_car_s += (double)prev_size * 0.02 * v_speed;
                
                if (car_lane == lane) { // Car is in ego vehicle lane
                    front_car |= v_car_s > car_s && v_car_s - car_s < 30;
                }
                else if (car_lane < lane) { // Car is in left lane of ego vehicle
                    leftlane_car |= car_s - 15 < v_car_s && car_s + 30 > v_car_s;
                }
                else if (car_lane > lane) { // car is in right lane of ego vehicle
                    rightlane_car |= car_s - 15 < v_car_s && car_s + 30 > v_car_s;
                }
            }
            
            // Step 2: Behavior planner
            const double max_speed = 45;
            const double max_acc = .224;
            
            if (front_car) { // There is car ahead of ego vehicle
                if (!leftlane_car && lane > 0) { // left lane present and no car in left lane
                    lane--; // Change lane
                }
                else if (!rightlane_car && lane !=2) { // right lane present and no car in right lane
                    lane++; // Change lane
                }
                else {
                    initial_vel -= max_acc;
                }
            }
            else {
                if (lane != 1) { // Ego vehicle is not in the center lane
                    if ((lane == 0 && !rightlane_car) || (lane == 2 && !leftlane_car)) {
                        lane = 1; // Steer back to center lane
                    }
                }
                if (initial_vel < max_speed) {
                    initial_vel += max_acc;
                }
            }
            
            // Step 3: Trajectory construction
            
            // We have waypoints evenly spaced at 30m by default, extract that
            vector<double> pt_x;
            vector<double> pt_y;
            
            // We will use car's current position as reference point or previous path's end point as reference point
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg_rad (car_yaw);
            
            // If prev path has few elements, use car's current position as reference point
            if (prev_size < 2) {
                
                // Tangents
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);
                
                pt_x.push_back(prev_car_x);
                pt_y.push_back(prev_car_y);
                pt_x.push_back(car_x);
                pt_y.push_back(car_y);
            }
            
            // Prev path's end point as reference point
            else {
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];
                double prev_ref_x = previous_path_x[prev_size - 2];
                double prev_ref_y = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
                
                pt_x.push_back(prev_ref_x);
                pt_y.push_back(prev_ref_y);
                pt_x.push_back(ref_x);
                pt_y.push_back(ref_y);
            }
            vector<double> next_wp0 = Cartesian(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = Cartesian(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = Cartesian(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            pt_x.push_back(next_wp0[0]);
            pt_x.push_back(next_wp1[0]);
            pt_x.push_back(next_wp2[0]);
            
            pt_y.push_back(next_wp0[1]);
            pt_y.push_back(next_wp1[1]);
            pt_y.push_back(next_wp2[1]);
            
            for (int i =0; i < pt_x.size(); i++) {
                double shift_x = pt_x[i] - ref_x;
                double shift_y = pt_y[i] - ref_y;
                
                pt_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                pt_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
            
            // Spline
            tk::spline spln;
            
            spln.set_points (pt_x, pt_y);
            
            // Define points for planner use
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            // Start with prev path points
            for (int i = 0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            //Spline split
            double target_x = 30.0;
            double target_y = spln(target_x);
            double target_dist = sqrt(target_x * target_x + target_y * target_y);
            double x_add = 0;
            
            for (int i = 1; i < 50 - previous_path_x.size(); i++) {
                double n = target_dist / (0.02 * initial_vel / 2.24);
                double x_pt = x_add + target_x / n;
                double y_pt = spln(x_pt);
                x_add = x_pt;
                double x_ref = x_pt;
                double y_ref = y_pt;
                
                // Back to global coordinates
                x_pt = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                y_pt = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                x_pt += ref_x;
                y_pt += ref_y;

                next_x_vals.push_back(x_pt);
                next_y_vals.push_back(y_pt);
            }
            // End of core code //
            
            
            
            
          json msgJson;

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage
    
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
    });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
