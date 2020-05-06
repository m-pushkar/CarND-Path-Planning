#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <math.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "collisionavoid.h"
#include "coordinates_transformation.h"
#include "polynomialsolver.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;
using namespace Eigen;

typedef struct Vehicle {
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double speed;;
} Vehicle;

typedef struct Planner {
    double target_d;
    vector<Vehicle> obstacles;
    Vehicle target_follow;
    double target_dist;
    int follow_id;
    MatrixXd s_trajectories;
    VectorXd s_costs;
    MatrixXd d_trajectories;
    VectorXd d_costs;
    bool obstacle_path;
    bool feasible_traj;
    int optimal_s;
    int optimal_d;
    double min_cost;
    int iters;
} Planner;

int mylane(double d0) {                                 // mylane - function, my_lane - object
    int my_lane = 1;
    if(d0 > 0 && d0 <= 4) {
        my_lane = 0;
    }
    else if(d0 > 4 && d0 <= 8) {
        my_lane = 1;
    }
    else {
        my_lane = 2;
    }
    return my_lane;
}

// Check availability of SocketIO having JSON data, if not available empty string "" will be returned
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

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
    
    VectorXd optimal_s_coeff(6);
    VectorXd optimal_d_coeff(6);
    double s_cost = 999;
    double d_cost = 999;
    int step = 0;
    int min_max_s = 0;
    
    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                &map_waypoints_dx,&map_waypoints_dy,&optimal_s_coeff,&optimal_d_coeff,&step,&max_s,&min_max_s]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
            
            double pos_x;
            double pos_y;
            double pos_yaw;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
            
            double mph_mps = 1.0/2.237;
            double max_speed = 45 * mph_mps;
            int prev_path_size = previous_path_x.size();
            int max_s_waypt = map_waypoints_s[map_waypoints_s.size() - 1];
            
            step +=1;
            
            cout << "------------------------------------------------------" << endl;
            cout << " [-] Step: " << step << endl;
            cout << " [-] Maxium speed: " << max_s << endl;
            cout << " [-] Frenet coordinates s: , d: " << car_s << "," << car_d << endl;
            cout << " [-] Speed: " << car_speed * mph_mps << "m/sec" << endl;
            
            // Waypoints smoothing
            int last_map_id = map_waypoints_x.size() - 1;
            int nearest_waypt_id = NearestWaypt(car_x, car_y, map_waypoints_x, map_waypoints_y);
            int interpolate_start_id = nearest_waypt_id - 5;
            int interpolate_end_id = nearest_waypt_id + 8;
            
            vector<double> map_x_interpolate;
            vector<double> map_y_interpolate;
            vector<double> map_s_interpolate;
            double s_map;                                                      // _map_s = s_map
            double x_map;
            double y_map;
            
            for (int map_id = interpolate_start_id; map_id < interpolate_end_id; ++map_id) {
                if (map_id > last_map_id ) {
                    int new_map_id = map_id - last_map_id - 1;
                    s_map = map_waypoints_s[new_map_id] + max_s;
                    x_map = map_waypoints_x[new_map_id];
                    y_map = map_waypoints_y[new_map_id];
                }
                else if (map_id < 0) {
                    int new_map_id = last_map_id + map_id + 1;
                    s_map = map_waypoints_s[new_map_id] - max_s;
                    x_map = map_waypoints_x[new_map_id];
                    y_map = map_waypoints_y[new_map_id];
                }
                else {
                    s_map = map_waypoints_s[map_id];
                    x_map = map_waypoints_x[map_id];
                    y_map = map_waypoints_y[map_id];
                }
                map_s_interpolate.push_back(s_map);
                map_x_interpolate.push_back(x_map);
                map_y_interpolate.push_back(y_map);
            }
            
            tk::spline x_s;
            tk::spline y_s;
            x_s.set_points(map_s_interpolate, map_x_interpolate);
            y_s.set_points(map_s_interpolate, map_y_interpolate);
            
            vector<double> map_ss;
            vector<double> map_xs;
            vector<double> map_ys;
            double new_s = map_s_interpolate[0];
            
            while (new_s < map_s_interpolate[map_s_interpolate.size() - 1]) {
                double new_x = x_s(new_s);
                double new_y = y_s(new_s);
                map_ss.push_back(new_s);
                map_xs.push_back(new_x);
                map_ys.push_back(new_y);
                new_s += 0.1;
            }
            
            // Initialize Planner
            vector<Planner> planners;
            for (int i = 0; i < 3; ++i) {
                double new_target_d = 2.0 + 4 * i - 0.15;
                Planner planner;
                MatrixXd s_trajectories (6,0);
                VectorXd s_costs (0);                                                           // s_costs or s_cost??
                MatrixXd d_trajectories (6,0);
                VectorXd d_costs (0);                                                          // d_costs or d_cost??
                
                planner.s_trajectories = s_trajectories;
                planner.s_costs = s_costs;
                planner.d_trajectories = d_trajectories;
                planner.d_costs = d_costs;
                planner.target_d = new_target_d;
                planner.target_dist = 999.9;
                planner.obstacle_path = false;
                planner.feasible_traj = true;
                planner.min_cost = 9999999.9;
                planner.optimal_s = 0;
                planner.optimal_d = 0;
                planner.iters = -1;
                planners.push_back(planner);
            }
            
            // Vehicles nearby
            vector<Vehicle> vehicle_nearby;
            for (int i = 0; i < sensor_fusion.size(); ++i) {
                // Parsing sensor fusion data
                double s_other = sensor_fusion[i][5];
                double s_dist = s_other - car_s;
                double d_other = sensor_fusion[i][6];
                
                // Vehicles nearby
                double detect_range_front = 70.0;
                double detect_range_backward = 20.0;
                
                if ((s_dist < detect_range_front) && (s_dist >= -detect_range_backward) && (d_other > 0)) {
                    Vehicle vehicle;
                    vehicle.id = sensor_fusion[i][0];
                    vehicle.x = sensor_fusion[i][1];
                    vehicle.y = sensor_fusion[i][2];
                    vehicle.vx = sensor_fusion[i][3];
                    vehicle.vy = sensor_fusion[i][4];
                    vehicle.s = sensor_fusion[i][5];
                    vehicle.d = sensor_fusion[i][6];
                    vehicle.speed = sqrt(vehicle.vx * vehicle.vx + vehicle.vy * vehicle.vy);
                    
                    vehicle_nearby.push_back(vehicle);
                }
            }
            
            int planning_horizon = 150;
            int pass = planning_horizon - prev_path_size;
            int start_index = pass - 1;
            double start_time = start_index * 0.02;
            double s0 = position(optimal_s_coeff, start_time);
        
            if (s0 > max_s) {
                s0 = s0 - max_s;
            }
            
            double s0dot = velocity(optimal_s_coeff, start_time);
            double s02dot = acceleration(optimal_s_coeff, start_time);
            double d0 = position(optimal_d_coeff, start_time);
            double d0dot = velocity(optimal_d_coeff, start_time);
            double d02dot = acceleration(optimal_d_coeff, start_time);
            int my_lane = mylane(car_d);
            
            // Nearest vehicle for each lane
            for (int i = 0; i < vehicle_nearby.size(); ++i) {
                Vehicle new_vehicle = vehicle_nearby[i];
                for (int j = 0; j < planners.size(); ++j) {
                    if ((new_vehicle.d > planners[j].target_d - 2) && (new_vehicle.d <= planners[j].target_d + 2)) {
                        double ego_other = new_vehicle.s - car_s;
                        if (j == my_lane) {
                            if (ego_other >= 3) {
                                planners[j].obstacles.push_back(new_vehicle);
                            }
                        }
                        else {
                            planners[j].obstacles.push_back(new_vehicle);
                        }
                        if (ego_other >= -1.0) {
                            if (ego_other < planners[j].target_dist) {
                                planners[j].target_dist = ego_other;
                                planners[j].target_follow = new_vehicle;
                            }
                            if (ego_other <= 65) {
                                planners[j].obstacle_path = true;
                            }
                        }
                    }
                }
            }
            
            int emp;
            
            if (prev_path_size == 0) {
                double target_s1dot = 20 * mph_mps;
                bool in_lane = true;
                
                emp = velocitykeeptrajectories(car_s, car_speed, 0, target_s1dot, max_speed, planners[1].s_trajectories, planners[1].s_costs);
                emp = lateraltrajectories(car_d, 0, 0, 6.0, in_lane, planners[1].d_trajectories, planners[1].d_costs);
                
                vector<int> opt_idx = optimalcombo(planners[1].s_costs, planners[1].d_costs);
                optimal_s_coeff = planners[1].s_trajectories.col(opt_idx[0]);
                optimal_d_coeff = planners[1].d_trajectories.col(opt_idx[1]);
                next_x_vals.push_back(car_x);
                next_y_vals.push_back(car_y);
                
                for (int z = 0; z < planning_horizon; z++) {
                    double s = position(optimal_s_coeff, z * 0.02);
                    double d = position(optimal_d_coeff, z * 0.02);
                    
                    vector<double> xy = Cartesian(s, d, map_ss, map_xs, map_ys);
                    next_x_vals.push_back(xy[0]);
                    next_y_vals.push_back(xy[1]);
                }
            }
            else {
                
                // Generate trajectory and costs
                double target_s1dot = (car_speed) * mph_mps;
                if (target_s1dot > max_speed) {
                    target_s1dot = max_speed;
                }
                for (int i = 0; i < 3; ++i) {
                    
                    // Longitudinal trajectory
                    if (i == my_lane) {
                        bool in_lane = true;
                        if (planners[i].obstacle_path) {
                            Vehicle target_obstacle = planners[i].target_follow;
                            
                            emp = followtrajectories(s0, s0dot, s02dot, target_obstacle.s ,target_obstacle.speed - 0.2, max_speed,            planners[i].s_trajectories, planners[i].s_costs);
                            
                            emp = velocitykeeptrajectories(s0, s0dot, s02dot, target_obstacle.speed - 10.0, max_speed, planners[i].s_trajectories, planners[i].s_costs);
                        }
                        else {
                            emp = velocitykeeptrajectories(s0, s0dot, s02dot, target_s1dot, max_speed, planners[i].s_trajectories, planners[i].s_costs);
                        }
                        // Stop velocity keep
                        emp = velocitykeeptrajectories(s0, s0dot, s02dot, 0.0, max_speed, planners[i].s_trajectories, planners[i].s_costs);
                        
                        // Lateral trajectory
                        emp = lateraltrajectories(d0, d0dot, d02dot, planners[i].target_d, in_lane, planners[i].d_trajectories, planners[i].d_costs);
                    }
                    else if ((abs(i - my_lane) <= 1) && (car_speed * mph_mps > 9.0)) {
                        bool in_lane = false;
                        if (planners[i].obstacle_path) {
                            Vehicle target_obstacle = planners[i].target_follow;
                            
                            emp = followtrajectories(s0, s0dot, s02dot, target_obstacle.s ,target_obstacle.speed - 0.2, max_speed,            planners[i].s_trajectories, planners[i].s_costs);
                            
                            emp = velocitykeeptrajectories(s0, s0dot, s02dot, target_s1dot - 10.0, max_speed, planners[i].s_trajectories, planners[i].s_costs);
                        }
                        else {
                            // Keep
                            emp = velocitykeeptrajectories(s0, s0dot, s02dot, target_s1dot, max_speed, planners[i].s_trajectories, planners[i].s_costs);
                        }
                        
                        // Stop velocity keep
                        emp = velocitykeeptrajectories(s0, s0dot, s02dot, 0.0, max_speed, planners[i].s_trajectories, planners[i].s_costs);
                        
                        // Lateral trajectory
                        emp = lateraltrajectories(d0, d0dot, d02dot, planners[i].target_d, in_lane, planners[i].d_trajectories, planners[i].d_costs);
                        
                    }
                    else {
                        planners[i].feasible_traj = false;
                        cout << " [*] Feasible trajectory does not exist" << endl;
                    }
                    
                    // Optimal trajectory
                    double k_long = 1.0;
                    double k_lat = 1.8;
                    int ns = planners[i].s_costs.size();
                    int nd = planners[i].d_costs.size();
                    int ntraj = ns * nd;
                    
                    // Cost matrix
                    if (ntraj == 0) {
                        planners[i].feasible_traj = false;
                        cout << " [*] Feasible trajectory does not exist" << endl;
                    }
                    else {
                        MatrixXd sd_costs(ns, nd);
                        for (int ss = 0; ss < ns; ++ss) {
                            for (int dd = 0; dd < nd; ++dd) {
                                sd_costs(ss,dd) = k_long * planners[i].s_costs[ss] + k_lat * planners[i].d_costs[dd];
                            }
                        }
                        // cout << " [-] sd_costs size: ("<< ns << "," << nd <")" << endl;
                        
                        // Check for collision
                        int max_iters = 200;
                        int iters = -1;
                        if (max_iters >= ntraj) {
                            max_iters = ntraj;
                        }
                        for (int k = 0; k < max_iters; ++k) {
                            bool crash_predict = false;
                            int min_s_idx;
                            int min_d_idx;
                            double minimum_cost = sd_costs.minCoeff(&min_s_idx, &min_d_idx);
                            optimal_s_coeff = planners[i].s_trajectories.col(min_s_idx);
                            optimal_d_coeff = planners[i].d_trajectories.col(min_d_idx);
                            
                            for (int l = 0; l < planning_horizon; ++l) {
                                
                                // Ego vehicle position
                                double new_s = position(optimal_s_coeff, l * 0.02);
                                double new_d = position(optimal_d_coeff, l * 0.02);
                                
                                // Other vehicle position
                                for (int m = 0; m < planners[i].obstacles.size(); ++m) {
                                    Vehicle other_vehicle = planners[i].obstacles[m];
                                    double new_s_other = other_vehicle.s + l * 0.02 * (other_vehicle.speed - 0.1);
                                    double new_d_other = other_vehicle.d;
                                    int crash = CollisionAvoid(new_s, new_d, 0, new_s_other, new_d_other, 0.0);
                                    if (crash == 1) {
                                        crash_predict = true;
                                        break;
                                    }
                                }
                                if (crash_predict) {
                                    sd_costs(min_s_idx, min_d_idx) = 9999999.9;
                                    break;
                                }
                            }
                            iters = k;
                            if (!crash_predict) {
                                planners[i].optimal_s = min_s_idx;
                                planners[i].optimal_d = min_d_idx;
                                planners[i].min_cost = sd_costs(min_s_idx, min_d_idx);
                                break;
                            }
                        }
                        planners[i].iters = iters;
                        if (iters == max_iters - 1) {
                            planners[i].feasible_traj = false;
                            cout << " [*] Feasible trajectory does not exist" << endl;
                        }
                    }
                }
                
                // Optimal lane
                double min_cost = 999999.9;
                int opt = 1;
                for (int i = 0; i < 3; ++i) {
                    if (planners[i].feasible_traj) {
                        if (planners[i].min_cost <= min_cost) {
                            opt = i;
                            min_cost = planners[i].min_cost;
                        }
                    }
                }
                for (int i = 0; i < 2; ++i) {
                    if ((i != opt) && (abs(planners[i].min_cost - min_cost) < 0.1)) {
                        if (planners[i].target_dist > planners[opt].target_dist) {
                            opt = i;
                        }
                    }
                }
                
                cout << " [-] Vehicle nearby: " << vehicle_nearby.size() << endl;
                cout << " [-] My lane: " << my_lane << endl;
                cout << " [-] Obstacles: " << planners[0].obstacles.size() << "," << planners[1].obstacles.size() << "," << planners[2].obstacles.size() << endl;
                cout << " [-] Feasible: " << planners[0].feasible_traj << "," << planners[1].feasible_traj << "," << planners[2].feasible_traj << endl;
                cout << " [-] Follow: " << planners[0].obstacle_path << "," << planners[1].obstacle_path << "," << planners[2].obstacle_path << endl;
                cout << " [-] Cost: " << planners[0].min_cost << "," << planners[1].min_cost << "," << planners[2].min_cost << endl;
                cout << " [-] Iters: " << planners[0].iters << "," << planners[1].iters << "," << planners[2].iters << endl;
                cout << " [-] Optimal lane: " << opt << endl;
                
                int opt_s = planners[opt].optimal_s;
                int opt_d = planners[opt].optimal_d;
                optimal_s_coeff = planners[opt].s_trajectories.col(opt_s);
                optimal_d_coeff = planners[opt].d_trajectories.col(opt_d);
                
                // Run
                
                for (int z = 0; z < planning_horizon + 1; ++z) {
                    double s = position(optimal_s_coeff, z * 0.02);
                    double d = position(optimal_d_coeff, z * 0.02);
                    
                    // Loop
                    if (s > map_s_interpolate[map_s_interpolate.size() - 1]) {
                        s = s - max_s;
                    }
                    else if (s < map_s_interpolate[0]) {
                        s = s + max_s;
                    }
                    
                    vector<double> xy = Cartesian(s, d, map_ss, map_xs, map_ys);
                    next_x_vals.push_back(xy[0]);
                    next_y_vals.push_back(xy[1]);
                }
            }
            
            if (min_max_s == 2) {
                return 0;
            }
            


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
    
    // Not needed to run the code but for compilation

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
