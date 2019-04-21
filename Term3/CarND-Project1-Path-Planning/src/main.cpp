#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double ego_x     = j[1]["x"];
          double ego_y     = j[1]["y"];
          double ego_s     = j[1]["s"];
          double ego_d     = j[1]["d"];
          double ego_yaw   = j[1]["yaw"];
          double ego_speed_mph = j[1]["speed"];
          int    egoC_lane  = ego_d/4 ;
          cout<<"\n\n ego_speed_mph ="<<ego_speed_mph <<"; ego_s ="     << ego_s ;
          cout<<"\n   ego_d     ="<<ego_d     <<"; egoC_lane =" << egoC_lane ;



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
          vector<double> next_point ;
          double egoC_next_s    = ego_s;
          double egoC_speed_mps = ego_speed_mph*0.44704;
          double max_accel = 9.5 ; //m/s^2
          double time_step = 0.02 ; //0.02 seconds = 20 milli seconds

          //Attempt 3: Car drives in straight line along s
          double dist_inc = 0.446;
          cout<<"\n Begin for loop---------------------------------------";
          for (int i = 0; i < 50; ++i) {

               cout<<"\n ego_speed_mph ="<<ego_speed_mph <<"; ego_s =" << ego_s << "; egoC_next_s =" << egoC_next_s ;
               //50mph = 22.352 m/s
               /*
               ----------------------------------------------------------------------------------------
                          Attempt at Lane Keeping along 's' using speed and acceleration limits
               ----------------------------------------------------------------------------------------

               Performance Comments:
               i)    he car is able to maintain the lane and drive at center of Lane (that is the good news)
               ii)  However ego car's speed hovers between 17-30mph. I think the time to reach 50mpph is too slow
                    (so might need to use a different approach rather than using these motion equations)
               iii) Occasionally at turns where the way points are clustered or something, the car exceeds speed limit to 100mph
               iv)  Also the max acceleration and jerk get exceeded
               v)   No mechanism for collision avoidance has benn included yet (so car will collide , but that is okay)

               Verdict/Goal/TODO :
               i)  Achieve lane keeping, without violating speed, acceleration and jerk limits
               ii) Use eiher polynomial lane generation or spline to be able to do this
               */

               if(egoC_speed_mps < 22) //increase ego_speed_mph, egoC_speed_mps
                { egoC_next_s    = egoC_next_s    + (egoC_speed_mps*time_step) + (0.5*max_accel*time_step*time_step);
                  egoC_speed_mps = egoC_speed_mps + (max_accel*time_step);
                  if(egoC_speed_mps> 22)
                     egoC_speed_mps = 22;
                }
               else if (22.352 < egoC_speed_mps) //decrease ego_speed_mph, egoC_speed_mps
               { egoC_next_s    = egoC_next_s    + (egoC_speed_mps*time_step) - (0.5*max_accel*time_step*time_step);
                 egoC_speed_mps = egoC_speed_mps - (max_accel*time_step);
                 if(egoC_speed_mps < 22)
                    egoC_speed_mps = 22;
               }
               else if ((22< egoC_speed_mps) && (egoC_speed_mps<22.352)) //maintain speed
               { egoC_next_s    = egoC_next_s    + (egoC_speed_mps*time_step) ;
                 egoC_speed_mps = egoC_speed_mps ;
               }

               next_point = getXY(egoC_next_s,ego_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
               next_x_vals.push_back(next_point[0]);
               next_y_vals.push_back(next_point[1]);
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
