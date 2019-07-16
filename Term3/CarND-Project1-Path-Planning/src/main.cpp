/*
main.cpp :
Version 0:  Base code from udacity

Version 1:  Make car drive in straight line, based on udacity code (car over shoots road)

Version 2:  Make car drive in circular path, based on udacity code

Version 3:  Make car drive in straight line along 's'

Version 4:  Make car drive in straight line along 's' using speed equations(no acceleration), car path explodes

Version 5:  Attempt at lane keeping (drive in straight line along s) using speed and acceleration limits.
            Car maintains lane However car still exceeds speed, accel and jerk limits several times
            Car Collision not handled

Version 6: Attempt at lane keeping (drive in straight line along s) using spline. Based on Udacity's Aaron's Code
Logic:
STEP 1: CREATE 5 POINTS SPACED FAR APART : GENERATE ptsx & ptsy: so this will be
(ptsx &,ptsy)= 5 points = {(ref_x,ref_y), (ref_prev_x,ref_prev_y), getXY(ego_s_30),getXY(ego_s_60),getXY(ego_s_90)}

STEP 2: CREATE SPLINE : FROM THE FAR SPACED 5 POINTS
spl = spline-from(ptsx,ptsy)

STEP 3: CREATE THE ACTUAL PATH POINTS: next_x_vals, next_y_vals for the Path Planner
STEP 3a: USE previous_path POINTS TO CREATE :next_x_vals, next_y_vals (DOES NOT USE THE SPLINE POINTS-spl)
---> This step does not require the use of the newly generated spline, we just use the previous_path_x & previous_path_y.
---> However spline could have been used to generate previous_path_x & previous_path_y (in the previous cycle of course)
---> Note: previous_path_x, previous_path_y  DO-NOT INCLUDE ALL  the points generated the previous cycle.
     They only consist of the remainder of the path points that the car did not use/did not travel in the previous cycle

STEP 3b: USE THE SPLINE 'spl' TO SAMPLE : finer next_x_vals, next_y_val
Ideally the distance we'd like to move every time step to maintain speed limit is per_time_step_dist  =  0.02*REF_VEL/2.24
But we need a way to map this 'per_time_step_dist ' to x & y co-ordinates on a curved road.
We achieve this using the spline points and 2-similar-triangles. See the write up that accompanies the code for more details

Resolved Issues:
Car maintians speed, accel and jerk limits (exceeds accel and jerk only during cold start i.e)

Issues to address:
i)   Handle accel and jerk exceeding during cold start
ii)  Handle collisions
iii) Handle Lane Changes
*/


#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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
          cout<<"\n   ego_d     ="<<ego_d     <<"; egoC_lane ="   << egoC_lane ;

          //start in lane 1
          int lane = 1;

          //have a reference velocity to target
          double REF_VEL = 49.5 ; //mph



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

          /*
          if(prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;

          //find ref_v to use
          for(int i =0; i< sensor_fusion.size(); i++) {

              //car is in my lane
              float d = sensor_fusion[i][6];
              if(d < (2+4*lane+2) && d >(2+4*lane-2)) {
                  double vx          = sensor_fusion[i][3];
                  double vy          = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx + vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  check_car_s+=((double)prev_size*0.02*check_speed); //if using previous points cann project s value out
                  //check s values greater than mine and s gap
                  if((check_car_s > car_s) && ((check_car_s - car_s)<30)) {

                    //Do some logic here, lower reference velocity so that we dont crash into the car in front of is
                    //also flag to try to change lanes
                    REF_VEL = 29.5; //30mph
                    //too_close = true ;
                  }

              }
          }
          */

          /*
          if(too_close) {
            REF_VEL -= 0.224;
          }
          else if(REF_VEL < 49.5){
            REF_VEL += 0.224;
          }
          */



          // --------------------------------------------------------------------------------------------------------------------------------------------
          // ----------------------------- STEP 1: CREATE 5 POINTS SPACED FAR APART : GENERATE ptsx & ptsy ----------------------------------------------
          // --------- (ptsx &,ptsy)= 5 points = {(ref_x,ref_y), (ref_prev_x,ref_prev_y), getXY(ego_s_30),getXY(ego_s_60),getXY(ego_s_90) ---------------
          // --------------------------------------------------------------------------------------------------------------------------------------------
          // Create a list of widely space (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points that control spacing
          vector <double> ptsx;
          vector <double> ptsy;

          // reference x,y, yaw states . The reference state is one of the two
          // i) either the current car state
          // ii) or the last point of the previous_path. but how do we know the last point of the previous path ??
          // we only get a list of the remainder of the points
          double ref_x     , ref_y      , ref_yaw;
          double ref_x_prev, ref_y_prev ;

          //if previous size is almost empty, use the car as starting reference
          if(prev_size < 2) {
            //Use two points that make the path tangent to the car
            //ksw comment:
            //i) one is the car's position ego_x,ego_y,ego_yaw
            //ii) the other point , use the ego car's yaw to go back one step.
            //But how do you know how far to go back along yaw ??? could you use the car's speed, or is that really unnecessary
            ref_x   = ego_x;
            ref_y   = ego_y;
            ref_yaw = ego_yaw; //in radian, important to note that while udacity uses ref_yaw in degrees, I use it in radian.

            ref_x_prev = ref_x - cos(ref_yaw);
            ref_y_prev = ref_y - sin(ref_yaw);

          }
          //use the previous path's en d point as starting reference
          else {
            //Use two points that make the path tangent to the previous path's end point
            //ksw comment : again you will use only two points, LOL !!! much ado about nothing
            //redefine reference state as previous path end point
            ref_x      = previous_path_x[prev_size -1];
            ref_y      = previous_path_y[prev_size -1];

            ref_x_prev = previous_path_x[prev_size-2];
            ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev) ; //in radian

          }

          //In Frenet add evenly 30m spaced points ahead of the starting reference look out for 30m, 60m, 90m)
          vector<double> next_wp0 = getXY(ego_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(ego_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(ego_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // So there are 5 waypoints
          //  i) previous 2 locations of the cars
          // ii) location of the car in 30 meters,  60 meters and 90 meters
          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);


          // Shift ptsx, ptsy  to the car's local co-ordinates i.e. with the car at zero. This transformation is a shift and rotation.
          // We did this in MPC. It makes the math much easier (will come into play later)
          // The last point of the previous path is at (0,0) origin, angle is also at zero degrees.
          // Note : The first  point on the list (ref_x_prev, ref_y_prev) is not at (0,0)
          //        The second point on the list (ref_x,      ref_y,    ref_yaw) is moved to (0,0,0) : This is where the car is
          for (int i =0; i < ptsx.size(); i++) {
            //shift ego_car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            //Rotation
            ptsx[i] = (shift_x *cos(0-ref_yaw) -shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw) +shift_y*cos(0-ref_yaw));

          }




          // ----------------------------------------------------------------------------------------------------------------------------------------
          // ------------------------------- STEP 2: CREATE SPLINE : FROM THE FAR SPACED 5 POINTS ---------------------------------------------------
          // ------------------------------- spl = spline-from(ptsx,ptsy) ---------------------------------------------------------------------------
          // ----------------------------------------------------------------------------------------------------------------------------------------
          //ksw comments: define a spline spl
          tk::spline spl;

          //ksw comments: set the anchor points ptsx and ptsy (from above) to the spline
          //ptsx, ptsy are the 5 points from above whichc are also transformed to car's local co-ordinates (whit car at zero)
          spl.set_points(ptsx, ptsy);




          // ---------------------------------------------------------------------------------------------------------------------------------------
          // ------ STEP 3: CREATE THE ACTUAL PATH POINTS: next_x_vals, next_y_vals for the Path Planner -------------------------------------------
          // ------ STEP 3a: USE previous_path POINTS TO CREATE :next_x_vals, next_y_vals (DOES NOT USE THE SPLINE POINTS-spl) ---------------------
          // -------Note: previous_path_x, previous_path_y  DO-NOT INCLUDE ALL the points generated the previous cycle.-----------------------------
          // -------They only consist of the remainder of the path points that the car did not use/did not travel in the previous cycle-------------
          // ---------------------------------------------------------------------------------------------------------------------------------------

          //Define the actual (x,y) points we will use for the Planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Start with all of the previous path points from last time and push those
          for(int i =0; i <previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // ----------------------------------------------------------------------------------------------------------------------------------------
          // ------ STEP 3b: USE THE SPLINE 'spl' TO SAMPLE : finer next_x_vals, next_y_val (USES SPLINE POINTS-spl) --------------------------------
          // ----------------------------------------------------------------------------------------------------------------------------------------

          // ksw comments:  see write up that accompanies code & explanation below
          // look at Project Q&A video at ~33min: in the project video 'd' is not the Frenet 'd', it is the 'distance'(hypotenuse calc as : dist= sqrt(x*x+y*y)
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          // sample_x: is the distance how far into the horizon you want to approximate the spline to
          // Just because you choose sample_x = 30m doesn't mean that the car will actually cover 30 meters along x in 50 time steps
          // In one cycle the car could travel more than 50 time steps or less. This sample_x is just for approximation.
          // Hence choose a value of x that approximates the distance travelled in 50 time steps i.e 50 x 0.02 x 49.5 / 2.24 = 22.098 meter.
          // You can even choose sample_x = 0.5m, 1m, 60m, 90m . This will just change the spline point y (sample_y) .
          // By change I mean that arc/polynomial : sample_y @0.5m could be significantly different from the spline sample_y @ 60m
          // And hence  dimensions of the similar triangle you are trying to approximate could be significantly different
          // sample_dist: the path between two consecutive x,y points is an arc(characterised by a spline/polynomial)
          //              However this is just approximated as a straight line (hypotenuse) to make life easier
          //So sample_x, sample_y, sample_dist form the base, height and hypotenuse respectively  of the Right-Angled-Triangle-1(generated using spline)
          double sample_x        = 30.0 ;
          double sample_y        = spl(sample_x) ;
          double sample_dist     = sqrt((sample_x*sample_x)+(sample_y*sample_y));

          // We want the car to move at 49.5 mph(49.5/2.24 m/s). so the per_time_step_dist covered needs to be 0.02*REF_VEL/2.24 meter
          // This distance is an arc along the spline. However similar to sample_dist, we approximate per_time_step_dist as a straight line instead of arc
          // Hence per_time_step_x, per_time_step_y, per_time_step_dist form the base, height and hypotenuse respectively of the Right-Angled-Triangle-2
          // Right-Angled-Triangle-1 & Right-Angled-Triangle-2 are 'similar'. This is used to establish unknowns per_time_step_x
          // per_time_step_y is not estimated from right_triangle but use spline directly (in loop)
          double per_time_step_dist  =  0.02*REF_VEL/2.24 ;
          double per_time_step_x     = sample_x*per_time_step_dist/sample_dist    ;



          double x_point_car_coord, y_point_car_coord;
          double x_point_map_coord, y_point_map_coord;
          //Fill up the rest of our path planner after filling it with previous points,
          //Here we ill always output 50 points
          for(int i =1; i<=50-previous_path_x.size(); i++) {
              x_point_car_coord = per_time_step_x*i ;
              y_point_car_coord = spl(x_point_car_coord) ;

              //Rotate back to map-coordinates(We had earlier rotated everything to car-ordinates)
              x_point_map_coord = (x_point_car_coord *cos(ref_yaw) - y_point_car_coord*sin(ref_yaw));
              y_point_map_coord = (x_point_car_coord *sin(ref_yaw) + y_point_car_coord*cos(ref_yaw));

              //Shift back to map-coordinates(We had earlier shifted everything to car-ordinates)
              x_point_map_coord += ref_x;
              y_point_map_coord += ref_y;

              next_x_vals.push_back(x_point_map_coord);
              next_y_vals.push_back(y_point_map_coord);

          }


          json msgJson;
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
