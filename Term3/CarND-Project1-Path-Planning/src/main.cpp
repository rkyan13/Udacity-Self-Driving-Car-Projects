/*
Note:
---> in Frenet co-ordinates d is lateral distance, s is vertical distance along the length of the road

---> d-value for lanes {0,
                       LeftLane:   Lane#0: (1,2,3),
                       4,
                       MiddleLane: Lane#1: (5,6,7),
                       8,
                       RightLane: Lane#2:  (9,10,11),
                       12}

---> previous_path_x, previous_path_y  DO-NOT INCLUDE ALL  the points generated the previous cycle.
    They only consist of the remainder of the path points that the car did not use/did not travel in the previous cycle

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

main.cpp :
Attempt 0:  Base code from udacity
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 1:  Make car drive in straight line, based on udacity code (car over shoots road)
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 2:  Make car drive in circular path, based on udacity code
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 3:  Make car drive in straight line along 's'
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 4:  Make car drive in straight line along 's' using speed equations(no acceleration), car path explodes
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 5:  Attempt at lane keeping (drive in straight line along s) using speed and acceleration limits.
            Car maintains lane However car still exceeds speed, accel and jerk limits several times
            Car Collision not handled
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 6: Attempt at lane keeping (drive in straight line along s) using spline. Based on Udacity's Aaron's Code
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
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 7a: Attempt at Handling Collisions
Resolved Issues:
Ego car does not collide: slows down to 29.5 mph if there is a car in front
(but ego-car reference velocity keeps changing from 29.5 to 49.5 mph every cycle. Quite irritating !!!)

Issues to address:
i)  Handle collisions in a better way
--> ego-car velocity keeps changing from 29.5 to 49.5 mph every cycle
--> ego-car slows down to 29.5 mph but never picks up the speed back to 49.5 mph
ii)   Handle accel and jerk exceeding during cold start
iii) Handle Lane Changes
-------------------------------------------------------------------------------

Attempt 7b: Attempt at Handling Collisions
---> Resolved the velocity changing from 29.5mph to 49.5 mph every cycle.Now ego-car-velocity remains at 29.5mph every cycle

--> Moved the following piece of code from inside int main()/h.onMessage(...) to outside int main()
//start in ego_lane 1
int ego_lane = 1;

//have a reference velocity to target
double REF_VEL = 49.5 ; //mph

-------------------------------------------------------------------------------

Attempt 7c: Attempt at Handling CollisionsVersion 7b: Attempt at Handling Collisions
Car slows down to the



3..333*speed-of-car-in-front instead of a standard (29.5mph)
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 8a: Collision Avoidance & Cold Start : Slow-Down, Speed up Gradually (EVERY CYCLE, OUTSIDE PATH PLANNER)
Resolved Issues:
i) Collision Handling: Smooth Accelerartion/Deceleration & Jerk Minimization
---> Ego car does not collide
---> Ego car slows down GRADUALLY @REF_VEL -= 0.224(deceleration 5m/s^2) every cycle(outside path planner)
---> Once the slow moving car has passed the ego-car gradually ramps up to IDEAL_SPEED_LIMIT @REF_VEL -= 0.224(acceleration 5m/s^2) every cycle(outside path planner)
ii) Cold Start Jerk Mimization
---> Car will start at 0 and gradually ramp up to IDEAL_SPEED_LIMIT @REF_VEL -= 0.224(acceleration 5m/s^2) every cycle (outside path planner)

Issues to address:
i)  Increment/Decrement the ego-speed(i.e the REF_VEL) every way-point,inside the path-planner (instead of doing it every cycle outside the path planner)
ii) Handle lane Changes

-------------------------------------------------------------------------------

Attempt 8b: Collision Avoidance & Cold Start : Slow-Down, Speed up Gradually (EVERY WAYPOINT, INSIDE PATH PLANNER)
Resolved Issues:
i)  Increment/Decrement the ego-speed(i.e the REF_VEL) every way-point,inside the path-planner loop (instead of doing it every cycle outside the path planner)
But Increment/Decrement-ing REF_VEL @0.224 every way-point causes jerk issues + collisions.
Hence ---> I decrement REF_VEL at a slower rate of 0.224/2.0, &
      ---> I increment REF_VEL at a slower rate of 0.224/3.0

Issues to address:
i) Handle lane Changes
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 9: First take on Lane change (See project Q&A 52.30min to 57min )
i) If there is is a slow moving car ahead, ego_car will move to the left lane (if left lane exists)

Issues to address:
i) Does not check if it is safe to change lanes.i.e does not check if car on left lane is within colliding distance, just blindly changes lane
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 10a: First Rubric Passing Version with Good Lane Changes !!!!
i) If there is is a car ahead, ego_car will change lanes to as appropriate left or right lane  whichever is clear of traffic
(or continue in the same lane, if neither left nor right lane is clear)

Issues to address:
i) Lane changes are sloppy. Often car undecided on whether to change lanes or not
ii) Fine tuning of some parameters required

Other possible improvments
i) If Ego-Car is in High-Speed-Left-Lane(Lane0) or Low-Speed-Right-Lane(Lane2), it does not make an effort to get back to Center-Lane1
Possibly address this later since this is not required in the rubric
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Attempt 10b: FRubric Passing Version with Cleaner Lane Changes and Better Collision Avoidance

Resolved issues
i) Better Speed Limit Maintenance by ego_car
Changed the following line of code
if((car_ahead == true)
to
if((car_ahead == true) && (REF_VEL > car_ahead_speed*2.24*0.93 )) to ensure that the speed of the ego_car never drops below 93% of the car_ahead.
Without this change the ego_car speed would often drop dangerously low to values as low as 15mph making lane change impossible/dangerous and would also lead to collisions

ii) Smoother Lane changes
      If there is a slower moving car at distance of <30m ahead the ego_car will attempt to change lanes to the left or right.In the
previous version the code was written so that the adjacent car lanes also needed to be clear of cars for 30meters to facilitate lane change.
This caused sloppy lane changes.
      For example assume ego_car travelling in Lane2(the right most lane) encounters a slow moving car ~29 meters ahead.
Also assume there is a slow moving car ~32 metres ahead of ego_car but on the Middle Lane (Lane 1) i.e. to the left of the ego car
The ego_car would move left from Lane-2(right lane) to Lane-1(middle lane) only to realise very quickly the car in Lane-1(Middle lane)
is also slow moving and jump back to Lane-1 . This would cause the ego_car to straddle lanes

      In order to facilitate smoother lane change I added the following parameters
SAFE_DIST_FROM_SAME_LANE_CARS            = 30.0
SAFE_DIST_FROM_ADJACENT_LANE_CARS_AHEAD = 40 & SAFE_DIST_FROM_ADJACENT_LANE_CARS_BEHIND =17 (I spent a fair amount of time fine tuning the values)
This way the ego_car will change lanes only if the adjacent lanes are clear of traffic 40m ahead (i.e more than the same_lane clearance of 30m) since there is really
no point in changing lanes and landing in a similar situation
    Also 30m for checking for cars behing the ego_car in adjacent lanes was too much making lane change tooo conservative and frustrating.
So I fine tuned SAFE_DIST_FROM_ADJACENT_LANE_CARS_BEHIND to 17m using trial and experimentation


Other possible improvments
i) If Ego-Car is in High-Speed-Left-Lane(Lane0) or Low-Speed-Right-Lane(Lane2), it does not make an effort to get back to Center-Lane1
Possibly address this later since this is not required in the rubric
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/



#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
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

//start in ego_lane 1
int ego_lane_intended = 1;

//have a reference velocity to target
const  double IDEAL_SPEED_LIMIT = 49.5;//mph
// REF_VEL will be the running reference_velocity of the ego_car based on what other cars are around/ lane change etc.
// start REF_VEL at 0 mph to take care of cold start
double REF_VEL = 0 ; //mph

const int debug = 0;

const double SAFE_DIST_FROM_SAME_LANE_CARS            = 30.0 ; //meter
const double SAFE_DIST_FROM_ADJACENT_LANE_CARS_AHEAD  = 40.0 ; //meter
const double SAFE_DIST_FROM_ADJACENT_LANE_CARS_BEHIND = 17.0 ; //meter


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

  //the h.onMessage is called/occurs every cycle
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message. The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();


        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double ego_x            = j[1]["x"];
          double ego_y            = j[1]["y"];
          double ego_s            = j[1]["s"];
          double ego_d            = j[1]["d"];
          double ego_yaw          = j[1]["yaw"];
          double ego_speed_mph    = j[1]["speed"];
          int    ego_lane_actual  = ego_d/4 ;
          //if d =4 , ego_lane_actual will become 3, but there are only 3 lanes: Lane 0,1 & 2
          if(ego_lane_actual == 3)
                 ego_lane_actual =2 ;

          if(debug == 1) {
                cout<<"\n\n ego_speed_mph ="<<ego_speed_mph <<"; ego_s ="     << ego_s ;
                cout<<"\n   ego_d     ="<<ego_d     <<"; ego_lane_actual ="   << ego_lane_actual     <<"; ego_lane_intended ="   << ego_lane_intended ;
          }


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

          // --------------------------------------------------------------------------------------------------------------------------------------------
          // ---------------- HANDLING COLLISIONS (Project Q & A : 40min - 49min ) ----------------------------------------------------------------------
          // ---------------- HANDLING COLLISIONS -PART 1: Read Sensor Fusion Data of Other Cars --------------------------------------------------------
          // --------------------------------------------------------------------------------------------------------------------------------------------
          if(prev_size > 0) {
            ego_s = end_path_s;
          }

          bool car_ahead = false;
          bool car_left  = false;
          bool car_right = false;

          double car_ahead_distance = 9999.99;
          double car_left_distance  = 9999.99;
          double car_right_distance = 9999.99;

          double car_ahead_speed = 999.99;
          double car_left_speed  = 999.99;
          double car_right_speed = 999.99;

          int car_ahead_num = -99,  car_ahead_lane  = -99;
          int car_left_num  = -99,  car_left_lane   = -99;
          int car_right_num = -99,  car_right_lane  = -99;


          double other_car_vx    ; //m/s
          double other_car_vy    ; //m/s
          double other_car_speed ; //m/s
          float  other_car_d     ; //m
          int    other_car_lane  ;
          double other_car_s, other_car_s_future  ; //m

          //find ref_v to use
          //Use sensor fusion data of all the other surrounding cars
          for(int i =0; i< sensor_fusion.size(); i++) {
              //sensor_fusion[i] has the values from ith car on the road

              other_car_vx    = sensor_fusion[i][3];
              other_car_vy    = sensor_fusion[i][4];
              other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);

              other_car_s     = sensor_fusion[i][5];
              //project other_car_s into the future. i.e if prev_size has 10 remaining values. where will the other car be in 10 time steps
              //KSW Question : previous_size differs every time. and everytime we calculate path for 50 time steps. Shouldn't we be projecting for more than previous_time step size
              other_car_s_future = other_car_s + ((double)prev_size*0.02*other_car_speed); //if using previous points cann project s value out

              other_car_d     = sensor_fusion[i][6];
              other_car_lane  = other_car_d/4;
              //if d =4 , other_car_lane will become 3, but there are only 3 lanes: Lane 0,1 & 2
              if(other_car_lane == 3)
                     other_car_lane =2;

              //check if other_car is in ego_lane
              if(other_car_lane == ego_lane_actual) {
                    if(0<(other_car_s_future - ego_s) &&(other_car_s_future - ego_s)< SAFE_DIST_FROM_SAME_LANE_CARS) {
                          car_ahead          = true ;
                          car_ahead_num      = i+1  ;
                          car_ahead_lane     = other_car_lane ;
                          car_ahead_distance = other_car_s_future - ego_s ;
                          car_ahead_speed    = other_car_speed ;
                    }
              }

              if(other_car_lane == ego_lane_actual-1){
                    if(0<(other_car_s_future - ego_s) && abs(other_car_s_future - ego_s)< SAFE_DIST_FROM_ADJACENT_LANE_CARS_AHEAD){
                          car_left          = true ;
                          car_left_num      = i+1  ;
                          car_left_lane     = other_car_lane ;
                          car_left_distance = other_car_s_future - ego_s ;
                          car_left_speed    = other_car_speed ;
                    }
                    else if(0>(other_car_s_future - ego_s) && abs(other_car_s_future - ego_s)< SAFE_DIST_FROM_ADJACENT_LANE_CARS_BEHIND){
                          car_left          = true ;
                          car_left_num      = i+1  ;
                          car_left_lane     = other_car_lane ;
                          car_left_distance = other_car_s_future - ego_s ;
                          car_left_speed    = other_car_speed ;
                    }
              }

              if(other_car_lane == ego_lane_actual+1){
                    if(0<(other_car_s_future - ego_s) && abs(other_car_s_future - ego_s)< SAFE_DIST_FROM_ADJACENT_LANE_CARS_AHEAD){
                          car_right          = true ;
                          car_right_num      = i+1  ;
                          car_right_lane     = other_car_lane ;
                          car_right_distance = other_car_s_future - ego_s ;
                          car_right_speed    = other_car_speed ;
                    }
                    else if(0>(other_car_s_future - ego_s) && abs(other_car_s_future - ego_s)< SAFE_DIST_FROM_ADJACENT_LANE_CARS_BEHIND){
                          car_right          = true ;
                          car_right_num      = i+1  ;
                          car_right_lane     = other_car_lane ;
                          car_right_distance = other_car_s_future - ego_s ;
                          car_right_speed    = other_car_speed ;
                    }
              }

          } //end of sensor fusion: for(int i =0; i< sensor_fusion.size(); i++)
          // ---------------- END of HANDLING COLLISIONS -PART 1: Reading Sensor Fusion Data of Other Cars Complete----------------------------------------------------------------------



          // --------------------------------------------------------------------------------------------------------------------------------------------
          // ---------------- HANDLING COLLISIONS -PART 2: LANE CHANGE LOGIC ----------------------------------------------------------------------------
          // --------------------------------------------------------------------------------------------------------------------------------------------

          //If there is a slow moving car_ahead i) try to change lanes ii) definitely slow down to not collide with the car
          if(car_ahead == true){
                //---------------------------- Print Some Messages for Clarity ---------------------------------------------------------------------------------------
                cout<<"\n\n CAR#" << car_ahead_num <<" AHEAD: ,@Lane: "<<car_ahead_lane<<" ,@Distance-ahead: "<<car_ahead_distance<<" m ,@Speed: "<<car_ahead_speed*2.24<<" mph .";
                if(car_left == true) {
                      if(car_left_distance>=0 )
                          cout<<"\n NOT SAFE to move LEFT!!! Car#" << car_left_num <<" ,@Lane: "<<car_left_lane<<" ,@Distance-ahead: "<<car_left_distance<<" m ,@Speed: "<<car_left_speed*2.24<<" mph .";
                      else
                          cout<<"\n NOT SAFE to move LEFT!!! Car#" << car_left_num <<" ,@Lane: "<<car_left_lane<<" ,@Distance-behind: "<<abs(car_left_distance)<<" m ,@Speed: "<<car_left_speed*2.24<<" mph .";
                }
                if(car_right == true) {
                      if(car_right_distance>=0 )
                          cout<<"\n NOT SAFE to move RIGHT!!! Car#" << car_right_num <<" ,@Lane: "<<car_right_lane<<" ,@Distance-ahead: "<<car_right_distance<<" m ,@Speed: "<<car_right_speed*2.24<<" mph .";
                      else
                          cout<<"\n NOT SAFE to move RIGHT!!! Car#" << car_right_num <<" ,@Lane: "<<car_right_lane<<" ,@Distance-behind: "<<abs(car_right_distance)<<" m ,@Speed: "<<car_right_speed*2.24<<" mph .";
                }
                //---------------------------- End of Print Messages for Clarity ---------------------------------------------------------------------------------------


                //-------------------------- LANE CHANGE LOGIC --------------------------------------------
                //If the ego_car is in Lane1, or Lane2 and there is no car on the left, move to left lane
                if((ego_lane_actual >0) && (car_left == false)) {
                    ego_lane_intended = ego_lane_actual -1;
                    cout<<"\n Ego-Car Moving Left to Lane#"<<ego_lane_intended;
                }
                //If the ego_car is in Lane0, or Lane1 and there is no car on the right, move to right lane
                else if((ego_lane_actual <2) && (car_right == false)) {
                    ego_lane_intended = ego_lane_actual +1;
                    cout<<"\n Ego-Car Moving Right to Lane#"<<ego_lane_intended;
                }
                //It is not safe to change lanes. Continue on the same lane
                else {
                    ego_lane_intended = ego_lane_actual ;
                    cout<<"\n Ego-Car NOT SAFE to Change Lane !!! Continuing in SAME LANE#"<<ego_lane_intended;
                }
                //-------------------------- END OF LANE CHANGE LOGIC --------------------------------------------
                /*

                //--------------------------  INCREMENTING/DECREMENTING REF_VEL --------------------------------------------
                ---> The following code increments/decrements the REF_VEL only every cycle and is inefficient.
                ---> Since there are many waypoints a car needs to go to in one cycle (we say 50waypoints but could be lesser),
                ---> we can also increment/decrement REF_VEL for every waypoint in the PATH PLANNER BELOW
                */

                /*
                //Decrease the REF_VEL of the ego-car gradually when
                //---> There is another car in front pg eg0-car.
                //---> 0.224 corresponds to a decrease in acceleration of ~5m/s^2
                if(car_ahead == true) {
                  REF_VEL -= 0.224;
                }
                //Increase the REF_VEL of the ego-car gradually when
                //---> i) when there is a cold start(starting at REF_VEL =0)
                //---> ii) if the ego-speed had falled since there was a slower car in front of ego-car and the ego-speed needs to pick up again
                //---> 0.224 corresponds to a increase in acceleration of ~5m/s^2
                else if(REF_VEL < IDEAL_SPEED_LIMIT){
                  REF_VEL += 0.224;
                }
                */
          }//eof if(car_ahead == true)


          // --------------------------------------------------------------------------------------------------------------------------------------------
          // ------------------------ STEP 1 of PATH PLANNER: CREATE 5 POINTS SPACED FAR APART : GENERATE ptsx & ptsy -----------------------------------
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
          vector<double> next_wp0 = getXY(ego_s+30, (2+4*ego_lane_intended), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(ego_s+60, (2+4*ego_lane_intended), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(ego_s+90, (2+4*ego_lane_intended), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
          // ------------------------ STEP 2 of PATH PLANNER: CREATE SPLINE : FROM THE FAR SPACED 5 POINTS ------------------------------------------
          // ------------------------------- spl = spline-from(ptsx,ptsy) ---------------------------------------------------------------------------
          // ----------------------------------------------------------------------------------------------------------------------------------------
          //ksw comments: define a spline spl
          tk::spline spl;

          //ksw comments: set the anchor points ptsx and ptsy (from above) to the spline
          //ptsx, ptsy are the 5 points from above whichc are also transformed to car's local co-ordinates (whit car at zero)
          spl.set_points(ptsx, ptsy);




          // ---------------------------------------------------------------------------------------------------------------------------------------
          // ------ STEP 3 of PATH PLANNER: CREATE THE ACTUAL PATH POINTS: next_x_vals, next_y_vals for the Path Planner ---------------------------
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

          double per_time_step_dist ;
          double per_time_step_x    ;

          double x_point_car_coord, y_point_car_coord;
          double x_point_map_coord, y_point_map_coord;
          //Fill up the rest of our path planner after filling it with previous points,
          //Here we will always output 50 points
          for(int i =1; i<=50-previous_path_x.size(); i++) {
              /*
              INCREMENTING/DECREMENTING REF_VEL
              ---> Since there are many waypoints a car needs to go to in one cycle (we say 50waypoints but could be lesser),
              ---> we increment/decrement REF_VEL for every waypoint in the PATH PLANNER BELOW. This is more efficient
              */

              //Decrease the REF_VEL of the ego-car gradually when
              //---> There is another car in front pg eg0-car.
              //---> 0.224 corresponds to a decrease in acceleration of ~5m/s^2.
              //---> But since we are decrementing every waypoint (and not every cycle) I decrement it at an even slower rate of 0.224/2.0 or 0.224/1.5
              if((car_ahead == true) && (REF_VEL > car_ahead_speed*2.24*0.93 )) {
                REF_VEL -= 0.224/1.5;
              }
              //Increase the REF_VEL of the ego-car gradually when
              //---> i) when there is a cold start(starting at REF_VEL =0)
              //---> ii) if the ego-speed had falled since there was a slower car in front of ego-car and the ego-speed needs to pick up again
              //---> 0.224 corresponds to a increase in acceleration of ~5m/s^2
              //---> But since we are incrementing every waypoint (and not every cycle) I increment it at a slower rate of 0.224/3.0 or 0.224/2.0
              else if(REF_VEL < IDEAL_SPEED_LIMIT){
                REF_VEL += 0.224/2.0;
              }

              // We want the car to move at 49.5 mph(49.5/2.24 m/s). so the per_time_step_dist covered needs to be 0.02*REF_VEL/2.24 meter
              // This distance is an arc along the spline. However similar to sample_dist, we approximate per_time_step_dist as a straight line instead of arc
              // Hence per_time_step_x, per_time_step_y, per_time_step_dist form the base, height and hypotenuse respectively of the Right-Angled-Triangle-2
              // Right-Angled-Triangle-1 & Right-Angled-Triangle-2 are 'similar'. This is used to establish unknowns per_time_step_x
              // per_time_step_y is not estimated from right_triangle but use spline directly (in loop)
              per_time_step_dist  =  0.02*REF_VEL/2.24 ;
              per_time_step_x     = sample_x*per_time_step_dist/sample_dist    ;

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
