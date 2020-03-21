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

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values
  stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

struct controls {
  int lane;
  double velocity;
};
vector <double> avg_speed;
typedef struct controls Struct;
int timestamp = 0;
template <typename T> Struct stateMachine(T sensor_fusion, int lane, double ref_vel, int prev_size,
  double car_s, double end_path_s, double car_speed){
  Struct c;

  if(prev_size > 0){
    car_s = end_path_s;
  }

  bool too_close = false;
  bool sped_up = false;

  bool left_possible = true;
  bool right_possible = true;
  double acc_dist = 100;
  double ref_vel_delta = 0;
  double car_ahead_speed = 0;
  //find rev_v to used
  double mindist = 100;

  for(int i = 0; i < sensor_fusion.size(); i ++){

    //car is in my lane
    float d = sensor_fusion[i][6];
    // check car in same lane
    if (d < (2+4*lane+2)&& d > (2+4*lane-2)){

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      check_car_s += ((double)prev_size*0.02*check_speed);
      //acc_dist = check_car_s - car_s;


        if((check_car_s > car_s)){

          if((check_car_s - car_s)<mindist){
            mindist = (check_car_s - car_s);
            car_ahead_speed = check_speed;
            avg_speed.push_back(check_speed);

          }
        }


      if(avg_speed.size()>10)
      {
        avg_speed.resize(10);
      }
      /*
      if((check_car_s > car_s) && ((check_car_s - car_s) <50) ){
        acc_dist = check_car_s - car_s-20;
        too_close = true;
        car_ahead_speed = check_speed;
      }
      */
    }

    if(mindist<50){
      too_close = true;
      //car_ahead_speed = check_speed;
      acc_dist = mindist- 30;
    }
    //check car in left lane

    if (d < (2+4*(lane-1)+2)&& d > (2+4*(lane-1)-2)&& lane > 0){
      //std::cout << d<< std::endl;

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      check_car_s += ((double)prev_size*0.02*check_speed);
      //std::cout << abs(check_car_s - car_s)<< std::endl;
      if(abs(check_car_s - car_s) < 20){

        left_possible = false;

      }
    }
    if (d < (2+4*(lane+1)+2)&& d > (2+4*(lane+1)-2)&& lane<2){
      //std::cout << d<< std::endl;

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      check_car_s += ((double)prev_size*0.02*check_speed);
      //std::cout << abs(check_car_s - car_s)<< std::endl;
      if(abs(check_car_s - car_s) < 20){

        right_possible = false;

      }
    }
  }
  if(lane==0){
    left_possible = false;
    //changing lanes
  }
  if(lane==2){
    right_possible = false;
  }
  //std::cout << left_possible<< std::endl;
  /*
  if(car_speed> 40.0){
    sped_up = true;
  }
  else{
    sped_up = false;
  }
*/

/*
  timestamp+=1;
  if(left_possible && timestamp>100 && too_close ){
    lane -=1;
    std::cout <<"lane change LEFT "<< std::endl;
    timestamp = 0;

  }
  if(right_possible && timestamp>100 && too_close ){
    lane +=1;
    std::cout <<"lane change RIGHT "<< std::endl;
    timestamp = 0;
  }
*/

  //if(too_close && !left_possible && !right_possible){

  if(!too_close && ref_vel <49.5){
    ref_vel += .224;//224
  }

/*
double factor = 0.05;
  if(too_close){
    ref_vel = 2*car_ahead_speed*2.23/(1+exp(-factor*(acc_dist)));
    if(ref_vel==0 || ref_vel>49.5 ){
      ref_vel = 49.5;
    }
  }
*/
int sumTotal = 0;
for(int k=0; k < avg_speed.size(); ++k){
     // not sure what to put here basically.
     sumTotal += avg_speed[k];

 }
car_ahead_speed = sumTotal / 10;


  if(too_close){
    /*
    ref_vel_delta = .5 * acc_dist/(1+abs(acc_dist));
    double k = 1;
    double cruise_speed = 49.5;
    double x0 = log((cruise_speed-car_ahead_speed)/cruise_speed);

    ref_vel = cruise_speed/(1+exp(-k* (0.3*acc_dist - x0 )));
    */
    ref_vel = 49.5 * acc_dist/(1 + abs(acc_dist));
    if(ref_vel<=0){
      ref_vel = 0;
    }
    if(ref_vel>49.5){
      ref_vel = 49.5;
    }

    /*
    //ref_vel_delta = .3 * tanh(7*acc_dist);
    //ref_vel += ref_vel_delta;
    if(ref_vel==0 || ref_vel>49.5 ){
      ref_vel = 49.5;
    }

    */
  }
  //std::cout <<"ref_vel"<< ref_vel << std::endl;

/*
ref_vel_delta =  0.1 * acc_dist - (0.224*30);

ref_vel += ref_vel_delta;
if(ref_vel>49.5){
  ref_vel=49.5;
}

*/
//ref_vel_delta =  0.1 * acc_dist - (0.224*30);

//std::cout <<"car_ahead_speed m/s"<< car_ahead_speed << std::endl;
//std::cout <<"own speed m/s"<< car_speed*0.447 << std::endl;
//std::cout <<"target_speed m/s"<< target_speed << std::endl;
/*
std::cout <<"left_possible? "<< left_possible << std::endl;
std::cout <<"right_possible? "<< right_possible << std::endl;
std::cout <<"too_close? "<< too_close << std::endl;
std::cout <<"car speed? "<< car_speed << std::endl;
std::cout <<"curren lane? "<< lane << std::endl;
*/
c.lane = lane;
c.velocity = ref_vel;

return c;
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

  //int lane = 1;
  int lane = 0;

  double ref_vel = 49.5;



  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
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

          int prev_size = previous_path_x.size();

          json msgJson;

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


        Struct state_controls;

        state_controls = stateMachine(sensor_fusion, lane, ref_vel, prev_size, car_s, end_path_s, car_speed);
        ref_vel = state_controls.velocity;
        lane = state_controls.lane;
        //std::cout << sped_up << " sped up" << std::endl;

        vector <double> ptsx;
        vector <double> ptsy;

        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);

        if(prev_size < 2){
          double prev_car_x = car_x - cos(car_yaw);
          double prev_car_y = car_y - sin(car_yaw);

          ptsx.push_back(prev_car_x);
          ptsx.push_back(car_x);

          ptsy.push_back(prev_car_y);
          ptsy.push_back(car_y);
        }
        else {

          ref_x = previous_path_x[prev_size-1];
          ref_y = previous_path_y[prev_size-1];

          double ref_x_prev = previous_path_x[prev_size-2];
          double ref_y_prev = previous_path_y[prev_size-2];
          ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);
        }



        //30 60 90
        // Transform from Frenet s,d coordinates to Cartesian x,y
        vector <double> next_wp0 = getXY(car_s + 30,(2+4 * lane),map_waypoints_s,map_waypoints_x, map_waypoints_y);
        vector <double> next_wp1 = getXY(car_s + 60,(2+4 * lane),map_waypoints_s,map_waypoints_x, map_waypoints_y);
        vector <double> next_wp2 = getXY(car_s + 90,(2+4 * lane),map_waypoints_s,map_waypoints_x, map_waypoints_y);

        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);

        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);


        for ( int i = 0; i < ptsx.size(); i++ ) {
          double shift_x = ptsx[i] - ref_x;
          double shift_y = ptsy[i] - ref_y;

          ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
          ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
        }

        tk::spline s;

        for(int i = 0; i<ptsx.size(); i++){
          std::cout << ptsx[i] << "   "<< ptsy[i]   << std::endl;
        }
        std::cout << "--------------" << std::endl;

        vector <double> sorted_ptsx;
        vector <double> sorted_ptsy;
        for (auto i: sort_indexes(ptsx)) {

          sorted_ptsx.push_back(ptsx[i]);
          sorted_ptsy.push_back(ptsy[i]);

        }

        s.set_points(ptsx, ptsy);
        vector<double> next_x_vals;
        vector<double> next_y_vals;

		    for (int i = 0; i < previous_path_x.size(); i++) {
  			     next_x_vals.push_back(previous_path_x[i]);
  			     next_y_vals.push_back(previous_path_y[i]);
		    }

        // Calculate distance y position on 30 m ahead.
        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt(target_x*target_x + target_y*target_y);

        double x_add_on = 0;

        for( int i = 1; i < 50 - prev_size; i++ ) {

          double N = target_dist/(0.02*ref_vel/2.24);
          double x_point = x_add_on + target_x/N;
          double y_point = s(x_point);

          x_add_on = x_point;

          double x_ref = x_point;
          double y_ref = y_point;

          x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);


          x_point += ref_x;
          y_point += ref_y;

          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);
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
