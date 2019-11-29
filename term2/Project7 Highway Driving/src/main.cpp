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
  
  // set some vars here
  double ref_vel=0.0;
  int lane =1;
  int too_long_dist=0;
  double safe_side_s=35.0;
  double safe_front_s=40.0;

  h.onMessage([&ref_vel,&lane,&too_long_dist,&safe_side_s,&safe_front_s,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          int prev_size=previous_path_x.size();
          
          if (prev_size>0)
          {
            car_s=end_path_s;
          }
          bool front_too_close=false;
          bool change_to_right_lane=true;
          bool change_to_left_lane=true;
          
          if(lane==0)
          {
            change_to_left_lane=false;
          }
          if(lane==2)
          {
            change_to_right_lane=false;
          }
          
          for(int i=0; i<sensor_fusion.size(); i++)
          {
            float d=sensor_fusion[i][6];
            double vx=sensor_fusion[i][3];
            double vy=sensor_fusion[i][4];
            double check_other_car_speed=sqrt(vx*vx+vy*vy);
            double check_other_car_s=sensor_fusion[i][5];
            
            check_other_car_s+=(double)prev_size*0.02*check_other_car_speed;
            
            if(d<(4*lane+4)&&(d>4*lane))
            {                     
              if((check_other_car_s>car_s)&&((check_other_car_s-car_s)<safe_side_s))
              {
                front_too_close=true;
              }
            }
            if(d<(4*(lane+1)+4)&&(d>4*(lane+1)))
            {
              if((check_other_car_s>car_s)&&((check_other_car_s-car_s)<safe_side_s))
              {
                change_to_right_lane=false; 
              }
              if((check_other_car_s<car_s)&&((car_s-check_other_car_s)<safe_side_s))
              {
                change_to_right_lane=false; 
              }
            }
            if(d<(4*(lane-1)+4)&&(d>4*(lane-1)))
            {
              if((check_other_car_s>car_s)&&((check_other_car_s-car_s)<safe_side_s))
              {
                change_to_left_lane=false;
              }
              if((check_other_car_s<car_s)&&((car_s-check_other_car_s)<safe_side_s))
              {
                change_to_left_lane=false; 
              }
            }
          }
          
          if(front_too_close)
          {
            ref_vel-=0.2;
            too_long_dist+=1;
            if(change_to_left_lane&&(too_long_dist>safe_front_s))
            {
              too_long_dist=0;
              lane-=1;
            }
            else if(change_to_right_lane&&(too_long_dist>safe_front_s))
            {
              too_long_dist=0;
              lane+=1;
            }
          }
          else if(ref_vel<49.5)
          {
            ref_vel+=0.2;
          }
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          int add_points =prev_size;
          
          if(prev_size<2)
          {
            double prev_car_x=car_x-cos(car_yaw);
            double prev_car_y=car_y-sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
                        
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          else
          {
            ref_x=previous_path_x[add_points-1];
            ref_y=previous_path_y[add_points-1];
            
            double ref_x_prev=previous_path_x[add_points-2];
            double ref_y_prev=previous_path_y[add_points-2];
            ref_yaw=atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }
          
          double free_space=30.0;
          
          vector<double> next_wp0=getXY(car_s+free_space,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1=getXY(car_s+free_space*2,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2=getXY(car_s+free_space*3,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          
          for(int i =0;i<ptsx.size();i++){
            double shift_x=ptsx[i]-ref_x;
            double shift_y=ptsy[i]-ref_y;
            ptsx[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
          
          tk::spline s;
          
          s.set_points(ptsx, ptsy);
          
          
          for(int i=0;i<add_points;i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double goal_x=30.0;
          double goal_y=s(goal_x);
          double goal_dist=sqrt((goal_x)*(goal_x)+(goal_y)*(goal_y));
          double next_x_add=0;
          
          
          for(int i=1;i<=50-previous_path_x.size();i++)
          {
            double N=(goal_dist/(0.02*ref_vel/2.24));
            double x_points=next_x_add+goal_x/N;
            double y_points=s(x_points);
            
            next_x_add=x_points;
            
            double x_ref=x_points;
            double y_ref=y_points;
            
            x_points=x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
            y_points=x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);
            
            x_points+=ref_x;
            y_points+=ref_y;
            
            next_x_vals.push_back(x_points);
            next_y_vals.push_back(y_points);
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