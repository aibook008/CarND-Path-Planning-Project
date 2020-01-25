#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
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
 //define some tuning variables
  

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
  int lane=1;
  double ref_vel=0;
  double target_speed=48;


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

  h.onMessage([&target_speed,&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          // double dist_inc=0.5;
          // for (int i=0;i<50;i++)
          // {
          //   next_x_vals.push_back(car_x+dist_inc*i*cos(deg2rad(car_yaw)));
          //   next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          // }
          //define the reference velocity
          if(ref_vel<target_speed)
          {ref_vel+=2;}
          //deal with sensor fusion data
          int index=0;
          double front_object_s=9999;
          double front_object_v=9999;
          bool front =false;
          for (index=0;index<sensor_fusion.size();index++)
          {
         
            //get the fused objects info
            double vx=sensor_fusion[index][3];
            double vy=sensor_fusion[index][4];
            double checked_velocity=std::sqrt(vx*vx+vy*vy);
            double object_s=sensor_fusion[index][5];
            double object_d=sensor_fusion[index][6];
            if( object_d < ( 2 + 4 * lane + 2 ) && object_d > ( 2 + 4 * lane - 2 ))
            {
              if ((object_s-car_s)>0 && (object_s-car_s)<30 )
              {
                front=true;
                front_object_s=object_s;
                front_object_v=checked_velocity;
                std::cout<<"front observed"<<std::endl;
                std::cout<<"front_object_v"<<front_object_v<<std::endl;
                std::cout<<"front observed"<<std::endl;
                break;
              }
            }       

          }
          if (front)
          {
            ref_vel=std::min(ref_vel,front_object_v);
             std::cout<<"ref_vel"<<ref_vel<<std::endl;
          }
          //temp solution for avoiding collision

          //first try for the const heading angle
          //get previous points
          //the way to generate a circular path
          double pos_x;
          double pos_y;
          double angle;
          int path_size_previous=previous_path_x.size();
          for(int i=0;i<path_size_previous;i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
            
          }
          if(path_size_previous==0)
          {
            pos_x=car_x;
            pos_y=car_y;
            angle=deg2rad(car_yaw);
          }
          else
          {
            pos_x=previous_path_x[path_size_previous-1];
            pos_y=previous_path_y[path_size_previous-1];
            double pos_x2=previous_path_x[path_size_previous-2];
            double pos_y2=previous_path_y[path_size_previous-2];
            angle=atan2(pos_y-pos_y2,pos_x-pos_x2);
          }
          //  double dist_inc=0.5;
          // for (int i=0;i<50-path_size_previous;i++)
          // {
          //   next_x_vals.push_back(pos_x);
          //   next_y_vals.push_back(pos_y);
          //   pos_x+=(dist_inc)*cos(angle+(i)*(pi()/100));
          //   pos_y+=(dist_inc)*sin(angle+(i)*(pi()/100));
          // }

          //realize to stay in a constant lane
          //first thing is to select the referrence point as it will be selected as the local coordination original point
          vector<double> ptsx;
          vector<double> ptsy;
          double refer_x;
          double refer_y;
          double refer_yaw;
          if(path_size_previous<2)
          {
            refer_x=car_x;
            refer_y=car_y;
            refer_yaw=car_yaw;

          }
          else
          {
            refer_x = previous_path_x[path_size_previous - 1];
            refer_y=previous_path_y[path_size_previous-1];
            double ref_x_prev = previous_path_x[path_size_previous - 2];
            double ref_y_prev = previous_path_y[path_size_previous - 2];
           // refer_y=previous_path_y[path_size_previous-1];
            refer_yaw=atan2(refer_y - ref_y_prev, refer_x - ref_x_prev);
            
          }
          ptsx.push_back(refer_x);
          ptsy.push_back(refer_y);
          //transform to have a spline in the local coordination
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          for(int i=0;i<ptsx.size();i++)
          {
            //do the transition
            double shifx = ptsx[i] - refer_x;
            double shify = ptsy[i] - refer_y;

            ptsx[i] = (shifx * cos(0 - refer_yaw) - shify * sin(0 - refer_yaw));
            ptsy[i] = (shifx * sin(0 - refer_yaw) + shify * cos(0 - refer_yaw));
          }

          tk::spline s;
          s.set_points(ptsx,ptsy);
          double target_x=30;
          double target_y=s(target_x);
          double target_dis=sqrt(( target_x ) * ( target_x ) + ( target_y ) * ( target_y ));
          double x_add_on=0;
          for(int i=0;i<50-previous_path_x.size();i++)
          {
            double N=(target_dis/(0.02*ref_vel/2.24));
            double x_point=x_add_on+(target_x)/N;
            double y_point=s(x_point);
            x_add_on=x_point; //value replaced for further iteration

            // rotate back to normal after rotating it earlier
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = (x_ref * cos(refer_yaw) - y_ref * sin(refer_yaw));
            y_point = (x_ref * sin(refer_yaw) + y_ref * cos(refer_yaw));

            x_point += refer_x;
            y_point += refer_y;

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