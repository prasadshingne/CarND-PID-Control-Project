#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;
//using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// function to reset simulater for automated PID tuning
void resetSimulator(uWS::WebSocket<uWS::SERVER> ws)
{
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


int main() {
  uWS::Hub h;

  PID pid, pid_speed;
  /**
   * TODO: Initialize the pid variable.
   */

  vector<double> pid_val; //read 

  fstream init_file;
  init_file.open("pid.init", ios::in); // open initialization file
  if (init_file.is_open()){
    
    double num = 0.0; 
    while (init_file >> num) {
      pid_val.push_back(num);
    }
    
    init_file.close();
  }

  ofstream out_file;
  out_file.open("output.txt"); // create output file for each run --> used to compute cross track and speed errors to minimize during optimization

  for (size_t i = 0; i<pid_val.size(); ++i){ // read input file for PID parameters
    cout << pid_val[i] << endl;
  }

  //pid.Init(pid_val[0], pid_val[1], pid_val[2]); // read PID parameters steer values from input file --> used during optimization of the parameters

  //pid_speed.Init(pid_val[3], pid_val[4], pid_val[5]); // read PID parameters speed values from input file --> selected manually

  // PID
  pid.Init(0.14937642030249174, 0.0008028211368793284, 3.355222551300104); // set after manual tuning and optimization to minimize cros track error root mean squared error

  // PI
  //pid.Init(0.14937642030249174, 0.0008028211368793284, 0.0); // record behavior for PI control

  // P
  //pid.Init(0.14937642030249174, 0.0, 0.0); // record behavior for P control
  
  // PD
  //pid.Init(0.14937642030249174, 0.0, 3.355222551300104); // record behavior for P control

  pid_speed.Init(0.025, 0.0001, 2.5); // selected manually

  // Get starting timepoint 
  auto tstart = chrono::steady_clock::now(); 

  //auto tstep = high_resolution_clock::now();

  // specify target speed 
  double target_speed = 33.0; // mph --> based on the type of road in simuilator I assumed the speed limit to be 35 mph hence target was set to 33 mph

  // specify duration of sim in milliseconds
  double sim_dur = 50000; //ms --> used for repeated simulations during optimization


  h.onMessage([&pid, &pid_speed, &out_file, &tstart, &target_speed, &sim_dur](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          // update steer controller
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          // update speed controller
          double speed_err = target_speed-speed;
          pid_speed.UpdateError(speed_err);
          double throttle = pid_speed.TotalSpeedError();
          //double throttle = 0.4 ;

          // DEBUG
          auto now = chrono::steady_clock::now();
          double dur = chrono::duration_cast<chrono::milliseconds>(now - tstart).count();
          cout << "time step: " << dur << " CTE: " << cte << " Steering Value: " << steer_value << " Speed error :" << speed_err << 
          " throttle :" << throttle <<  " Speed: " << speed << endl; // output to screen

          // write output
          //tstep = high_resolution_clock::now(); 
          // out_file << tstep-tstart << cte << steer_value << speed << '\n';
          out_file << " " << dur << " " << cte << " " << steer_value << " " << speed_err << " " << throttle << " " << speed << '\n'; // write to output.txt
          // cout <<  cte << steer_value << speed << endl;
          //if ((tstep-tstart) > 10.0){
          //  out_file.close();
          //  exit(0);
          //}

          /*if (dur > sim_dur){ // use this section during optimization --> if simulation duration exceeds x value, close output file and reset simulation 
            out_file.close();
            cout << "time step: " << dur << " CTE: " << cte << " Steering Value: " << steer_value << " Speed error :" << speed_err << 
            " throttle :" << throttle <<  " Speed: " << speed << endl;
            resetSimulator(ws);
            exit(0);
          }*/

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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