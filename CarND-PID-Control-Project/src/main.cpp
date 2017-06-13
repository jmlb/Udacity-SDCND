#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <list>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double total_cnt = 0;
double rms_err = 0;
double acc_cte = 0;
double set_thr = 0.6;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// argc nbr of argument && argv string of arg Kp,Kd, Ki 
int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  //Convert string to doubles
  double Kp_init = atof(argv[1]);
  double Kd_init = atof(argv[2]);
  double Ki_init = atof(argv[3]);
  double min_thr = atof(argv[4]);
  pid.Init(Kp_init, Kd_init, Ki_init);
  int count = 0;
  double prev_cte = 0;



  h.onMessage([&pid, &Kp_init, &Ki_init, &Kd_init, &count, &min_thr](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          //update error


          // COMMENTS
          // >> Describe how the final hyperparameters were chosen.
          // I used both visualization and the not-normalized rms error to find the optimum PID gains.
          // The PID gains are adjusted to reduce the not-normalized rms accumulated over the course 
          // of a full lap completed by the agent.
          //
          // $$ rms = sum of cte^2 $$
          //
          // Below are our optimized PID gains value:
          //
          // Kp = 0.032, K_d = 0.006, K_i=1.3

          // ./pid 0.032 0.006 1.3 0.3
          //
          //The last number (0.3) is the minimum threshold allowed. The throttle opening is limted to the range [0.3, 0.8] 
          //
          //
          // >> Description of the effect each of the P, I, D components had in your implementation.
          // * P gain: controls how fast the agent reacts to error. With a high K_p, the agent 
          // reacts fast but result in an oscillatory trajectory of the car around the truth value of 
          // the cross track position.
          // * I gain: enables to damp the oscillatory behavior. With an optimum I and P the car 
          //is able to converge towards the true cross track position (resulting in cte =0). 
          // If K_i is too small, it will take more oscillations before the agent stabilizes. If I 
          // is too large, the cte could continuouslyly increase, and the car would get further away from the true 
          //cross track position.
          // * D gain: K_d enables to compensate for the drift of the steering. A large K_d would make it 
          // harder for the car to turn left or right and reduce the cte.

          // My strategy was to first optimize K_p and K_i simulatenously while keeping K_d=0 until the trajectory 
          // of the car was satisfactory. By changing K_p and K_i simulatenously, we prevent the system to be trapped 
          // in a local minimum. Then, K_d was varied to further improve the car behavior in autonomous mode. 
          // This first optimization was conducted at constant throttle: 0.3. 
          // If the throttle is increased, the car would show more oscillatory behavior. For the car to drive faster, 
          // without sacrificing on the overall car behavior on the road, we implement a small decision tree using if/else statements.

          // For example, if the accumulated cte (in absolute value) over the last 10 frames are below a threshold, we can assume 
          // that the car is "stable", and therefore increase the throttle. 
          // When the car starts to be unstable, i.e it shows significant oscillations, the throttle is reduced. 
          


          pid.UpdateError(cte);
          rms_err = rms_err + cte*cte;
          steer_value = pid.TotalError();
          //calculate accumulated cte
          acc_cte = acc_cte + fabs(cte);
          std::cout << " Accumulated CTE" << acc_cte << std::endl;
          //Keep steer value in range [-1, 1]
          if (steer_value > 1){steer_value = 1;}

          if (steer_value < -1){steer_value = -1;}
          
          if (acc_cte < 6){  //
            //we want 10 consecutives cte to total value below 2 (so in average abs(cte)=0.2 per frame)
            if (count == 10){
              //check speed limit
              if (speed <= 65){ //if speed limit not reached-> speedup
                set_thr = set_thr + 0.05;
                if (set_thr > 0.8){set_thr = 0.8;} //limit throttle to 0.8
              }
              else{set_thr = set_thr - 0.1;} //above speed limit--> slow down
              count = 0;
              //reset accumulated cte
              acc_cte = 0;
            }
            else{count = count + 1;}
          }
          else{
            if (set_thr == 0.8){set_thr = -0.1;} //if angles 
            else{ set_thr = min_thr;}
            count = 0;
            //reset accumulated cte
            acc_cte = 0;
          }

          // DEBUG Normalized rms_error = rms_err/total_cnt
          std::cout << "(Not-normalized) RMS_error: " << rms_err << std::endl;
          total_cnt += 1;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = set_thr;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
