#include "PID.h"
#include "json.hpp"
#include <iostream>
#include <math.h>
#include <uWS/uWS.h>

// for convenience
using json = nlohmann::json;

// Three possibilities:
// useTwiddle = none : twiddle algorithm deactived
// useTwiddle = simple : use twiddle algo in main.cpp
// useTwiddle = thread : use twiddle algo as worker thread in PID.cpp
enum TwiddleModes { none, simple, thread };

TwiddleModes useTwiddle = thread;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
double distance = 0.0;
int counter = 0;
int cycle = 0;

int main() {
  uWS::Hub h;

  PID pid;

  // ********************************************
  // Initialize the pid variable.
  // pid.Init(0.5,1.33,3);
  // pid.Init(0.5,1.43,3.06);
  // pid.Init(0.014,0.111,0.112);
  // pid.Init(0.0997544, 0.126883, 0.122702);

  // Final values with thread-twiddle optimized
  //pid.Init(0.1167219, 0.132172, 0.133131);
  pid.Init(0, 0.13,0);
  // ********************************************

  // I have implemented two twiddle algorithms
  // 1) in the main.cpp method
  // 2) as separate worker thread in PID.cpp (useTwiddle = True)
  if (useTwiddle == thread) {
    pid.startTwiddle();
  }

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
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
          * Calcuate steering value here, remember the steering value is [-1, 1]
          * NOTE: Feel free to play around with the throttle and speed. Maybe
          * use another PID controller to control the speed!
          */

          // in case of no twiddle or twiddle in PID.cpp
          // start with Updating the error and calculate the PID response
          if ((useTwiddle == none) || (useTwiddle == thread)) {
            pid.UpdateError(cte);
            steer_value = pid.calcPID(cte);

            // in case of twiddle in worker thread of PID.cpp
            // additionally calculate the driven distance
            // in order to resample next twiddle step after
            // resampleDistance meters
            int resampleDistance = 500;

            if (useTwiddle == thread) {
              distance += pid.deltaTime * speed;   // distance = time * speed
              if (distance > resampleDistance) {
                pid.twiddleWait = false;  // now initialte resample
                std::cout << "<Besterror:" << pid.bestError <<" Kp: "
                          << pid.Kp << " Ki: " << pid.Ki
                          << " Kd: " << pid.Kd << std::endl;
                distance = 0.0;  // reset
              }
            } else {
              // DEBUG
              std::cout << "Total Error: " << pid.totalError << "  CTE: " << cte
                        << " Steering Value: " << steer_value << std::endl;
            }

            json msgJson;
            msgJson["steering_angle"] = steer_value;

            // implement a trivial controller for throttle
            // in case of steering angles > 0.15 --> reduce speed
            // otherwise go "full speed ahead"
            if (fabs(steer_value) < 0.15) {
              msgJson["throttle"] = 0.6;
            } else {
              msgJson["throttle"] = 0.2;
            }
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            if (useTwiddle == none) {
              std::cout << msg << std::endl;
            }

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // here now is the simple twiddle in main.cpp
          } else if (useTwiddle == simple) {
            counter++;

            // don't use distance here, but iterations
            // reset after 2500 iterations (close to one round in the track)
            if (counter % 2500 == 0) {
              std::cout << "Twiddle-Iteration: " << counter
                        << "\t best error: " << pid.bestError
                        << "\t parameters Kp = " << pid.Kp << " Ki = " << pid.Ki
                        << " Kd = " << pid.Kd << std::endl;

              std::vector<double> params;
              params = {pid.Kp, pid.Ki, pid.Kd};
              std::vector<double> mod_params;
              mod_params = {pid.p_error, pid.i_error, pid.d_error};
              pid.UpdateError(cte);

              std::cout << "Best Error: " << pid.bestError
                        << " Total Error:  " << pid.totalError << std::endl;

              // i = the PID parameter to be optimized
              // i = 0 : Ki , integral part
              // i = 1 : Kp , proportional part
              // i = 2 : Kd , differential part
              int i = -1;
              int factor1 = 10;
              int factor2 = 1;

              // just optimize a different parameter after factor1*factor2
              // iterations
              if (cycle < 1 * factor2 * factor1) {
                i = 1;
              } else if (cycle < 2 * factor2 * factor1) {
                i = 0;
              } else if (cycle < 3 * factor2 * factor1) {
                i = 2;
              } else if (counter > 3 * factor2 * factor1) {
                cycle = 0;
              }

              // for (int i = 0; i < 3; i++) {
              std::cout << "cycle = " << cycle << " , i = " << i << std::endl;

              params[i] += mod_params[i];

              double twiddle_current_error;

              twiddle_current_error = pid.totalError;

              // last optimization seems to work - go!
              if (twiddle_current_error <= pid.bestError) {
                mod_params[i] *= 1.1;
                pid.bestError = twiddle_current_error;

              // else try to decrease value
              } else {
                params[i] -= 2 * mod_params[i];
                twiddle_current_error = pid.totalError;

                // check if decrease works
                if (twiddle_current_error <= pid.bestError) {
                  pid.bestError = twiddle_current_error;
                  mod_params[i] *= 1.1;
                } else {
                  params[i] += mod_params[i];
                  mod_params[i] *= .90;
                }
              }

              pid.Kp = params[0];
              pid.Ki = params[1];
              pid.Kd = params[2];

              pid.p_error = mod_params[0];
              pid.i_error = mod_params[1];
              pid.d_error = mod_params[2];

              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              counter = 0;
              cycle++;

              pid.Init(pid.Kp, pid.Ki, pid.Kd);
              std::cout << "Total Error: " << pid.totalError << "  CTE: " << cte
                        << " Steering Value: " << steer_value << std::endl;
            } else {
              pid.UpdateError(cte);
              steer_value = pid.calcPID(cte);
              // DEBUG
              // std::cout << "Total Error: " << pid.totalError << "  CTE: " <<
              // cte << " Steering Value: " << steer_value << std::endl;

              json msgJson;
              msgJson["steering_angle"] = steer_value;

              // implement a trivial controller for throttle
              // in case of steering angles > 0.15 --> reduce speed
              // otherwise go "full speed ahead"
              if (fabs(steer_value) < 0.15) {
                msgJson["throttle"] = 0.6;
              } else {
                msgJson["throttle"] = 0.2;
              }
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              // std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
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
