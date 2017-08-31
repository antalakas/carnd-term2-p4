#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // initial
  // I started using the parameters from the lesson. Using throttle: 0.3 the track is completed flawlessly
//  const double init_Kp = 0.2;
//  const double init_Ki = 0.004;
//  const double init_Kd = 3.0;

  // I then tried to implement twiddle and used the outcome for throttle 0.3
  // The result was better. Please note that throttles 0.4, 0.5, 0.6 in this case lead to crashes
  // throttle = 0.3 / twiddle
//  const double init_Kp = 0.2;
//  const double init_Ki = -0.096;
//  const double init_Kd = 3.2;

  // Using the previous values i optimized parameters for throttle 0.3 and used them to execute
  // using throttle 0.4: success
  // throttle = 0.4 / twiddle
//  const double init_Kp = 0.3;
//  const double init_Ki = -0.196;
//  const double init_Kd = 3.281;

  // Using the previous values i optimized parameters for throttle 0.4 and used them to execute
  // using throttles 0.5: success, 0.6: after some rounds a crash occured
  // throttle = 0.5, 0.6 / twiddle
  const double init_Kp = 0.2;
  const double init_Ki = -0.196;
  const double init_Kd = 3.381;

  // Line 95 should be uncommented to execute twiddle

  // At the end of this file you can find the output of the twiddle optimization runs (4 runs
  // throttles: 0.3, 0.4, 0.5, 0.6)

  pid.Init(init_Kp, init_Ki, init_Kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          pid.UpdateError(cte);
//          pid.Twiddle();
          steer_value = pid.TotalError();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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

// RUN 1 (0.3)
///Users/andreas/workspace/andreas/udacity/term2/submissions/carnd-term2-p4/cmake-build-debug/pid
//    Listening to port 4567
//Connected!!!
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//Best error = 0.000158723
//    ------ PID params ------
//Kp = 0.2
//Kd = 3
//Ki = 0.004
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.181988
//    ------ PID params ------
//Kp = 0.1
//Kd = 3
//Ki = 0.004
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.744251
//    ------ PID params ------
//Kp = 0.2
//Kd = 3
//Ki = 0.004
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.072146
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = 0.004
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.11526
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.00357604
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.148687
//    ------ PID params ------
//Kp = 0.11
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 1.27103
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.181988
//    ------ PID params ------
//Kp = 0.2
//Kd = 2.99
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.3481
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.00559504
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = -0.206
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 1.0042
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0324
//    ------ PID params ------
//Kp = 0.119
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.15101
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 1.681e-05
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.199
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.319451
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.199
//Ki = -0.195
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.014161
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.199
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//
//Process finished with exit code 15


// RUN 2: 0.4
///Users/andreas/workspace/andreas/udacity/term2/submissions/carnd-term2-p4/cmake-build-debug/pid
//    Listening to port 4567
//Connected!!!
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//Best error = 0.000358561
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.2
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0652802
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.153116
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.1
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.077618
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.096
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.104652
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.00180625
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.322738
//    ------ PID params ------
//Kp = 0.19
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.151788
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0736037
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.11
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0820822
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.00767376
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.306
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0897002
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.20313
//    ------ PID params ------
//Kp = 0.201
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0566916
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.2
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.00154449
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.247705
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.281
//Ki = -0.295
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.017849
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0892814
//    ------ PID params ------
//Kp = 0.2109
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.268739
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//
//Process finished with exit code 15

// RUN 3
///Users/andreas/workspace/andreas/udacity/term2/submissions/carnd-term2-p4/cmake-build-debug/pid
//    Listening to port 4567
//Connected!!!
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//Best error = 0.000191056
//    ------ PID params ------
//Kp = 0.3
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 1.44962
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0616529
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0450713
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.381
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0809972
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.381
//Ki = -0.296
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.730341
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.381
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0126563
//    ------ PID params ------
//Kp = 0.31
//Kd = 3.381
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 32.1546
//    ------ PID params ------
//Kp = 0.31
//Kd = 3.271
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//
//Process finished with exit code 15
//
//CRASH!!!

// RUN 4
///Users/andreas/workspace/andreas/udacity/term2/submissions/carnd-term2-p4/cmake-build-debug/pid
//    Listening to port 4567
//Connected!!!
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//Best error = 8.54978e-05
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.381
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 10.0521
//    ------ PID params ------
//Kp = 0.1
//Kd = 3.381
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.0949872
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.381
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.505094
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.281
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 0.152881
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.381
//Ki = -0.196
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//800 \ 1000
//1000 \ 1000
//error = 365.766
//    ------ PID params ------
//Kp = 0.2
//Kd = 3.381
//Ki = -0.296
//    ------   TWIDDLE  ------
//200 \ 1000
//400 \ 1000
//600 \ 1000
//
//Process finished with exit code 15
//
//
//CRASH