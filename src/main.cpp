#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <chrono>
#include <string>
// for convenience
using json = nlohmann::json;
using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Constants

const double speed_to_m_per_s_factor = 0.44704;

const double show_waypoint_res = 2;
const int num_show_waypoint = 30;

int latency_in_ms = 100;
double latency_in_s = (latency_in_ms) / 1000.0;

const double max_steer_angle = 25;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

double cl_double(char * param, int argc, char * argv[], const double def) {
    for(int i = 0; i < argc; i++) {
        if(strcmp(param,argv[i])==0) {
            return stod(argv[i+1]);
        }
    }
    return def;
}


int cl_int(char * param, int argc, char * argv[], const int def) {
    for(int i = 0; i < argc; i++) {
        if(strcmp(param,argv[i])==0) {
            return stoi(argv[i+1]);
        }
    }
    return def;
}

typedef unsigned long  ulong;
extern ulong N;

int main(int argc, char*argv[]) {

  latency_in_ms = cl_int("--latency",argc,argv,100);
  latency_in_s = (latency_in_ms) / 1000.0;

//  N= cl_int("--N",argc,argv,10);
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    milliseconds ms = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch()
    );

    cout  << ms.count() << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v = v* speed_to_m_per_s_factor;


          vector<double> waypoints_x;
          vector<double> waypoints_y;

          // Transform Waypoints to car coordination system
          //
          for (int i = 0; i < ptsx.size(); i++) {
              double dx = ptsx[i] - px;
              double dy = ptsy[i] - py;
              waypoints_x.push_back(dx * cos(psi) + dy * sin(psi));
              waypoints_y.push_back(dy * cos(psi) - dx * sin(psi));
          }

          /*
          *
          * : Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          double* ptrx = &waypoints_x[0];
          double* ptry = &waypoints_y[0];
          Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
          Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);

          auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
          double cte = polyeval(coeffs, 0);  // px = 0, py = 0
          double epsi = -atan(coeffs[1]);  // p

          Eigen::VectorXd state(6);
#if 1

          double sa = j[1]["steering_angle"];
          double th = j[1]["throttle"];
          const double Lf = 2.67;
          const double px_act = v * latency_in_s;
          const double py_act = 0;
          const double psi_act = - v * sa * latency_in_s / Lf;
          const double v_act = v + th * latency_in_s;
          const double cte_act = cte + v * sin(epsi) * latency_in_s;
          const double epsi_act = epsi + psi_act;

          state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
#else
          state << 0, 0, 0, v, cte, epsi;
#endif
          auto vars = mpc.Solve(state, coeffs);
          steer_value = vars[0];
          throttle_value = vars[1];

          cout << "STV " << steer_value << endl;
          steer_value = steer_value/deg2rad(max_steer_angle);
             cout << "STV " << steer_value << endl;
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 2; i < vars.size(); i ++) {
             if (i%2 == 0) {
               mpc_x_vals.push_back(vars[i]);
             }
             else {
               mpc_y_vals.push_back(vars[i]);
             }
            }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line



//        for (int i = 1; i < num_show_waypoint; i++) {
//             next_x_vals.push_back(show_waypoint_res * i);
//             next_y_vals.push_back(polyeval(coeffs, show_waypoint_res * i));
//         }

        for (int i = 1; i < waypoints_x.size(); i++) {
             next_x_vals.push_back(waypoints_x.at(i));
             next_y_vals.push_back(waypoints_y.at(i));
         }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << "-> " << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          cout << "Latency " << latency_in_s << endl;
          this_thread::sleep_for(chrono::milliseconds(latency_in_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
