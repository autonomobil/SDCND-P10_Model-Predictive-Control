#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <iomanip>      // std::setw

std::fstream parameter_data;

double real_a = 0;
// for convenience
using json = nlohmann::json;
using namespace std;
using namespace Eigen;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

size_t no_parameters = 9;
double max_speed_reached = 0;

void reset_simulator(uWS::WebSocket<uWS::SERVER> ws){
  // reset
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

VectorXd read_parameter(size_t no_parameters){

  // N, dt, ref_v, factorCTE, factorErrorPsi, factorErrorV, factor_d_Delta, factor_d_Thrust
  VectorXd parameter(no_parameters);
  string parameter_str[no_parameters];

  parameter_data.open("../parameter.dat", ios::in);

  for (size_t i = 0; i < no_parameters; i++) {
    parameter_data >> parameter_str[i];
    parameter(i) = std::stod(parameter_str[i]);
  }

  cout << "Parameter loaded"<< endl;
  cout << parameter << endl;

  parameter_data.close();

  return parameter;
}

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

int main() {
  uWS::Hub h;
  // MPC is initialized here!
  MPC mpc;

  VectorXd parameter = read_parameter(no_parameters);

  mpc.parameter = parameter;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> targetPoints_x = j[1]["ptsx"];
          vector<double> targetPoints_y = j[1]["ptsy"];
          double posGlob_x = j[1]["x"];     // position x
          double posGlob_y = j[1]["y"];     // position y
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          // v = v  * 2.23694; //mph -> m/s
          double delta = j[1]["steering_angle"];     // delta = steering angle
          double throttle_value  = j[1]["throttle"];

          size_t N = targetPoints_x.size();     // Number of waypoints, timestep based
          double cos_psi = cos(-psi);
          double sin_psi = sin(-psi);
          const double delay = 0.1;     // delay in seconds
          const size_t delay_ms = 100;     // delay in milliseconds and interger

          // convert from global to vehicle coordinate system
          VectorXd targetPointsCar_x(N);
          VectorXd targetPointsCar_y(N);
          for(size_t t = 0; t < N; t++) {
            double dx = targetPoints_x[t] - posGlob_x;
            double dy = targetPoints_y[t] - posGlob_y;
            targetPointsCar_x[t] = dx * cos_psi - dy * sin_psi;
            targetPointsCar_y[t] = dx * sin_psi + dy * cos_psi;
          }

          // fit 3rd order polynomial to points
          auto coeffs = polyfit(targetPointsCar_x, targetPointsCar_y, 3);

          // calculate cte & err_psi for state with delay
          VectorXd state(6);
          double cte = coeffs[0];
          double err_psi = -atan(coeffs[1]);

          double x_predict = 0.0 + v * delay;     // x = 0 + v * delay * cos(0) <- cos(0) = 1
          double y_predict = 0.0;         // y = 0 + v * delay * sin(0) <- sin(0) = 0
          double psi_predict = 0.0 - (v * delta * delay / mpc.Lf);
          double v_predict = v + throttle_value * delay;     // this is a bit inaccurate, because a ~= throttle_value because of mass
          double cte_predict = cte + (v * sin(err_psi) * delay);
          double err_psi_predict = err_psi - (v * err_psi * delay / mpc.Lf);

          if (cte > 5 || cte < -5) {
            reset_simulator(ws);

            std::cout << "---> Controller terminated because car left track, please restart with ./mpc" << std::endl;
            std::terminate();

          }
          else{
            // state << x_predict, y_predict, psi_predict, v_predict, cte_predict, err_psi_predict;
            state << x_predict, y_predict, psi_predict, v_predict, cte_predict, err_psi_predict;

            // let the solver do its magic
            auto mpcResult = mpc.Solve(state, coeffs);

            // Calculate steering angle and throttle using MPC
            delta = mpcResult[0] / deg2rad(25);     // convert to [-1..1] range
            throttle_value = mpcResult[1];

            // std::cout << '\n' << mpcResult[0] << '\t' << mpcResult[1] << '\n';

            json msgJson;
            msgJson["steering_angle"] = delta;
            msgJson["throttle"] = throttle_value;

            //Display the MPC predicted trajectory
            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Green line
            vector<double> mpc_x_vals = {state[0]};
            vector<double> mpc_y_vals = {state[1]};

            for (size_t i = 2; i < mpcResult.size(); i+=2 ) {
              mpc_x_vals.push_back(mpcResult[i]);
              mpc_y_vals.push_back(mpcResult[i+1]);
            }

            msgJson["mpc_x"] = mpc_x_vals;
            msgJson["mpc_y"] = mpc_y_vals;

            //Display the waypoints/reference line
            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Yellow line
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for (size_t i = 0; i < N; i++ ) {
              next_x_vals.push_back(targetPointsCar_x[i]);
              next_y_vals.push_back(targetPointsCar_y[i]);
            }

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;

            if(v > max_speed_reached) {
              max_speed_reached = v;
            }

            std::cout << '\t' <<  "cte: "  << std::setw(8) << cte  << " | "  << std::setw(8) << cte_predict;
            std::cout << '\t'  << "psi: "  << std::setw(8) << psi  << " | "  << std::setw(8) << rad2deg(psi_predict);
            std::cout << '\t'  << "v: "  << std::setw(8) << v << " | "  << std::setw(8) << v_predict;
            std::cout << '\t' << "throttle: "  << std::setw(8) << throttle_value << " | delta: " << std::setw(8) << delta<< '\n';
            // std::cout << "max speed: " << max_speed_reached << std::endl;


            // Latency
            // The purpose is to mimic real driving conditions where
            // the car does not actuate the commands instantly.
            this_thread::sleep_for(chrono::milliseconds(delay_ms));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
