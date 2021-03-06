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

// for convenience
using json = nlohmann::json;

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

const double Lf = 2.67;
const int time_latency = 100;

int main() {
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
                    
                    v = v * 1600.0/3600.0;
                    //++
                    double delta_t = j[1]["steering_angle"];
                    double a_t = j[1]["throttle"];
                    
                    // coordinates transformation between map and car: shift and rotate
                    
                    for (int i = 0; i< ptsx.size(); i++){
                        //apply translation to (px, py)
                        double shift_x = ptsx[i] - px;
                        double shift_y = ptsy[i] - py;
                        
                        //apply rotatation with -psi angle
                        ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
                        ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
                    }
                    
                    double* pointer_x = &ptsx[0];
                    double* pointer_y = &ptsy[0];
                    Eigen::Map<Eigen::VectorXd> ptsx_fit(pointer_x, 6);
                    Eigen::Map<Eigen::VectorXd> ptsy_fit(pointer_y, 6);
                    
                    auto coeffs = polyfit(ptsx_fit, ptsy_fit, 3);
                    // in the car coordinate px = 0 py = 0, psi = 0
                    
                    //Compute CTE
                    double cte = polyeval(coeffs, 0); // polyeval(coeffs, px) - py
                    
                    //COMPUTE epsi
                    double epsi = - atan(coeffs[1]);  // psi - atan(coeffs[1]) + 2*px*coeffs[2]+3*px*px*coeff[3]
                    
                    //  Dealing with Latency
                    // As the lecture 20.7 suggests
                    //One approach would be running a simulation using the vehicle model starting from the current state
                    //for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.
                    
                    
                    //Using the model defined in lecture 19.4
                    double x0 = 0.0;
                    double y0 = 0.0;
                    double psi0 = 0.0;
                    double v0 = v;
                    
                    if (time_latency > 0){
                        double dt = time_latency / 1000.0; // latency in seconds
                        // In the car coordinates psi = 0
                        
                        x0 = v * dt;
                        
                        psi0 =  - (v/Lf) * delta_t * dt ;
                        v0 = v + a_t * dt;
                        
                        //Cross Track Error, lecture 19.10
                        cte += v * sin(epsi) * dt;
                        
                        //Orientation error, lecture 19.10
                        epsi -= (v/Lf) * delta_t * dt ;
                    }
                    
                    Eigen::VectorXd state(6);
                    state << x0, y0, psi0, v0, cte, epsi;
                    auto result_var = mpc.Solve(state, coeffs);
                    double steer_value = result_var[0];
                    double throttle_value = result_var[1];
                    
                    
                    //++
                    
                    /*
                     * TODO: Calculate steering angle and throttle using MPC.
                     *
                     * Both are in between [-1, 1].
                     *
                     */
                    //double steer_value;
                    //double throttle_value;
                    
                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = -steer_value / deg2rad(25);
                    msgJson["throttle"] = throttle_value;
                    
                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    
                    for(int i = 2; i < result_var.size();i += 2){
                        mpc_x_vals.push_back(result_var[i]);
                        mpc_y_vals.push_back(result_var[i+1]);
                    }
                    
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;
                    
                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    
                    int nums_points = 30;
                    double poly_incr = 2;
                    for(int i = 1; i< nums_points; i++){
                        next_x_vals.push_back(poly_incr * i);
                        next_y_vals.push_back(polyeval(coeffs, poly_incr * i));
                    }
                    
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
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
