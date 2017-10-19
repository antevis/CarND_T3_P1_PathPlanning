#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//#include "spline.h"
//#include "constants.h"
#include "utils.h"
//#include "types.h"
#include "pathAgent.hpp"
#include "stateAgent.hpp"

using namespace std;

typedef std::vector< std::vector<double> > double2D;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


int main() {
    uWS::Hub h;
    
    PathAgent pathAgent = PathAgent();
    StateAgent stateAgent;
    
    h.onMessage([&pathAgent, &stateAgent](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        
        /**
         Main car's localization Data (No Noise)
         ["x"] The car's x position in map coordinates
         ["y"] The car's y position in map coordinates
         ["s"] The car's s position in frenet coordinates
         ["d"] The car's d position in frenet coordinates
         ["yaw"] The car's yaw angle in the map
         ["speed"] The car's speed in MPH
         
         ["sensor_fusion"] A 2d vector of cars and then that car's
         [car's unique ID,
         car's x position in map coordinates,
         car's y position in map coordinates,
         car's x velocity in m/s,
         car's y velocity in m/s,
         car's s position in frenet coordinates,
         car's d position in frenet coordinates.
         */
        
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            
            auto s = hasData(data);
            
            if (s != "") {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> previous_path_x = j[1]["previous_path_x"];
                    vector<double> previous_path_y = j[1]["previous_path_y"];
                    
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    for(int i = 0; i < previous_path_x.size(); i++)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    
                    double end_path_s = j[1]["end_path_s"];
                    end_path_s = effective_s(end_path_s);
                    
                    double end_path_d = j[1]["end_path_d"];
                    double2D sensor_fusion = j[1]["sensor_fusion"];
                    
                    CartesianPath previousPath = {previous_path_x, previous_path_y};
                    FrenetPoint terminalFrenet = {end_path_s, end_path_d};
                    Telemetry t;
                    
                    t.state.point.cartesian.x = j[1]["x"];
                    t.state.point.cartesian.y = j[1]["y"];
                    t.state.point.frenet.s = effective_s(j[1]["s"]);
                    t.state.point.frenet.d = j[1]["d"];
                    
                    double yaw = radFrom(j[1]["yaw"]);
                    double mph = j[1]["speed"];
                    double ms = mph * MS_PER_MPH;
                    t.set_yaw_velocity(yaw, ms); // also sets cartesian velocity
                    
                    stateAgent.updateState(terminalFrenet, previousPath, t, sensor_fusion);
                    
                    stateAgent.roadMap(225, 25);
                    
                    CartesianPath newPath = pathAgent.extendPath(stateAgent);

                    int newPointsCount = IMPLEMENTATION_STEP_COUNT - previousPath.length();
                    
                    next_x_vals.insert(next_x_vals.end(), newPath.xs.begin(), newPath.xs.begin()+newPointsCount);
                    next_y_vals.insert(next_y_vals.end(), newPath.ys.begin(), newPath.ys.begin()+newPointsCount);
                    
                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    auto msg = "42[\"control\","+ msgJson.dump()+"]";
                    
                    //this_thread::sleep_for(chrono::milliseconds(1000));
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

