//
//  types.h
//  Path_Planning
//
//  Created by Ivan Kazakov on 10/10/2017.
//

#ifndef types_h
#define types_h

#include <chrono>
#include <vector>
#include <math.h>
#include "constants.h"

typedef std::vector<double> double1D;
typedef std::vector<double1D> double2D;

/**
 To keep track of the closest bees per lane.
 'Bee' is any other than ego.*/
struct Proxy {
    double distance;
    double velocity;
    
    Proxy(double dist=FAR_AWAY, double v=SPEED_LIMIT_MS) {
        distance = dist;
        velocity = v;
    }
};

struct CartesianPoint {
    double x;
    double y;
};

struct FrenetPoint {
    double s;
    double d;
};

struct Point {
    CartesianPoint cartesian;
    FrenetPoint frenet;
};

struct Velocity {
    double vx;      // m/s
    double vy;      // m/s
};

struct BeeState {
    
    Point point;
    Velocity velocity;
    double yaw;
};

struct BeeSuperState {
    int id;
    
    BeeState global;
    BeeState local;
};

/**
 used to store some important telemetry data.
 Also used for ego's terminal state (at the end of the path)*/
struct State {
    Point point;
    double yaw;         // radians
    double velocity;        // m/s
    double acceleration;    // m/(s^2)
};

/**
 Represents full structure of the ego telemetry data
 */
struct Telemetry {
    
    State state;
    Velocity vxy;
    
    void set_yaw_velocity(double yaw_rad, double v_ms) {
        state.yaw = yaw_rad;
        state.velocity = v_ms;
        
        vxy.vx = state.velocity * cos(state.yaw);
        vxy.vy = state.velocity * sin(state.yaw);
    }
};

struct CartesianPath {
    std::vector<double> xs;
    std::vector<double> ys;
    
    int length() const {
        return xs.size();
    }
};

/**
 used only once, to conveniently represent kinematic data when
 computing levelling with the nearest bee in the current lane*/
struct KinematicTuple {
    double duration;
    double distance;
};


#endif /* types_h */
