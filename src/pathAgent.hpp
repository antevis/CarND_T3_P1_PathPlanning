//
//  pathAgent.hpp
//  path_planning
//
//  Created by Ivan Kazakov on 17/10/2017.
//

#ifndef pathAgent_hpp
#define pathAgent_hpp

#include <stdio.h>
#include <string>

#include <sstream>  // for std::istringstream
#include <fstream>  // for std::ifstream
#include <iostream> // for std::cout
#include <iomanip>  // for std::setw

#include "spline.h"
#include "stateAgent.hpp"
#include "types.h"

class PathAgent {
public:
    
    PathAgent();
    
    /**
     map data*/
    double1D x_map;
    double1D y_map;
    double1D s_map;
    double1D dx_map;
    double1D dy_map;
    
    /**
     splines data*/
    tk::spline sx_spline;
    tk::spline sy_spline;
    tk::spline sdx_spline;
    tk::spline sdy_spline;
    
    /**
     Essential logic for path creation*/
    CartesianPath followLane(const int currLaneIdx, const int targetLaneIdx, const double refVelocity, StateAgent &sa, bool &safe);
    
    /**
     Prepares waypoint carcass for the new path*/
    std::vector<CartesianPoint> setupFrameWork(StateAgent &sa, const int currLaneIdx, const int targetLaneIdx);
    
    /**
     Cartesian point from Frenet*/
    CartesianPoint pointFrom(const double s, const double d);
    
    tk::spline splineByCartesianPoints(std::vector<CartesianPoint> &pts);
    
    /**
     Borrowed from Aaron Brown's native implementation 'as is', though converting
     variables to constants where possible, not even really delved into the code.
     needed getFrenet() to estimate the Ego Frenet location for collision checks.
     Could be avoided by estimating in Cartesian space, but Frenet seems to
     be more convenient in this case*/
    int ClosestWaypoint(const double x, const double y);
    int NextWaypoint(const double x, const double y, const double theta);
    FrenetPoint getFrenet(const double x, const double y, const double theta);
    
    /**
     Assigns scores to each of 3 lanes. Those been later recursively evaluated
     untill lane deemed to be safe picked.*/
    std::map<int, double> computeProximityScore(const std::vector<Proxy> &proximityByLane, const int currentLane);
    
    /**
     Core logic for lane picking, path generation and execution.*/
    CartesianPath extendPath(StateAgent &stateAgent);
    
    /**
     computes time and distance to level with the forward proxy
     @param v velocity
     @param a initial acceleration
     @param j maximum allowed jerk
     */
    KinematicTuple getBrakingDisance(double v, double a, double j);
};

#endif /* pathAgent_hpp */
