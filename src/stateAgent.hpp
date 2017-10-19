//
//  stateAgent.hpp
//  path_planning
//
//  Created by Ivan Kazakov on 17/10/2017.
//

#ifndef stateAgent_hpp
#define stateAgent_hpp

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include "types.h"
#include "utils.h"

class StateAgent{
    
public:
    
    StateAgent();
    
    /**
     'Bee' is any vehicle other than the Ego (on our side of the road).
     'Swarm' is the community of bees.
     BeeSuperState keeps both local and global states for each bee.*/
    std::vector<BeeSuperState> swarm;
    
    /**
     Ego telemetry data*/
    Telemetry * telemetry;
    
    CartesianPath * previousPath;
    
    /**
     Ego state at the end of the previous path*/
    State * egoTerminalState;
    
    /**
     Sensor fusion data.
     Effectively unstructured swarm of bees
     represented by vector of vectors of doubles*/
    double2D * sensorFusion;
    
    /**
     Holds the forward proximity data*/
    std::vector<Proxy> proximityByLane;
    
    void updateState(const FrenetPoint &egoTerminalfPoint, CartesianPath &prevPath, Telemetry &t, double2D &sf);
    
    /**
     Resets forward proximity data*/
    void resetProximity();
    
    /**
     adds sensor fusion record to the swarm as a
     tructured BeeSuperState */
    void addBee(double1D &sensorFusionRecord);
    
    /**
     Paints roadmap to console*/
    void roadMap(const int depthForward, const int depthBackward);
    
    /**
     Paints individual 3-lane road 'slice'*/
    void paintSlice(const double s);
    
    
};

#endif /* stateAgent_hpp */
