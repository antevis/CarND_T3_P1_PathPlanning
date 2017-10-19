//
//  stateAgent.cpp
//  path_planning
//
//  Created by Ivan Kazakov on 17/10/2017.
//

#include "stateAgent.hpp"

typedef StateAgent SA;

void SA::resetProximity() {
    proximityByLane = std::vector<Proxy>(3);
}

SA::StateAgent() {
    resetProximity();
    egoTerminalState = new State();
}

void SA::updateState(const FrenetPoint &egoTerminalfPoint, CartesianPath &prevPath, Telemetry &t, double2D &sf) {
    
    resetProximity();
    swarm.clear();
    telemetry = &t;
    previousPath = &prevPath;
    
    sensorFusion = &sf;

    /** at least 3 points required to derive the terminal point acceleration,
     which in turn used to calculate braking distance later*/
    if(prevPath.length() >= 3) {
        const CartesianPoint prePenUltEgoXy =   {back(prevPath.xs, 2), back(prevPath.ys, 2)};
        const CartesianPoint penultEgoXY =      {back(prevPath.xs, 1), back(prevPath.ys, 1)};
        
        egoTerminalState->point.frenet.s = egoTerminalfPoint.s;
        egoTerminalState->point.frenet.d = egoTerminalfPoint.d;
        
        egoTerminalState->point.cartesian.x = back(prevPath.xs, 0); // alternative to '.back()'
        egoTerminalState->point.cartesian.y = back(prevPath.ys, 0); // alternative to '.back()'
        egoTerminalState->yaw = atan2(egoTerminalState->point.cartesian.y-penultEgoXY.y, egoTerminalState->point.cartesian.x-penultEgoXY.x);
        
        egoTerminalState->velocity = cartesianDistance(penultEgoXY.x, penultEgoXY.y,
                                                       egoTerminalState->point.cartesian.x,
                                                       egoTerminalState->point.cartesian.y)/ SIM_STEP;
        const double penUltimateVelocity = cartesianDistance(penultEgoXY.x, penultEgoXY.y, prePenUltEgoXy.x, prePenUltEgoXy.y)/ SIM_STEP;
        
        egoTerminalState->acceleration = (egoTerminalState->velocity - penUltimateVelocity) * SIM_STEP;
        
    } else {
        *egoTerminalState = telemetry->state;
    }
    
    /**
     Updating proximityByLane
     could be moved to a dedicated method*/
    for(std::vector<double> &bee: *sensorFusion)
    {
        /// Piggybacking the loop through sensor fusion to update the 'Swarm'
        addBee(bee);
        
        const float beeD = bee[6];
        const int beeLaneIdx = lane_by(beeD);
        const bool validBee = (beeD >= 0 && beeD <= 12);
        const double beeVelocity = cartesianDistance(bee[3], bee[4]);
        const double currentBeeS = effective_s(bee[5]);
        const double currentBeeForwardDistance =  currentBeeS - telemetry->state.point.frenet.s;
        
        if (validBee && currentBeeForwardDistance > 0 &&
            currentBeeForwardDistance < proximityByLane[beeLaneIdx].distance) {
            
            const Proxy newProxy(currentBeeForwardDistance, beeVelocity);
            proximityByLane[beeLaneIdx] = newProxy;
        }
    }
}

void SA::addBee(double1D &record) {
    
    const CartesianPoint local = globalFrom(CartesianPoint({record[1], record[2]}),
                                            telemetry->state.yaw, telemetry->state.point.cartesian);
    const double vx = record[3];
    const double vy = record[4];
    
    /// Representing velocity as point.
    /// Because I can, that's why :p
    const CartesianPoint vxy_local = globalFrom(CartesianPoint({vx, vy}), telemetry->state.yaw, CartesianPoint({0, 0}));
    
    const double yaw = atan2(vy, vx);
    const double yaw_local = yaw - telemetry->state.yaw;
    
    const double s_global = effective_s(record[5]);
    const double d_global = record[6];
    
    const double s_local = telemetry->state.point.frenet.s - s_global;
    const double d_local = telemetry->state.point.frenet.d - d_global;
    
    BeeSuperState state;
    
    state.id = int(record[0]);
    
    // Global
    state.global.point.cartesian.x = record[1];
    state.global.point.cartesian.y = record[2];
    state.global.point.frenet.s = s_global;
    state.global.point.frenet.d = d_global;
    state.global.velocity.vx = vx;
    state.global.velocity.vy = vy;
    state.global.yaw = yaw;
    
    // Local to the Ego
    state.local.point.cartesian.x = local.x;
    state.local.point.cartesian.y = local.y;
    state.local.point.frenet.s = s_local;
    state.local.point.frenet.d = d_local;
    state.local.velocity.vx = vxy_local.x - telemetry->vxy.vx;
    state.local.velocity.vy = vxy_local.y - telemetry->vxy.vy;
    state.local.yaw = yaw_local;
    
    swarm.push_back(state);
}


void SA::roadMap(const int depthForward, const int depthBackward) {
    const int step = 5; //meters
    
    for (int i = -depthForward; i <= depthBackward; i+=step) {
        paintSlice(i);
    }
    
    std::cout << std::endl;
}

void SA::paintSlice(const double s) {
    
    for(int i = 0; i < 3; ++i) {
        
        std::cout << "|";
        bool occupied = false;
        
        for (const BeeSuperState &bee: swarm) {
            
            const int beeLane = lane_by(bee.global.point.frenet.d);
            
            if (beeLane == i && bee.local.point.frenet.s > s-2.5 && bee.local.point.frenet.s < s+2.5) {
                occupied = true;
                std::cout << std::setw(2) << bee.id;
                break;
            }
        }
        
        if (!occupied) {
            if (s == 0 && lane_by(telemetry->state.point.frenet.d) == i) {
                std::cout << "**";
            } else {
                std::cout << "  ";
            }
            
        }
    }
    std::cout << "|" << std::endl;
}
