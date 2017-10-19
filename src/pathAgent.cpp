//
//  pathAgent.cpp
//  path_planning
//
//  Created by Ivan Kazakov on 17/10/2017.
//

#include "pathAgent.hpp"

typedef PathAgent PA;

PA::PathAgent() {
    
    std::string mapFilePath = "../data/highway_map.csv";
    
    std::ifstream in_map_(mapFilePath.c_str(), std::ifstream::in);
    
    std::string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        x_map.push_back(x);
        y_map.push_back(y);
        s_map.push_back(s);
        dx_map.push_back(d_x);
        dy_map.push_back(d_y);
    }
    
    /// Smoothing the joint point
    double first_x = x_map.front();
    double first_y = y_map.front();
    double first_dx = dx_map.front();
    double first_dy = dy_map.front();
    
    x_map.push_back(first_x);
    y_map.push_back(first_y);
    dx_map.push_back(first_dx);
    dy_map.push_back(first_dy);
    s_map.push_back(LOOP_LEN);
    
    sx_spline.set_points(s_map, x_map);
    sy_spline.set_points(s_map, y_map);
    sdx_spline.set_points(s_map, dx_map);
    sdy_spline.set_points(s_map, dy_map);
}


CartesianPoint PA::pointFrom(const double s, const double d) {
    
    CartesianPoint point;
    
    const double true_s = effective_s(s);
    const double xs = sx_spline(true_s);
    const double ys = sy_spline(true_s);
    const double dxs = sdx_spline(true_s);
    const double dys = sdy_spline(true_s);
    
    point.x = xs + dxs * d;
    point.y = ys + dys * d;
    
    return point;
}

tk::spline PA::splineByCartesianPoints(std::vector<CartesianPoint> &pts) {
    tk::spline s;
    CartesianPath path = transpose(pts);
    s.set_points(path.xs,path.ys);
    return s;
}

std::vector<CartesianPoint> PA::setupFrameWork(StateAgent &sa, const int currLaneIdx, const int targetLaneIdx) {
    std::vector<CartesianPoint> frameWork;
    
    /// '< 2' scenario may be used in all cases, but might produce
    /// edgy trajectories for certain marginal scenarios,
    /// leading to acceleration limit breaks
    if(sa.previousPath->length() < 2) {
    
        const double prev_car_x = sa.egoTerminalState->point.cartesian.x - cos(sa.egoTerminalState->yaw);
        const double prev_car_y = sa.egoTerminalState->point.cartesian.y - sin(sa.egoTerminalState->yaw);
        
        frameWork.push_back({prev_car_x, prev_car_y});
        frameWork.push_back({sa.egoTerminalState->point.cartesian.x, sa.egoTerminalState->point.cartesian.y});
    
    } else {
        
        const CartesianPoint penUltimatePoint = {back(sa.previousPath->xs,1), back(sa.previousPath->ys, 1)};
        const CartesianPoint ultimatePoint = {back(sa.previousPath->xs,0), back(sa.previousPath->ys, 0)};
        
        frameWork.push_back(penUltimatePoint);
        frameWork.push_back(ultimatePoint);
    }
    
    const int laneShift = targetLaneIdx - currLaneIdx;
    
    CartesianPoint point;
    
    double displacementS = 0;
    
    const double d = fmin(fmax(2 + targetLaneIdx * NOMINAL_LANE_WIDTH, 2.2), 9.8);
    
    /// Adding next waypoint extending forward for WP_EXTENSION ofr 2xWP_EXTENSION
    /// Depending on the value of laneShift
    /// That way, the same rate of change maintained for 1 and 2 lane changing scenarios
    const int wpExtension = fmax(1, 2*abs(laneShift));
    displacementS += WP_INCREMENT * wpExtension;
    point = pointFrom(effective_s(sa.egoTerminalState->point.frenet.s + displacementS), d);
    frameWork.push_back(point);
    
    /// Adding the last point extending small disnance to straighten the curve.
    /// Small is important, as extending further might produce significant
    /// outshoots beyond ther lane boundaries, triggering 'outside of lane' state.
    displacementS += 2;
    point = pointFrom(effective_s(sa.egoTerminalState->point.frenet.s+displacementS), d);
    frameWork.push_back(point);
    
    localFromGlobalPath(frameWork, *sa.egoTerminalState);
    
    return frameWork;
}

int PA::ClosestWaypoint(const double x, const double y)
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    
    for(int i = 0; i < x_map.size(); i++)
    {
        const double map_x = x_map[i];
        const double map_y = y_map[i];
        const double dist = cartesianDistance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    
    return closestWaypoint;
}

int PA::NextWaypoint(const double x, const double y, const double theta)
{
    
    int closestWaypoint = ClosestWaypoint(x,y);
    
    const double map_x = x_map[closestWaypoint];
    const double map_y = y_map[closestWaypoint];
    
    //heading vector
    const double hx = map_x-x;
    const double hy = map_y-y;
    
    //Normal vector:
    const double nx = dx_map[closestWaypoint];
    const double ny = dy_map[closestWaypoint];
    
    //Vector into the direction of the road (perpendicular to the normal vector)
    const double vx = -ny;
    const double vy = nx;
    
    //If the inner product of v and h is positive then we are behind the waypoint so we do not need to
    //increment closestWaypoint, otherwise we are beyond the waypoint and we need to increment closestWaypoint.
    const double inner = hx*vx+hy*vy;
    if (inner<0.0) {
        closestWaypoint++;
    }
    
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetPoint PA::getFrenet(const double x, const double y, const double theta)
{
    const int next_wp = NextWaypoint(x,y, theta);
    
    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = x_map.size()-1;
    }
    
    const double n_x = x_map[next_wp]-x_map[prev_wp];
    const double n_y = y_map[next_wp]-y_map[prev_wp];
    const double x_x = x - x_map[prev_wp];
    const double x_y = y - y_map[prev_wp];
    
    // find the projection of x onto n
    const double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    const double proj_x = proj_norm*n_x;
    const double proj_y = proj_norm*n_y;
    
    double frenet_d = cartesianDistance(x_x,x_y,proj_x,proj_y);
    
    //see if d value is positive or negative by comparing it to a center point
    const double center_x = 1000-x_map[prev_wp];
    const double center_y = 2000-y_map[prev_wp];
    const double centerToPos = cartesianDistance(center_x,center_y,x_x,x_y);
    const double centerToRef = cartesianDistance(center_x,center_y,proj_x,proj_y);
    
    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    
    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += cartesianDistance(x_map[i],y_map[i],x_map[i+1],y_map[i+1]);
    }
    
    frenet_s += cartesianDistance(0,0,proj_x,proj_y);
    
    return {frenet_s,frenet_d};
    
}

CartesianPath PA::followLane(const int currLaneIdx, const int targetLaneIdx, const double refVelocity, StateAgent &sa, bool &safe) {
    
    std::vector<CartesianPoint> frameWork = setupFrameWork(sa, currLaneIdx, targetLaneIdx);
    
    const tk::spline s = splineByCartesianPoints(frameWork);
    
    CartesianPath path;
    
    const int extesionLength = PATH_SIZE - sa.previousPath->length();
    
    const double targetLocalX = WP_INCREMENT;
    const double targetLocalY = s(targetLocalX);
    const double targetDistance = cartesianDistance(targetLocalX, targetLocalY);
    
    double currentLocalX = 0;
    
    bool collision = false;
    
    double currentEgoVelocity = sa.egoTerminalState->velocity;
    
    for(int i = 0; i < extesionLength; ++i) {
        
        if(refVelocity > currentEgoVelocity) {
            
            currentEgoVelocity+= MAX_INSTANT_ACC;
            currentEgoVelocity = fmin(currentEgoVelocity, SPEED_LIMIT_MS);
            
        } else if(refVelocity < currentEgoVelocity) {
            
            currentEgoVelocity-= MAX_INSTANT_ACC;
            currentEgoVelocity = fmax(currentEgoVelocity, 0);
        }
        
        currentLocalX += targetLocalX / (targetDistance / (SIM_STEP * currentEgoVelocity));
        const CartesianPoint point = {currentLocalX, s(point.x)};
        
        const double localYaw = path.length() > 0 ? atan2(point.y - back(path.ys), point.x - back(path.xs)) : 0;
        
        const CartesianPoint global = globalFrom(point, sa.egoTerminalState->yaw, sa.egoTerminalState->point.cartesian);
        
        /// No sense to check the rest of the path for collision once it's been detected.
        /// Though the path still being computed fully.
        if(!collision) {
        
            const FrenetPoint egoCurrentFrenet = getFrenet(global.x, global.y, sa.egoTerminalState->yaw + localYaw);
            
            for(std::vector<double> &bee: *sa.sensorFusion){
                const double beeVx = bee[3];
                const double beeVy = bee[4];
                const double beeV = cartesianDistance(beeVx, beeVy);
                
                const double startBeeS = effective_s(bee[5]);
                const double currentBeeS = effective_s(startBeeS + double(sa.previousPath->length() + i) * SIM_STEP * beeV);
                const double currentBeeD = bee[6];
                
                const double distS = currentBeeS - effective_s(egoCurrentFrenet.s);
                const double distD = currentBeeD - egoCurrentFrenet.d;
                
                /// 3/5 seems to be reliable enough as a collision buffer ratio
                if (distS < HOOD_BUFF * 3./ 5. && distS > -TRUNK_BUFF) {
                    
                    if (distD < LAT_BUFF && distD > -LAT_BUFF) {
                    
                        collision = true;
                    }
                }
                if (collision) {
                    /// No sense to check the rest of the 'swarm'
                    break;
                }
            }
        }
        
        /// As mentioned before, the path is being computed fully, regrardless of
        /// whether it leads to collision or not.
        path.xs.push_back(global.x);
        path.ys.push_back(global.y);
    }
    
    safe = !collision;
    
    return path;
}

std::map<int, double> PA::computeProximityScore(const std::vector<Proxy> &proximityByLane, const int currentLane) {
    std::map<int, double> distanceScoreByLane;
    
    for (int i = 0; i < proximityByLane.size(); ++i) {
        
        /** Core formula to estimate the 'attractiveness' of each lane
         depending on it free forward space and remoteness from the current lane.
         '+ i' to slightly favor rightmost lane all else being equal.
         Weight of 1.0 for free forward distance works reliable enough.
         Encreasing above 1.0 makes the Ego behavior less reluctant to lane changing.*/
        const double score = 1.0 * proximityByLane[i].distance / (abs(currentLane - i) + 1) + i;
        
        distanceScoreByLane[i] = score;
    }
    return distanceScoreByLane;
}

KinematicTuple PA::getBrakingDisance(double v, double a, double j=JERK_LIMIT) { // velocity, deceleration, jerk
    
    /**
     x = x0 + v0t + 1/2a0t^2 + 1/6jt^3
     v = v0 + a0t + 1/2jt^2
     a = a0 + jt
     */
    
    double x = 0;
    double t = SIM_STEP;
    int counter = 0;
    
    while (v > 0) {
        x += v * t + a * pow(t, 2) / 2. - j * pow(t, 3) / 6.;
        v += a * t - j * pow(t, 2) / 2.;
        v = fmax(v, 0);
        a -= j * t;
        a = fmax(-ACC_LIMIT, a);
        j = v > 0 ? JERK_LIMIT : 0;
        
        counter++;
    }
    
    double duration = double(counter) * t;
    
    return {x, duration};
}

CartesianPath PA::extendPath(StateAgent &stateAgent) {
    CartesianPath newPath;
    
    const int currLaneIdx = lane_by(stateAgent.telemetry->state.point.frenet.d);
    
    std::map<int, double> proximityScoreByLane = computeProximityScore(stateAgent.proximityByLane, currLaneIdx);
    
    /// Not being used in case of no lane change.
    bool safe = false;
    
    const Proxy proxy = stateAgent.proximityByLane[currLaneIdx];
    
    double refVel;
    
    /// Velocity difference between the Ego and forward proxy at the end of
    /// the previous path
    const double deltaVterminal = stateAgent.egoTerminalState->velocity - proxy.velocity;
    
    /// Current and terminal (at the end of the previous path) s-coordinate
    /// of the proxy
    const double proxyCurrentS = stateAgent.telemetry->state.point.frenet.s + proxy.distance;
    const double proxyTerminalS = proxyCurrentS + proxy.velocity * stateAgent.previousPath->length() * SIM_STEP;
    
    const double terminalProximity = proxyTerminalS - stateAgent.egoTerminalState->point.frenet.s;
    
    /// What it takes to level with the proxy in terms of time and distance
    /// As the velocity difference doesn't generally gets large,
    /// the levelling values are generally small
    const KinematicTuple levelling = getBrakingDisance(deltaVterminal, stateAgent.egoTerminalState->acceleration);
    
    /// Adjusting the velocity.
    /// Probably the most controversial part of all the pipeline.
    if (terminalProximity < HOOD_BUFF) {
        refVel = proxy.velocity - 5 * MS_PER_MPH;
    } else {
    
        for(int i = 0; i >= 4; ++i) {
            if (terminalProximity > i * levelling.distance + HOOD_BUFF){
                refVel = proxy.velocity - (4-i) * MS_PER_MPH;
            }
        }
    }
    if (terminalProximity > 5 * levelling.distance + HOOD_BUFF) { refVel = SPEED_LIMIT_MS; }

    
    /// Recursive picking of best lane while no safe lane found
    /// or no lane change assumed.
    int proposedLaneIdx;
    do {
        proposedLaneIdx = get_max(proximityScoreByLane).first;
        
        newPath = followLane(currLaneIdx, proposedLaneIdx, refVel, stateAgent, safe);
        
        if (safe){
            
            break;
        } else {
            proximityScoreByLane.erase(proposedLaneIdx);
        }
        
    } while (proposedLaneIdx != currLaneIdx);
    

    return newPath;
}
