//
//  utils.h
//  Path_Planning
//
//  Created by Ivan Kazakov on 10/10/2017.
//

#ifndef utils_h
#define utils_h

#include <map>

#include "constants.h"
#include "spline.h"
#include "types.h"

inline int lane_by(const double d) {
    return int(d / NOMINAL_LANE_WIDTH);
}

inline int min_idx(const std::vector<double> &v) {
    return std::distance(v.begin(), std::min_element(v.begin(), v.end()));
}

inline int max_idx(const std::vector<double> &v) {
    return std::distance(v.begin(), std::max_element(v.begin(), v.end()));
}

inline double min_value(const std::vector<double> v) {
    
    assert(v.size() > 0);
    return v[min_idx(v)];
}

inline double max_value(const std::vector<double> v) {
    
    assert(v.size() > 0);
    return v[max_idx(v)];
}

inline CartesianPoint localFrom(const CartesianPoint & global, const double & refYaw, const CartesianPoint & ref) {
    
    const double shiftX = global.x - ref.x;
    const double shiftY = global.y - ref.y;
    const double localX = shiftX * cos(-refYaw) - shiftY * sin(-refYaw);
    const double localY = shiftX * sin(-refYaw) + shiftY * cos(-refYaw);
    
    return {localX, localY};
}

inline CartesianPoint globalFrom(const CartesianPoint & local, const double & refYaw, const CartesianPoint & ref) {
    
    const double globalX = local.x * cos(refYaw) - local.y * sin(refYaw) + ref.x;
    const double globalY = local.x * sin(refYaw) + local.y * cos(refYaw) + ref.y;
    
    return {globalX, globalY};
}

/**
 To easily access elements from the back of the vector.
 @param vec - the vector which elements being accessed
 @param n - number of the required element (0 = last)
 */
template <typename T>
inline T back(const std::vector<T> &vec, const int n=0) {
    
    assert (vec.size() > n);
    return vec[vec.size()-(n+1)];
}

inline void localFromGlobalPath(std::vector<CartesianPoint> &pts, const CartesianPoint &egoXY, const double ref_yaw) {
    for (CartesianPoint &p: pts) {
        p = localFrom(p, ref_yaw, egoXY);
    }
}

inline void localFromGlobalPath(std::vector<CartesianPoint> &pts, const State &state) {
    for (CartesianPoint &p: pts) {
        p = localFrom(p, state.yaw, state.point.cartesian);
    }
}

/**
 Transforms vector of cartesian points : {{x1, y1},{x2, y2},{x3, y3}...}
 to a cartesian path: {{x1, x2, x3...},{y1, y2, y3...}}
 
 There's probably a more efficient way to do this - maybe elaborate later
 */
inline CartesianPath transpose(std::vector<CartesianPoint> &pts) {
    
    CartesianPath result;
    
    for(CartesianPoint &p: pts) {
        result.xs.push_back(p.x);
        result.ys.push_back(p.y);
    }
    
    return result;
}

// https://stackoverflow.com/questions/9370945/c-help-finding-the-max-value-in-a-map
template<typename KeyType, typename ValueType>
std::pair<KeyType,ValueType> get_max( const std::map<KeyType,ValueType>& x ) {
    using pairtype=std::pair<KeyType,ValueType>;
    return *std::max_element(x.begin(), x.end(), [] (const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
    });
}

/**
 workaroud for looping*/
inline double effective_s(const double s) {
    return fmod(s, LOOP_LEN);
}

inline double radFrom(const double deg) { return deg * M_PI / 180; }
inline double degFrom(const double rad) { return rad * 180 / M_PI; }

inline double cartesianDistance(const double x1, const double y1, const double x2=0, const double y2=0)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

#endif /* utils_h */
