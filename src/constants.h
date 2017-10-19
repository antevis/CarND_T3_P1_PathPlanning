//
//  constants.h
//  Path_Planning
//
//  Created by Ivan Kazakov on 10/10/2017.
//

#ifndef constants_h
#define constants_h

const double SPEED_LIMIT_MPH = 49;
const double MS_PER_MPH = 0.44704;
const double SPEED_LIMIT_MS = SPEED_LIMIT_MPH * MS_PER_MPH;
const double HOOD_BUFF = 20;                //Front-end buffer
const double TRUNK_BUFF = 10;               //Tail buffer

const double NOMINAL_LANE_WIDTH = 4;
const double LAT_BUFF = NOMINAL_LANE_WIDTH; // Side buffer

/**
 meters. Sensor fusion horizon seems to be around that value.
 */
const double FAR_AWAY = 250;
const double LOOP_LEN = 6945.554;
const double WP_INCREMENT = 25;

/**
 Executing less steps than computed simplifies handling emergencies.
 */
const int IMPLEMENTATION_STEP_COUNT = 20;

const int PATH_SIZE = 50;
const double SIM_STEP = .02;                // ms

/**
 Deprecated.
 Computed dynamically depending on the velicity difference
 between the Ego and nearest forward 'bee'
 */
const double SAFE_DISTANCE = 30;

/**
 A bit lower than the allowed acceleration (which is 10*.02),
 to simplify handling non-linear motion. More than enough, though
 */
const double MAX_INSTANT_ACC = .15;

/**
 Used to calculate emergency braking.
 Deliberately made those twice as less than the max allowed values
 to have more space for emergency breaking.
 */
const double ACC_LIMIT = 5;
const double JERK_LIMIT = 5;


#endif /* constants_h */
