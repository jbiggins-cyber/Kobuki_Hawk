/*
* KobukiNavigationStatechart.c
*
*/

#include "kobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

#define MAX_DIST 100000
#define MAX_DIST_AVOID 130
#define TURN_ANGLE 90
#define HILL_THRESHOLD 0.06
#define REORIENT_THRESHOLD 0.01
#define WHEEL_SPEED 400
#define REORIENT_WHEEL_SPEED 25
#define NORMAL_ANGLE 45


// Program States
typedef enum{
	INITIAL = 0,						// Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,								// Drive straight
	TURN_LEFT,							/// Turn
	TURN_RIGHT,
	//DRIVE_AVOID,
	CENTER_RIGHT,
	CENTER_LEFT,
	REORIENT_LEFT,
	REORIENT_RIGHT
} robotState_t;

#define DEG_PER_RAD			(180.0 / M_PI)		// degrees per radian
#define RAD_PER_DEG			(M_PI / 180.0)		// radians per degree

void KobukiNavigationStatechart(
	const int 				    maxWheelSpeed,
	const int 					netDistance,
	const int 					netAngle,
	const KobukiSensors_t		sensors,
	const accelerometer_t		accelAxes,
	short * const 				pRightWheelSpeed,
	short * const 				pLeftWheelSpeed,
	const bool					isSimulator
	){

	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	//static robotState_t			postCollisionState = TURN_LEFT;	// state post-collision
	//static int					distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int					angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg
	static bool					inArc = false;
	static int					driveDist = MAX_DIST;

	// outputs
	short						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	short						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s
	double						accel = 0;
	double						accelx = 0;
	double						accely = 0;

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************

	if (isSimulator) {
		accel = accelAxes.x;
	}
	else {
		accel = -accelAxes.y;
	}

	if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0				// pause button
		){
		switch (state){
		case INITIAL:
			// set state data that may change between simulation and real-world
			if (isSimulator){
			}
			else{
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!sensors.buttons.B0){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// state transition - run region      *
	//*************************************

	else if (state == DRIVE &&
			(sensors.cliffCenter
			|| sensors.cliffLeft
			|| sensors.cliffRight)) {
		state = UNPAUSE_WAIT_BUTTON_PRESS;
	}

	else if (state == DRIVE && fabs(accel) >= HILL_THRESHOLD) {

		if (accel > 0) {
			state = REORIENT_RIGHT;
		}
		else {
			state = REORIENT_LEFT;
		}

	}

	else if ((state == REORIENT_LEFT || state == REORIENT_RIGHT)

		&& fabs(accel) < REORIENT_THRESHOLD) {
		state = DRIVE;
	}

	else if (state == DRIVE &&
		//(abs(netDistance - distanceAtManeuverStart) >= driveDist 
		(
		    sensors.bumps_wheelDrops.bumpLeft
			|| sensors.bumps_wheelDrops.bumpCenter
			|| sensors.bumps_wheelDrops.bumpRight
			)) {
		angleAtManeuverStart = netAngle;
		//distanceAtManeuverStart = netDistance;
		inArc = true;
		if (sensors.bumps_wheelDrops.bumpRight)
		{
			state = TURN_LEFT;
			
		}
		else if (sensors.bumps_wheelDrops.bumpLeft)
		{

			state = TURN_RIGHT;
		}
		else if (sensors.bumps_wheelDrops.bumpCenter) {

			if (netAngle > 0) {

				state = CENTER_LEFT;
			}
			else {
				
				state = CENTER_RIGHT;
			}
		}
	} 

	if (state == CENTER_LEFT && abs(netAngle - angleAtManeuverStart) >= NORMAL_ANGLE) {
		
		angleAtManeuverStart = netAngle;
		state = TURN_RIGHT;
		inArc = true;
	} else if (state == CENTER_RIGHT && abs(netAngle - angleAtManeuverStart) >= NORMAL_ANGLE) {

		angleAtManeuverStart = netAngle;
		state = TURN_LEFT;
		inArc = true;
	}

	if (state == TURN_LEFT && abs(netAngle - angleAtManeuverStart) >= TURN_ANGLE){
		
		angleAtManeuverStart = netAngle;

		if (inArc) {

			state = TURN_RIGHT;
			inArc = false;
		}
		else {
			state = DRIVE;
		}
	}

	if (state == TURN_RIGHT && abs(netAngle - angleAtManeuverStart) >= TURN_ANGLE) {

		angleAtManeuverStart = netAngle;
		//distanceAtManeuverStart = netDistance;
		if (inArc) {

			state = TURN_LEFT;
			inArc = false;
		}
		else {
			state = DRIVE;
		}
	}

	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	switch (state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:

		leftWheelSpeed = WHEEL_SPEED;
		rightWheelSpeed = leftWheelSpeed;
		break;
	case CENTER_RIGHT:

		leftWheelSpeed = WHEEL_SPEED;
		rightWheelSpeed = - leftWheelSpeed;
		break;
	case CENTER_LEFT:
		rightWheelSpeed = WHEEL_SPEED;
		leftWheelSpeed = - rightWheelSpeed;
		break;

	case TURN_RIGHT:
		leftWheelSpeed = (short) 1.5 * WHEEL_SPEED;
		rightWheelSpeed = (short) 0.5 * leftWheelSpeed;
		break;

	case TURN_LEFT:
		rightWheelSpeed = (short) 1.5 * WHEEL_SPEED;
		leftWheelSpeed = (short) 0.8 * rightWheelSpeed;
		break;

	case REORIENT_RIGHT:
		leftWheelSpeed = REORIENT_WHEEL_SPEED;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case REORIENT_LEFT:
		rightWheelSpeed = REORIENT_WHEEL_SPEED;
		leftWheelSpeed = -rightWheelSpeed;
		break;

	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}

	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
