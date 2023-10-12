/*
* KobukiNavigationStatechart.c
*
*/

#include "kobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

#define MAX_DIST 				9999
#define MAX_DIST_AVOID 			200
#define TURN_ANGLE 				90
#define HILL_UPPER_THRESHOLD 	0.05 // 0.06
#define HILL_LOWER_THRESHOLD 	0.02 // 0.01
#define DRIVE_SPEED 			225 // 100
#define TURN_SPEED				100 // 75
#define ORIGINAL_ANGLE_THRESH	2
#define REVERSE_DIST			50
#define REVERSE_CLIFF_DIST		100
#define OFFSET_WHEEL_SPEED		30

# define int16_t short
# define int32_t int
# define int64_t long int


// Program States
typedef enum {
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,								// Drive straight
	TURN_LEFT,							// Turn left 90 degrees
	TURN_RIGHT,							// Turn right 90 degrees
	TURN_LEFT_ORIGIN,					// Turn left back to original direction of travel
	TURN_RIGHT_ORIGIN,					// Turn right back to original direction of travel
	REVERSE,							// Reverse after consecutive bumps
	REVERSE_LEFT,
	REVERSE_RIGHT

} robotState_t;

#define DEG_PER_RAD			(180.0 / M_PI)		// degrees per radian
#define RAD_PER_DEG			(M_PI / 180.0)		// radians per degree
void KobukiNavigationStatechart(
	const short 				maxWheelSpeed,
	const int 					netDistance,
	const int 					netAngle,
	const KobukiSensors_t		sensors,
	const accelerometer_t		accelAxes,
	short* const 				pRightWheelSpeed,
	short* const 				pLeftWheelSpeed,
	const bool					isSimulator
) {

	// local state
	static robotState_t 		state = UNPAUSE_WAIT_BUTTON_PRESS;	// current program state
	static robotState_t			unpausedState = DRIVE;				// state history for pause region
	static int					distanceAtManeuverStart = 0;		// distance robot had travelled when a maneuver begins, in mm
	static int					angleAtManeuverStart = 0;			// angle through which the robot had turned when a maneuver begins, in deg
	static int					driveDist = MAX_DIST;
	static int					reorientWheelOffset = 0;

	// inputs
	bool pauseButton =	sensors.buttons.B0;
	bool bumpLeft =		sensors.bumps_wheelDrops.bumpLeft;
	bool bumpRight =	sensors.bumps_wheelDrops.bumpRight;
	bool bumpCenter = 	sensors.bumps_wheelDrops.bumpCenter;
	bool cliffLeft =	sensors.cliffLeft;
	bool cliffCenter =	sensors.cliffCenter;
	bool cliffRight =	sensors.cliffRight;
	double perp_accel, para_accel;							// acceleration of gyroscope perpendicular to direction of travel
	if (isSimulator) {
		perp_accel = accelAxes.x;
		para_accel = accelAxes.y;
	}
	else {
		perp_accel = -accelAxes.y;
		para_accel = accelAxes.x;
	}

	// outputs
	short						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	short						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************

	if (state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| pauseButton						// pause button
		) {
		switch (state) {
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!pauseButton) {
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!pauseButton) {
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (pauseButton) {
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

	else if (state == DRIVE
		|| state == TURN_LEFT
		|| state == TURN_RIGHT
		|| state == TURN_LEFT_ORIGIN
		|| state == TURN_RIGHT_ORIGIN
		|| state == REVERSE
		|| state == REVERSE_LEFT
		|| state == REVERSE_RIGHT
		|| bumpLeft
		|| bumpRight
		|| bumpCenter
		|| cliffLeft
		|| cliffCenter
		|| cliffRight) {
		switch (state) {
		case DRIVE:
			// Remain in this state until a bump, a cliff,
			// the maximum distance has been traversed
			// and we are not travelling in the right direction,
			// a hill is detected,
			// or we need to align with a slope

			// CLIFF!
			if (cliffLeft || cliffCenter || cliffRight) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;

				if (cliffLeft) {
					state = REVERSE_RIGHT;
					driveDist = REVERSE_CLIFF_DIST;
				} else if (cliffRight) {
					state = REVERSE_LEFT;
					driveDist = REVERSE_CLIFF_DIST;
				} else {
					if (netAngle < 0) {
						state = REVERSE_RIGHT;
					}

					else {
						state = REVERSE_LEFT;
					}
					driveDist = REVERSE_DIST;
				}
			}

			// BUMPED!
			else if (bumpLeft
					|| cliffLeft
					|| bumpRight
					|| cliffRight
					|| bumpCenter) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;

				// Reverse for consecutive bumps
				if (driveDist == MAX_DIST_AVOID) {
					state = REVERSE;
					driveDist = REVERSE_DIST;
				}

				// Turn right
				else if (bumpLeft) {
					state = TURN_RIGHT;
					driveDist = MAX_DIST_AVOID;
				}

				// Turn left
				else if (bumpRight) {
					state = TURN_LEFT;
					driveDist = MAX_DIST_AVOID;
				}

				// Bump center turn left
				else if (netAngle < 0) {
					state = TURN_LEFT;
					driveDist = MAX_DIST_AVOID;
				}

				else {
					state = TURN_RIGHT;
					driveDist = MAX_DIST_AVOID;
				}
			}

			// MAX DISTANCE and WRONG DIRECTION!
			else if (abs(netDistance - distanceAtManeuverStart) >= driveDist
				&& abs(netAngle) >= ORIGINAL_ANGLE_THRESH) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;

				// Turn left to correct direction
				if (netAngle < 0) {
					state = TURN_LEFT_ORIGIN;
					driveDist = MAX_DIST;
				}

				// Turn right to correct direction
				else {
					state = TURN_RIGHT_ORIGIN;
					driveDist = MAX_DIST;
				}
			}

			// SLOPE!
			else if (fabs(perp_accel) >= HILL_UPPER_THRESHOLD) {

				// Uphill right
				if (perp_accel > 0 && para_accel > 0) {
					reorientWheelOffset = OFFSET_WHEEL_SPEED;
				}

				// Uphill left
				else if (perp_accel < 0 && para_accel > 0) {
					reorientWheelOffset = -OFFSET_WHEEL_SPEED;
				}

				// Downhill right
				else if (perp_accel > 0 && para_accel < 0) {
					reorientWheelOffset = -OFFSET_WHEEL_SPEED;
				}

				// Downhill left
				else if (perp_accel < 0 && para_accel < 0) {
					reorientWheelOffset = OFFSET_WHEEL_SPEED;
				}
			}

			// NO SLOPE!
			if (fabs(perp_accel) < HILL_LOWER_THRESHOLD) {
				reorientWheelOffset = 0;
			}
			break;

		case TURN_LEFT:
		case TURN_RIGHT:
			// Stay in these states until we have rotated 90 degrees

			if (abs(netAngle - angleAtManeuverStart) >= TURN_ANGLE) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;
				state = DRIVE;
			}
			break;

		case TURN_LEFT_ORIGIN:
		case TURN_RIGHT_ORIGIN:
			// Stay in there states until we are facing the correct direction

			if (abs(netAngle) < ORIGINAL_ANGLE_THRESH) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;
				state = DRIVE;
			}
			break;

		case REVERSE:
			// Stay in this state until we have reversed the required distance

			if (abs(netDistance - distanceAtManeuverStart) >= driveDist) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;

				// Turn left to correct direction
				if (netAngle < 0) {
					state = TURN_LEFT_ORIGIN;
					driveDist = MAX_DIST;
				}

				// Turn right to correct direction
				else {
					state = TURN_RIGHT_ORIGIN;
					driveDist = MAX_DIST;
				}
			}
			break;

		case REVERSE_LEFT:

			if (abs(netDistance - distanceAtManeuverStart) >= driveDist) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;
				state = TURN_LEFT;
				driveDist = MAX_DIST_AVOID;
			}

			break;

		case REVERSE_RIGHT:

			if (abs(netDistance - distanceAtManeuverStart) >= driveDist) {
				angleAtManeuverStart = netAngle;
				distanceAtManeuverStart = netDistance;
				state = TURN_RIGHT;
				driveDist = MAX_DIST_AVOID;
			}

			break;

		default: // state is unknown: reset
			unpausedState = DRIVE;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			driveDist = MAX_DIST;
			break;
		}
	}

	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************

	switch (state) {
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:
		leftWheelSpeed = DRIVE_SPEED + reorientWheelOffset;
		rightWheelSpeed = DRIVE_SPEED - reorientWheelOffset;
		break;

	case TURN_LEFT:
	case TURN_LEFT_ORIGIN:
		rightWheelSpeed = TURN_SPEED;
		leftWheelSpeed = -rightWheelSpeed;
		break;

	case TURN_RIGHT:
	case TURN_RIGHT_ORIGIN:
		leftWheelSpeed = TURN_SPEED;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case REVERSE:
	case REVERSE_LEFT:
	case REVERSE_RIGHT:
		leftWheelSpeed = -DRIVE_SPEED;
		rightWheelSpeed = leftWheelSpeed;
		break;

	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}

	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
