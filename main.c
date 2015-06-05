/*
USE : Position Robot in the middle of the starting square

knows left wall follow will finish, so take advantage of that
*/

#include <stdio.h>
#include "picomms.h"
#include <math.h>
#include "position.h"
#include "phaseOne.h"
#include "phaseTwo.h"

#define INMIDDLEOFSQUARE
#define STRAIGHT
#define CHECKWALLS
#define TURN
#define STOPPED 0

int state = STOPPED;

int main()
{
	connect_to_robot();
	initialize_robot();
	set_ir_angle(LEFT, -45); set_ir_angle(RIGHT, 45);
	setSquareCenters();
	initialBoxCalibration();
	while(!state)
	{
		state = checkWalls();
		calcPos();		
	}
	racePace();
	return 1;
}