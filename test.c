/*
USE : Position Robot in the middle of the starting square

knows left wall follow will finish, so take advantage of that
*/

#include <stdio.h>
#include "picomms.h"
#include <math.h>
#include "v1.h"
#include "turn.h"
#include "routeThruWalls.h"

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
	set_origin();
	set_ir_angle(LEFT, -45); set_ir_angle(RIGHT, 45);
	setSquareCenters();
	while(!state)
	{
		state = checkWalls();
		calcPos();		
	}
	racePace();
	return 1;
}