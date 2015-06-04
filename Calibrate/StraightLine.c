#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
#include "turn.h"
#include "v1.h"

void main() {
	connect_to_robot();
	initialize_robot();

	int l, r;
	// do {

	// 	set_motors(10, 10);

	// 	get_motor_encoders(&l, &r);

	// } while (l < 100*1.0/(10.0*M_PI/360));

	turnToDirection(8);
	set_motors(0,0);
}
