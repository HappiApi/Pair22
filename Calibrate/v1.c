/*

Module Usage:

Requirements : Have to be called every time going through loop

*/

#include <stdio.h>
#include "picomms.h"
#include <math.h>
// /#include "calcPos2.h"
// calibrate (take measurements) before start 
// figure out when in center of box and draw walls
// calibrate - 1m drive (cal the wheel diameter) + 720 turn (cal the width of robot) 
// Turns by using axes as targets
// use both sensors to detect left walls, as one is used to follow
// use right 
// state machine - have modes, so turn left mode, then dead straight, then straight mode 
// increnment counter when detecting walls (like data smoothing)

#define RoboWidth 23.8
#define CMPerEnc 10.0*M_PI/360

//11.925 mark uses width = 22.5 - 22.8
int leftEnc, rightEnc;
double travleftEnc,travrightEnc;
int prevleftEnc = 0;
int prevrightEnc = 0;
double dcen;
double angle, totalAngle, currX,currY;

void calcPos()
{
		get_motor_encoders(&leftEnc, &rightEnc);
		travleftEnc = (leftEnc - prevleftEnc) * CMPerEnc;
		travrightEnc = (rightEnc - prevrightEnc) * CMPerEnc;
		dcen = (travleftEnc + travrightEnc) / 2.0;
		angle = (travleftEnc - travrightEnc)/ RoboWidth;
		currX += dcen * sin(totalAngle);
		currY += dcen * cos(totalAngle);
		totalAngle += angle;
		printf("X: %f  Y: %f  Angle : %f \n",currX,currY,totalAngle);
		//log_trail((int)currX,(int)currY); not a good idea unless you have a super computer
		prevleftEnc = leftEnc;
		prevrightEnc = rightEnc;
}

double currentHeading()
{
	return totalAngle;
}

double currentX()
{
	return currX;
}

double currentY()
{
	return currY;
}

//not done convert to bearing(360) and modulo when too many turns ?
double bearing()
{
	/*if(totalAngle < 0 )
	{
		return 360 - fabs(totalAngle*(180/M_PI));
	}
	else
	{*/
		return totalAngle*(180/M_PI);
	//}
}	
		//totalAngle = totalAngle % (2*M_PI); accumulates error
		/*
		if(totalAngle < 0 )
		{
			totalAngle = 2*M_PI + totalAngle;
		}

		if(totalAngle > (2*M_PI))
		{
			totalAngle = 0 + (2*M_PI) - totalAngle;
		}
		*/
void adjustAngle()
{
	if(totalAngle > (2*M_PI))
	{
		totalAngle -= (2*M_PI);
	}
	if(totalAngle < (0))
	{
		totalAngle += (2*M_PI);
	}
}

void resetDist() {
	currX = 0.0;
	currY = 0.0;
	prevleftEnc = 0;
	prevrightEnc = 0;
	reset_motor_encoders();
	printf("\n\ndist reset: %f, %f\n\n", currX, currY);
}