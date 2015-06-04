/*
Module Usage : Accepts bearings x, where 0 < x < 360 where 0 is aligned with the y-axis
			   Clockwise is positive, vice versa
*/
#include <stdio.h>
#include <math.h>
#include "v1.h"
#include "picomms.h"
#include "wallStruct.h"

#define HalfPi (M_PI/2.0)
#define turnThreshold 0.01 //most recent 0.005//0.007 too much error //0.0004 wiggles 
/* 
	Test on M_PI/2.0 Turn

	Smallerst possible on simulator before infinite loop is 0.0004 (decreasing by 0.0001)

	Max Error on each M_PI/2.0 turn is 0.0004/(M_PI/2.0) * 100 = 0.0254% (3s.f) 

		Travelling at speed (1,-1) min increment for angle is = 0.007408 radians on simualtor
	(consistent across 5 measured differences). Should we set turnThreshold to 0.007 for real robot?
	Maybe min increment for angle will be smaller for faster algorithm ? 

		On test for Pi/2 turn value jumped between 1.1570486 & 1.1577894 for turnThreshold 0.0003
	where Pi/2 is 1.570796 (6d.p) (as precise as can be represented as double ?)

*/
#define slowThreshold 0.05 //0.05 works
/*
	Slow down when reach

	COORDINATES SYSTEM FOR SQUARES
*/

double idealX = 0;
double idealY = 0;

struct boxCoord
{
	int x;
	int y;
};

struct boxCoord bCoord = {0,0};

void correctToStraight(int motorSpeed) {
	if (get_side_ir_dist(LEFT) < 15) {
		set_motors(motorSpeed + 3, motorSpeed);
	}
	if (get_side_ir_dist(RIGHT) < 15) {
		set_motors(motorSpeed, motorSpeed + 3);
	}
}



void initializeWalls()
{
	int i;
	for(i=0;i<24;i++)
	{
		Walls[i].horizontalNumber = 0;
		Walls[i].verticalNumber = 0;
		Walls[i].state = 2; 
	}
}

// #define U 1
// #define R 2
// #define D 3
// #define L 4

int currentDirection() {
	// int head = (int)round(currentHeading() / (M_PI / 2));
	// if (head == 4) {
	// 	head = 0;
	// }
	// printf("\n\n\n%i\n\n\n", (int)round(currentHeading() / (M_PI / 2)));
	return (int)round(currentHeading() / (M_PI / 2));
}

#define LEFT_TURN -1
#define RIGHT_TURN 1
#define ABOUT_TURN 2

void turnToDirection(int turnDirection) {
	adjustAngle();
	int currentDir;
	currentDir = currentDirection();
	double target = (currentDir + turnDirection) * M_PI / 2;

	while(fabs(currentHeading() - target) > turnThreshold) {
		if(fabs(currentHeading() - target) < slowThreshold) {
			if(currentHeading() > target) {
				set_motors(-5,5);
			}
			if(currentHeading() < target) {
				set_motors(5,-5);
			}
		} else {
			if(currentHeading() > target) {
				set_motors(-15,15);
			}
			if(currentHeading() < target) {
				set_motors(15,-15);
			}
		}
		calcPos();
	}
	set_motors(0,0);
	adjustAngle();
	usleep(10000);
}

void turn90()
{
	while( fabs(currentHeading() - (M_PI/2.0)) > turnThreshold)
	{
		if(fabs(currentHeading() - (M_PI/2.0)) < slowThreshold)
		{
			if(currentHeading() > (M_PI/2.0))
			{
				set_motors(-1,1);
			}

			if(currentHeading() < (M_PI/2.0))
			{
				set_motors(1,-1);
			}
		}

		else
		{
			if(currentHeading() > (M_PI/2.0))
			{
				set_motors(-15,15);
			}

			if(currentHeading() < (M_PI/2.0))
			{
				set_motors(15,-15);
			}
		}
		
		calcPos();
	}
	set_motors(0,0);
	printf("Target : %f\n", M_PI/2.0);
}

void adjustStart()
{
	turnToDirection(ABOUT_TURN);
	int backDistFromWall = get_us_dist();
	int targetDistSide = 27;
	while(fabs(backDistFromWall - 27) > 1)
	{
		printf("%d\n",backDistFromWall);
		if(backDistFromWall > 27)
		{
			set_motors(5,5);
		}
		else
		{
			set_motors(-5,-5);
		}
		backDistFromWall = get_us_dist();
		calcPos();
	}
	turnToDirection(LEFT_TURN);
	int sideDistFromWall = get_us_dist();
	while(fabs(sideDistFromWall - targetDistSide) > 1)
	{
		printf("%d\n",sideDistFromWall);
		if(sideDistFromWall > targetDistSide)
		{
			set_motors(5,5);
		}
		else
		{
			set_motors(-5,-5);
		}
		sideDistFromWall = get_us_dist();
		calcPos();
	}
	turnToDirection(LEFT_TURN);
}

void setSquareCenters()
{
	adjustStart();
	set_origin();
	resetDist();

	int i, j;

	for (i = 0; i < 240; i += 60) {
		for (j = 60; j < 300; j += 60) {
			set_point(i, j);
		}
	}
}

void Straight()
{
	adjustAngle();
	// Going Up

	int i;
	int motorSpeed;

	for (i = 0; i < 11; i++) {
		set_motors(3 * i, 3 * i);
		usleep(100000);
	}

	if(currentDirection() == 0 || currentDirection() == 4) {
		idealY += 60;
		printf("UP TargetY: %f\n",idealY);

		while(fabs(currentY()-idealY) > 1) {
			if(fabs(currentY()-idealY) < 10) {
				if(currentY() < idealY) {
					correctToStraight(2 * fabs(currentY()-idealY));
					// set_motors(motorSpeed, motorSpeed);
				} else {
					correctToStraight(-2 * fabs(currentY()-idealY));
					// set_motors(-5 -2 * fabs(currentY()-idealY),-5 -2 * fabs(currentY()-idealY));
				}
			} else if(currentY() < idealY) {
				set_motors(40,40);
			} else if(currentY() > idealY) {
				set_motors(-40,-40);
			}
			calcPos();
			//printf("X:%f Y:%f\n", currentX(),currentY());
		}
		set_motors(0,0);
		bCoord.y += 1;
	}

	// Going Down
	if(currentDirection() == 2) {
		idealY -= 60;
		printf("DOWN TargetY: %f\n",idealY);

		while(fabs(currentY()-idealY) > 1) {
			if(fabs(currentY()-idealY) < 10) {
				if(currentY() > idealY) {
					correctToStraight(2 * fabs(currentY()-idealY));
					// set_motors(5 + 2 * fabs(currentY()-idealY), 5 + 2 * fabs(currentY()-idealY));
				} else {
					correctToStraight(-2 * fabs(currentY()-idealY));
					// set_motors(-5 -2 * fabs(currentY()-idealY),-5 -2 * fabs(currentY()-idealY));
				}
			} else if(currentY() < idealY) {
				set_motors(-40,-40);
			} else if(currentY() > idealY) {
				set_motors(40,40);
			}
			calcPos();
			//printf("X:%f Y:%f\n", currentX(),currentY());
		}
		set_motors(0,0);
		bCoord.y -= 1;
	}

	// Going Right
	if(currentDirection() == 1) {
		idealX += 60;
		printf("RIGHT TargetX: %f\n",idealX);

		while(fabs(currentX()-idealX) > 1) {
			if(fabs(currentX()-idealX) < 10) {
				if(currentX() < idealX) {
					correctToStraight(2 * fabs(currentX()-idealX));
					// set_motors(5 + 2 * fabs(currentX()-idealX),5 + 2 * fabs(currentX()-idealX));
				} else {
					correctToStraight(-2 * fabs(currentX()-idealX));
					// set_motors(-5 -2 * fabs(currentX()-idealX),-5 -2 * fabs(currentX()-idealX));
				}
			} else if(currentX() < idealX) {
				set_motors(40,40);
			} else if(currentX() > idealX) {
				set_motors(-40,-40);
			}

			calcPos();
			//printf("X:%f Y:%f\n", currentX(),currentY());
		}
		set_motors(0,0);
		bCoord.x += 1;
	}

	// Going Left
	if(currentDirection() == 3) {
		idealX -= 60;
		printf("LEFT TargetX: %f\n",idealX);

		while(fabs(currentX()-idealX) > 1) {
			if(fabs(currentX()-idealX) < 10) {
				if(currentX() > idealX) {
					correctToStraight(2 * fabs(currentX()-idealX));
					// set_motors(5 + 2 * fabs(currentX()-idealX),5 + 2 * fabs(currentX()-idealX));
				} else {
					correctToStraight(-2 * fabs(currentX()-idealX));
					// set_motors(-5 -2 * fabs(currentX()-idealX),-5 -2 * fabs(currentX()-idealX));
				}
			} else if(currentX() < idealX) {
				set_motors(-40,-40);
			} else if(currentX() > idealX) {
				set_motors(40,40);
			}
			calcPos();
			//printf("X:%f Y:%f\n", currentX(),currentY());
		}
		set_motors(0,0);
		bCoord.x -= 1;
	}
}

int checkLeftWall()
{
	int noOfChecks = 5;
	int distances[noOfChecks];
	int final = 0;
	int i;
	for(i = 0;i<noOfChecks;i++)
	{
		distances[i] = get_side_ir_dist(LEFT);
		final += distances[i];
		//printf("%d\n",final );
	}
	final /= noOfChecks;
	printf("\nLeft wall Dist : %i\n", final);
	if(final <= 35) // tighter bound ?? e.g 21
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int checkRightWall()
{
	int noOfChecks = 5;
	int distances[noOfChecks];
	int final = 0;
	int i;
	for(i = 0;i<noOfChecks;i++)
	{
		distances[i] = get_side_ir_dist(RIGHT);
		final += distances[i];
		if (i > 2) {
			
		}
		//printf("%d\n",final );
	}
	final /= noOfChecks;
	printf("\nRight wall Dist : %i\n", final);
	if(final <= 35) // tighter bound ?? e.g 21
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int checkFrontWall()
{

	int distances[2];
	int final = 0;
	int i;
	for(i = 0;i<2;i++)
	{
		distances[i] = get_us_dist();
		final += distances[i];
		//printf("%d\n",final );
	}
	final /= 2;

	if(final <= 40)
	{
		return 1;	
	}
	else
	{
		return 0;
	}
}

int checkWallExists(int horizontalNumber, int verticalNumber) {
	int i;
	for (i = 0; i < 24; i++) {
		if (Walls[i].horizontalNumber == horizontalNumber && Walls[i].verticalNumber == verticalNumber) {
			return 1;
		}
	}
	return 0;
}

int averageUS() {
	int average = 0 ;
	int count;

	for (count = 0; count < 3; count++) {
		average += get_us_dist();
	}

	printf("average : %i\n", average / 3);
	return average / 3;
}

void correctPosition(int LorRwall) {  
  int targetDistNorth = 27;
  int targetDistSide = 27; //22

  if (LorRwall == LEFT) {
    while (averageUS() < targetDistNorth - 2 || averageUS() > targetDistNorth + 2) {
      if (averageUS() < targetDistNorth - 2) 
        set_motors(-5,-5);
      else
        set_motors(5,5);
      calcPos();
    }
    turnToDirection(LEFT_TURN);
    while (averageUS() < targetDistSide - 2 || averageUS() > targetDistSide + 2) {
      if (averageUS() < targetDistSide - 2) 
        set_motors(-5,-5);
      else
        set_motors(5,5);
      calcPos();
    }
  } else {
    while (averageUS() < targetDistNorth - 2 || averageUS() > targetDistNorth + 2) {
      if (averageUS() < targetDistNorth - 2) 
        set_motors(-5,-5);
      else
        set_motors(5,5);
      calcPos();
    }
    turnToDirection(RIGHT_TURN);
    while (averageUS() < targetDistSide - 2 || averageUS() > targetDistSide + 2) {
      if (averageUS() < targetDistSide - 2) 
        set_motors(-5,-5);
      else
        set_motors(5,5);
      calcPos();
    }
  }
}

int checkWalls()
{
	static int count = 0;
	int left,right,front;
	left = checkLeftWall();
	right = checkRightWall();
	front = checkFrontWall();
	int north = 0, south = 0, east = 0, west = 0;

	if(fabs(currentHeading()) < 0.1 || fabs((currentHeading()-(2*M_PI)))<0.1) { //north
		if (left) west = 1;
		if (right) east = 1;
		if (front) north = 1;
	}
	if(fabs(currentHeading()-(M_PI)) < 0.1) { //south
		if (left) east = 1;
		if (right) west = 1;
		if (front) south = 1;
	}
	if(fabs(currentHeading()-HalfPi) < 0.1) { //east
		if (left) north = 1;
		if (right) south = 1;
		if (front) east = 1;
		if (front) printf("\n\n\nwall ahead!! east : %i, wall exists : %i\n\n\n", east, checkWallExists(bCoord.x * 2, bCoord.y * 2 + 1));
	}
	if(fabs(currentHeading()-(1.5*M_PI)) < 0.1) { //west
		if (left) south = 1;
		if (right) north = 1;
		if (front) west = 1;
	}

	if (east) {
		if (!checkWallExists(bCoord.x * 2 + 2, bCoord.y * 2 - 1)) {
			Walls[count].horizontalNumber = bCoord.x * 2 + 2; Walls[count].verticalNumber = bCoord.y * 2 - 1; //left wall
			count++;
			set_point(60 * bCoord.x + 30, 60 * bCoord.y);
		}
	}
	if (west) {
		if (!checkWallExists(bCoord.x * 2, bCoord.y * 2 - 1)) {
			Walls[count].horizontalNumber = bCoord.x * 2; Walls[count].verticalNumber = bCoord.y * 2 - 1; //left wall
			count++;
			set_point(60 * bCoord.x - 30, 60 * bCoord.y);
		}
	}
	if (north) {
		if (!checkWallExists(bCoord.x * 2 + 1, bCoord.y * 2)) {
			Walls[count].horizontalNumber = bCoord.x * 2 + 1; Walls[count].verticalNumber = bCoord.y * 2; //left wall
			count++;
			set_point(60 * bCoord.x, 60 * bCoord.y + 30);
		}
	}
	if (south) {
		if (!checkWallExists(bCoord.x * 2 + 1, bCoord.y * 2 - 2)) {
			Walls[count].horizontalNumber = bCoord.x * 2 + 1; Walls[count].verticalNumber = bCoord.y * 2 - 2; //left wall
			count++;
			set_point(60 * bCoord.x, 60 * bCoord.y - 30);
		}
	}
	// 
	// Case 1 LR
	if(left && !front && right)
	{
		printf("CASE1 LR\n");
		Straight();
	}

	// Case 2 LF
	else if(left && front && !right)
	{
		printf("CASE2 LF\n");
		correctPosition(LEFT);
		turnToDirection(ABOUT_TURN);
		Straight();
	}

	// Case 3 LFR
	else if(left && front && right)
	{
		printf("CASE3 LFR\n");
		correctPosition(LEFT);
		turnToDirection(LEFT_TURN);
		Straight();
	}

	// Case 4 FR 
	else if(!left && front && right)
	{
		printf("CASE4 FR\n");
		correctPosition(RIGHT);
		turnToDirection(ABOUT_TURN);
		Straight();
	}

	// Case 5 L
	else if(left && !front && !right)
	{
		printf("CASE5 L\n");
		Straight();
	}

	// Case 6 R
	else if(!left && !front && right)
	{
		printf("CASE6 R\n");
		turnToDirection(LEFT_TURN);
		Straight();
	}

	// Case 7 F
	else if(!left && front && !right)
	{
		printf("CASE7 F\n");
		turnToDirection(LEFT_TURN);
		Straight();
	}

	// Case 8 None
	else if(!left && !front && !right)
	{
		printf("CASE8 NONE\n");
		turnToDirection(LEFT_TURN);
		Straight();
	}

	if(bCoord.x == 0 && bCoord.y == 0 && count > 3)
	{
			correctPosition(LEFT);
			turnToDirection(LEFT_TURN);
			return 1;
	}
	printf("bCoordX: %d bCoordY: %d\n",bCoord.x,bCoord.y );
	return 0;

}
