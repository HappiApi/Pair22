/*
Module Usage : Accepts bearings x, where 0 < x < 360 where 0 is aligned with the y-axis
			   Clockwise is positive, vice versa
*/
#include <stdio.h>
#include <math.h>
#include "position.h"
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

#define LEFT_TURN -1
#define RIGHT_TURN 1
#define ABOUT_TURN 2
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

int currentDirection() {
	return (int)round(currentHeading() / (M_PI / 2));
}

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
  idealX = currentX();
  idealY = currentY();
}

void setSquareCenters()
{
	int i, j;

	for (i = 0; i < 240; i += 60) {
		for (j = 60; j < 300; j += 60) {
			set_point(i, j);
		}
	}
}

void initialBoxCalibration()
{
	turnToDirection(LEFT_TURN);
	correctPosition(LEFT);
	turnToDirection(ABOUT_TURN);
	set_origin();
	resetDist();
}

void correctToStraight(int motorSpeed) {
	if (get_side_ir_dist(LEFT) < 10) {
		set_motors(motorSpeed + 3, motorSpeed);
	} else if (get_side_ir_dist(RIGHT) < 10) {
		set_motors(motorSpeed, motorSpeed + 3);
	} else {
		set_motors(motorSpeed, motorSpeed);
	}
}

#define X 0
#define Y 1

void driveStraight(int XorY, int idealVal) {
	int currentVal;

	if (XorY == X) 
		currentVal = currentX();
	else
		currentVal = currentY();

	while(fabs(currentVal-idealVal) > 1) {
		if(fabs(currentVal-idealVal) < 15) {
				correctToStraight(2 * fabs(currentVal-idealVal));
		} else {
			set_motors(40,40);
		}
		calcPos();

		if (XorY == X) {
			currentVal = currentX();
			// idealX = currentVal;
		} else {
			currentVal = currentY();
			// idealY = currentVal;
		}
		if (averageUS() < 25) {
			break;
		}
	}
	set_motors(0,0);
}

void Straight()
{
	adjustAngle();
	// Going Up

	int i;
	
	for (i = 0; i < 11; i++) {
		set_motors(3 * i, 3 * i);
		usleep(70000);
	}

	if(currentDirection() == 0 || currentDirection() == 4) {
		idealY += 60;
		printf("UP TargetY: %f\n",idealY);
		driveStraight(Y, idealY);
		bCoord.y += 1;
	}

	// Going Down
	if(currentDirection() == 2) {
		idealY -= 60;
		printf("DOWN TargetY: %f\n",idealY);
		driveStraight(Y, idealY);
		bCoord.y -= 1;
	}

	// Going Right
	if(currentDirection() == 1) {
		idealX += 60;
		printf("RIGHT TargetX: %f\n",idealX);
		driveStraight(X, idealX);
		bCoord.x += 1;
	}

	// Going Left
	if(currentDirection() == 3) {
		idealX -= 60;
		printf("LEFT TargetX: %f\n",idealX);
		driveStraight(X, idealX);
		bCoord.x -= 1;
	}

	set_motors(0,0);

}

int getAverage(int *values, int length) {
	int errorCount = 0; int count = 0;
	int i, j, outliers[length];
	int acceptableRange = 4;
	int result = 0;

	for (i = 0; i < length; i++) {
		for (j = 0; j < length; j++) {
			if (fabs(*(values + i) - *(values + j)) > acceptableRange) {
				errorCount++;
			} 
		}
		if (errorCount > length / 2) {
			outliers[count++] = i;
		}
		errorCount = 0;
	}

	count = 0;

	for (i = 0; i < length; i++) {
		if (i == outliers[count]) {
			count++;
		} else {
			result += *(values + i);
			errorCount++;
		}
	}

	return result / errorCount;
}

int checkWall(int LorR)
{
	int noOfChecks = 10;
	int distancesSide[noOfChecks]; int distancesFront[noOfChecks];
	int final = 0;
	int i;
	for(i = 0;i<noOfChecks;i++) {
		distancesSide[i] = get_side_ir_dist(LorR);
		distancesFront[i] = get_front_ir_dist(LorR);
		usleep(10000);
	}
	final = (getAverage(distancesSide, noOfChecks) + getAverage(distancesFront, noOfChecks)) / 2;
	if (LorR == LEFT) {
		printf("\nLeft");
	} else {
		printf("\nRight");
	}
	printf(" wall Dist : %i\n", final);
	if(final <= 40) // tighter bound ?? e.g 21
		return 1;
	else
		return 0;
}

int checkFrontWall()
{
	int noOfChecks = 5;
	int distances[noOfChecks];
	int final = 0;
	int i;
	for(i = 0; i < noOfChecks; i++) {
		distances[i] = get_us_dist();
	}
	final = getAverage(distances, noOfChecks);

	printf("\n\nFinal: %i\n", final);

	if(final <= 40)
		return 1;	
	else
		return 0;
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
	int noOfChecks = 5;
	int values[noOfChecks];
	int i, average;

	for (i = 0; i < noOfChecks; i++) {
		values[i] = get_us_dist();
	}

	average = getAverage(values, noOfChecks);
	printf("average US dist: %i\n", average);
	return average;
}

int checkWalls()
{
	static int count = 0;
	int left,right,front;
	left = checkWall(LEFT);
	right = checkWall(RIGHT);
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
