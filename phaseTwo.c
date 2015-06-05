#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
#include "wallStruct.h"

#define FOLLOW_SPEED 40
#define FOLLOW_BK_SPD 60
#define THRESHOLD_DIST 35.0
#define MIN_TURN_RADIUS 25
#define RIGHT_ANGLE_TICKS 210
#define NO_SLIP_TURN_SPD 15
#define TOP_TURN_SPEED 40
#define LEFT 0
#define RIGHT 1
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
#define ENC_TO_CM (9.55 * M_PI / 360)
#define ROBOT_WIDTH 22.5   // In cm

typedef struct {
	int xCoord;
	int yCoord;
} wallPosn;

typedef struct {
	int x;
	int y;
} coords;

typedef struct {
	coords current;
	int nextAvailable[4];
} graphNode;

struct dirs {
   coords current;
   int direction;
   struct dirs * next;
};

typedef struct dirs directions;

int mazeLength;
double xCurrent = 0, yCurrent = 0; double prevAngle = 0;
wallPosn walls[50];
graphNode graph[4][4];
directions* head = NULL; directions* tail;

void apisToDans() {
  int i;
  for (i = 0; i < 24; i++) {
    (walls + i)->xCoord = Walls[i].horizontalNumber; (walls + i)->yCoord = Walls[i].verticalNumber; 
    printf("\nwall %i x : %i, y : %i", i + 16, walls[i].xCoord, walls[i].yCoord);
  }
}

int checkAvailableNS(coords current, int NorS) {
	int i;
	for (i = 0; i < 50; i++) {
		if ((walls + i)->xCoord % 2 == 1 && (walls + i)->xCoord / 2 == current.x) {
			if ((walls + i)->yCoord / 2 == current.y && NorS == 2) {
				return 0;
			} else if ((walls + i)->yCoord / 2 == current.y + 1 && NorS == 0) {
				return 0;
			}
		}
	}
	return 1;
} 

int checkAvailableEW(coords current, int EorW) {
	int i;
	for (i = 0; i < 50; i++) {
		if ((walls + i)->yCoord % 2 == 1 && (walls + i)->yCoord / 2 == current.y) {
			if ((walls + i)->xCoord / 2 == current.x && EorW == 3) {
				return 0;
			} else if ((walls + i)->xCoord / 2 == current.x + 1 && EorW == 1) {
				return 0;
			}
		}
	}
	return 1;
} 

void checkAvailability(graphNode * node) {
	int i;
	for (i = 0; i < 4; i++) {
		if (i % 2 == 0) {
			node->nextAvailable[i] = checkAvailableNS(node->current, i);
		} else {
			node->nextAvailable[i] = checkAvailableEW(node->current, i);
		}
	}
}

void findPaths(graphNode * node) {
	static int currentDepth = 0, maxDepth = 1, complete = 0;
	if (node->current.x == 3 && node->current.y == 3) {
		complete = 1;
      // printf("[3,3] : currentDepth : %i\n", currentDepth);
	}
	if (currentDepth < maxDepth && complete == 0) {
		int i;
		for (i = 0; i < 4; i++) {
			if (node->nextAvailable[i] == 1) {
            currentDepth++;
				if (i == 0) {
               findPaths(&graph[node->current.x][node->current.y + 1]);
					currentDepth--;
				}
				if (i == 1) {
					findPaths(&graph[node->current.x + 1][node->current.y]);
					currentDepth--;
				}
				if (i == 2) {
					findPaths(&graph[node->current.x][node->current.y - 1]);
					currentDepth--;
				}
				if (i == 3) {
					findPaths(&graph[node->current.x - 1][node->current.y]);
					currentDepth--;
				}
			}
		}
      if (currentDepth == 0) {
         maxDepth++;
         findPaths(node);
      }
      if (complete == 1 && currentDepth > 0) {
         // printf("[%i,%i] : currentDepth : %i\n", node->current.x, node->current.y, currentDepth);
         tail = (directions *)malloc(sizeof(directions));
         tail->current.x = node->current.x;
         tail->current.y = node->current.y;
         tail->next = head;
         head = tail;
      }
	}
}

////////////////////////////////////////////////////////////////////////////////////
struct coordinates {
  double x;
  double y;
  int position;
  struct coordinates * next;
};

typedef struct coordinates path;

path* front = NULL; path* currentNode; int nodePosition = 1;

int calcDirection(int x, int y, int xNext, int yNext) { //calcs direction at end of square
  if (xNext - x == 1) {
    return EAST;
  } else if (xNext - x == -1) {
    return WEST;
  } else if (yNext - y == 1) {
    return NORTH;
  } else {
    return SOUTH;
  }
}

void addNode(int x, int y) {
  currentNode = (path *)malloc(sizeof(path)); //creates new node
  currentNode->x = x; //assigns value of x and y coords for new node
  currentNode->y = y;
  currentNode->position = nodePosition;
  currentNode->next = front;  //makes the new node point to the previous one
  front = currentNode; //changes the head of the list to be the new node
  nodePosition++;
}

void setPath(int currentX, int currentY, int nextX, int nextY) {
  int i;
  // static int prevPath = 0;   //define central = 0, upperright = 1, lowerright = 2, lowerleft = 3, upperleft = 4
  // int distAbovePath = 10;
  // int xCoord, yCoord;
  if (nextX != currentX) {
     for (i = 0; i < 60; ++i)
     {
        addNode(60*currentX + i * (nextX - currentX), 60*currentY + 60);
        set_point(60*currentX + i * (nextX - currentX), 60*currentY + 60);
     } 
  } else {
     for (i = 0; i < 60; ++i)
     {
        addNode(60*currentX, 60*currentY + 60 + i * (nextY - currentY));
        set_point(60*currentX, 60*currentY + 60 + i * (nextY - currentY));
     } 
  }

}

path* reverse(path* root) {
  path* new_root = 0;
  while (root) {
    path* next = root->next;
    root->next = new_root;
    new_root = root;
    root = next;
  }
  return new_root;
}
//////////////////////////////////////////////////////////////////////////////////
/*finds the angle between the heading direction and a point*/
double angleToTargetPoint(path * target) {
  double angleFromY, deltaX, deltaY, angleFromCurrent;// diagDist, aheadDist, acrossDist;

  deltaX = target->x - xCurrent;
  deltaY = target->y - yCurrent;

  angleFromY = atan2(deltaX, deltaY);
  if (angleFromY < 0) {
    angleFromY += 2 * M_PI;   //return a value from 0 to 2Pi for the angle from y
  }
  if (angleFromY - prevAngle > M_PI) {
    angleFromCurrent = -(2 * M_PI - (angleFromY - prevAngle));  //if the angle to be turned is over 180degrees to the right, convert it to (360 - angle) degrees to the left
  } else if (angleFromY - prevAngle < -M_PI) {
    angleFromCurrent = 2 * M_PI + (angleFromY - prevAngle); //if angle to be turned is over 180 degrees to the left, convert it to (360 + angle) degrees to the right
  } else {
    angleFromCurrent = angleFromY - prevAngle;
  }
  
  return angleFromCurrent * 180 / M_PI;
}

/*finds the distance between the robot and a point*/
double calcDistToPoint (path * temp) {
  double deltaX, deltaY, distToPoint;

  deltaX = temp->x - xCurrent;
  deltaY = temp->y - yCurrent; 
  distToPoint = sqrt(deltaX * deltaX + deltaY * deltaY);
  // printf("\ndistToPoint : %f\n", distToPoint);

  return distToPoint;
}

/*finds the furthest coordinate ahead within a set radius (target) */
path * findTarget () {
  path * temp = currentNode;
  double target = 30;
  double distToNode, closestNodeDist = 1000.0;
  
    while(1) {
      distToNode = calcDistToPoint(temp);
      if (distToNode < closestNodeDist) {
        currentNode = temp;
        closestNodeDist = distToNode;
      }
      if (temp->position == mazeLength * 60) {  //don't allow it to try searching beyond the end of the list
        return temp;
      }
      if (distToNode > target) {  
        printf("\n%i\n", temp->position);
        return temp;
      } else {
        temp = temp->next; 
      }
    }
}

void calcPositionMaths(int leftEnc, int rightEnc) {
  double dCenter;
  double angleTurned;

  dCenter = (leftEnc + rightEnc) / 2.0;
  angleTurned = ENC_TO_CM * (leftEnc - rightEnc) / ROBOT_WIDTH;
  if (prevAngle + angleTurned < 0) {
    prevAngle += 2 * M_PI + angleTurned;
  } else if (prevAngle + angleTurned > 2 * M_PI) {
    prevAngle += angleTurned - 2 * M_PI;
  } else {
    prevAngle += angleTurned ; 
  }
  yCurrent += dCenter * cos(prevAngle) * ENC_TO_CM;
  xCurrent += dCenter * sin(prevAngle) * ENC_TO_CM;

  set_point(round(xCurrent), round(yCurrent)); //set point in centimetres
  printf("X : %f Y : %f Angle : %f\n", xCurrent, yCurrent, prevAngle * 180 / 3.141592);
  printf("Dist Travelled: %icm, angle travelled at: %f degrees\n", (int)sqrt((xCurrent * xCurrent) + (yCurrent * yCurrent)), prevAngle * 180 / M_PI); 
}

/*apply data smoothing to the encoder readings within this function*/
void calcPosition() {
  static int i = 0, lEnc[3], rEnc[3], lEncFinal, rEncFinal, leftEncPrev = 0, rightEncPrev = 0;
  get_motor_encoders(&lEnc[i%3], &rEnc[i%3]);
  if (i > 1) {
    lEncFinal = round((lEnc[0] + lEnc[1] + lEnc[2]) / 3.0);
    rEncFinal = round((rEnc[0] + rEnc[1] + rEnc[2]) / 3.0);
    calcPositionMaths(lEncFinal - leftEncPrev, rEncFinal - rightEncPrev);
    leftEncPrev = lEncFinal; rightEncPrev = rEncFinal;
  } 
  i++;
}

/*Function that handles the following back of a linked list of coordinates*/
void followBack() {
  path * target;
  double angleToTarget;
  double ratio = 0.03; //0.03
  while(1) {
    target = findTarget();
    if (currentNode->position == nodePosition - 10) {
      set_motors(0,0);
      exit(0);
    }
    angleToTarget = angleToTargetPoint(target);
    if (angleToTarget > 0) {
      set_motors(FOLLOW_BK_SPD, (int)((1 - ratio * angleToTarget) * (float)FOLLOW_BK_SPD));
    } else {
      set_motors((int)((1.0 - (ratio * -angleToTarget)) * FOLLOW_BK_SPD), FOLLOW_BK_SPD);
    }

    calcPosition();
    usleep(1000);
  }
}

void startProcedure() {
  set_ir_angle(LEFT, 90);
  set_ir_angle(RIGHT, -90);
  sleep(1);
  set_ir_angle(LEFT, 0);
  set_ir_angle(RIGHT, 0);  
  reset_motor_encoders(); 
}

int racePace() {
	int i;
  // int prevDir;
  set_origin();
	for (i = 0; i < 50; i++) {
		(walls + i)->xCoord = 0;
		(walls + i)->yCoord = 0;
	}
	// for (i = 0; i < 4; i++) {
	// 	(walls + i)->xCoord = 0;
	// 	(walls + i)->yCoord = 2*i + 1;		//sets the left side walls
	// }
	// for (i = 4; i < 8; i++) {
	// 	(walls + i)->xCoord = 8;
	// 	(walls + i)->yCoord = 2*(i-4) + 1;		//sets the right side walls
	// }
	// for (i = 8; i < 12; i++) {
	// 	(walls + i)->xCoord = 2*(i-8) + 1;
	// 	(walls + i)->yCoord = 0;		//sets the bottom walls
	// }
	// for (i = 12; i < 16; i++) {
		// (walls + i)->xCoord = 2*(i-12) + 1;
		// (walls + i)->yCoord = 8;		//sets the top walls
	// }
  apisToDans();

	for (i = 0; i < 16; i++) {
		graph[i/4][i%4].current.x = i/4;
		graph[i/4][i%4].current.y = i%4;
	}
	for (i = 0; i < 16; i++) {
			checkAvailability(&graph[i/4][i%4]);
	}
	for (i = 0; i < 16; i++) {
			printf("(%i,%i) : N - %i, E - %i, S - %i, W - %i\n", i/4, i%4, graph[i/4][i%4].nextAvailable[0], graph[i/4][i%4].nextAvailable[1], graph[i/4][i%4].nextAvailable[2], graph[i/4][i%4].nextAvailable[3]);
	}
	findPaths(&graph[0][0]);

  setPath(0,-1,0,0);
  setPath(0,0,tail->current.x,tail->current.y);
  i = 0;
  while (tail->next) {
    i++;
    printf("%i : [%i, %i], direction: %i\n", i, tail->current.x, tail->current.y, tail->direction);
    setPath(tail->current.x, tail->current.y, tail->next->current.x, tail->next->current.y);
    tail = tail->next;
  }
  setPath(tail->current.x, tail->current.y, 3, 3);
  mazeLength = i + 3;
  printf("%i : [%i, %i], direction: %i\n", i, tail->current.x, tail->current.y, tail->direction);

  front = reverse(front);
  currentNode = front;
  path * temp = front;
  while (temp->next) {
    printf("[%f, %f]\n", temp->x, temp->y);
    temp = temp->next;
  }
  printf("\nCurrent pos : %f, %f", currentNode->x, currentNode->y);
  // sleep(5);

  startProcedure();

	followBack();

  return 0;
}

