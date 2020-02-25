/**********************************************************************
  Code by lingib ,modefied by Abdullah
  Last update 3 June 2019

  This program controls a three-wheel "omni-wheel CNC".
  ----------
  COPYRIGHT
  ----------
  This code is free software: you can redistribute it and/or
  modify it under the terms of the GNU General Public License as published
  by the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License. If
  not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// -------------------------------
// GLOBALS
// -------------------------------

// ----- constants
#
define PI 3.1415926535897932384626433832795# define HALF_PI 1.5707963267948966192313216916398# define TWO_PI 6.283185307179586476925286766559# define DEG_TO_RAD 0.017453292519943295769236907684886# define RAD_TO_DEG 57.295779513082320876798154814105

// ----- Bit set/clear/check/toggle macros
# define SET(x, y)(x |= (1 << y))# define CLR(x, y)(x &= (~(1 << y)))# define CHK(x, y)(x & (1 << y))# define TOG(x, y)(x ^= (1 << y))

// ----- motor definitions
# define STEPS_PER_MM 200 * 8 / (58 * PI) //200steps/rev; 8 x microstepping; 58mm dia. wheels
# define NUDGE STEPS_PER_MM * 5 //move   5mm (change number to suit)
# define STEP1 13 //arduino ports
# define STEP2 11# define STEP3 9# define DIRECTION1 12# define DIRECTION2 10# define DIRECTION3 8

# define CW false# define CCW true
bool ROTATE1; // Wheel W1 doesn't rotate in vertical direction
bool DIR1; // Wheel W1
bool DIR2; // Wheel W2
bool DIR3; // Wheel W3

long
PULSE_WIDTH = 2, //easydriver step pulse-width (uS)
  DELAY = 1000; //delay (uS) between motor steps (controls speed)

// ----- plotter definitions
#
define BAUD 9600# define XOFF 0x13 //pause transmission (19 decimal)
# define XON 0x11 //resume transmission (17 decimal)
# define LampRelay1 3# define LampRelay2 4

float
SCALE_FACTOR = 1.0, //drawing scale (1 = 100%)
  SCALE_FACTOR_VERT = sqrt(3), //allow for angled wheels
  ARC_MAX = 2.0; //maximum arc-length (controls smoothness)

int
CurentXRequiredtoMoveTo,
CurrentYRequiredtoMoveTo,
CurrentAngleRequiredtoMoveTo;


int
/*
   XY plotters only deal in integer steps.
*/
THIS_X = 0, //"scaled" x co-ordinate (rounded)
  THIS_Y = 0, //"scaled" y co-ordinate (rounded)
  LAST_X = 0, //"scaled" x co-ordinate (rounded)
  LAST_Y = 0; //"scaled" y co-ordinate (rounded)

// ----- gcode definitions
#
define STRING_SIZE 128 //string size

char
BUFFER[STRING_SIZE + 1],
  INPUT_CHAR;

String
INPUT_STRING,
SUB_STRING;
int GlobalAngleCurrent
int
INDEX = 0, //buffer index
  START, //used for sub_string extraction
  FINISH;

float
X, //gcode float values held here
Y,
I,
J;

#include <ros.h>

#include <std_msgs/Empty.h>

ros::NodeHandle nh;
ros::Subscriber < std_msgs::Empty > sub("VisionState", & messageCb);
void messageCb(const std_msgs::UInt16 & cmd_msg) {
  GlobalAngleCurrent = cmd_msg.data;
}

// -----------------------
// SETUP
// -----------------------
void setup() {

  //-- initiliasing the nodes
  nh.initNode();
  nh.subscribe(sub);
  // ----- initialise motor1
  pinMode(STEP1, OUTPUT);
  pinMode(DIRECTION1, OUTPUT);
  digitalWrite(DIRECTION1, CW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP1, LOW);

  // ----- initialise motor2
  pinMode(STEP2, OUTPUT);
  pinMode(DIRECTION2, OUTPUT);
  digitalWrite(DIRECTION2, CW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP2, LOW);

  // ----- initialise motor3
  pinMode(STEP3, OUTPUT);
  pinMode(DIRECTION3, OUTPUT);
  digitalWrite(DIRECTION3, CW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP3, LOW);

  //-- initialise THE RELAY 
  pinMode(LampRelay1, OUTPUT);
  pinMode(LampRelay2, OUTPUT);
  digitalWrite(LampRelay1, HIGH);
  digitalWrite(LampRelay2, HIGH);

  delay(100000); // wait for the amps to heat up and reach steady state condition 

}

//--------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------
void loop() {

  nh.spinOnce();
  delay(1);
  move_to(CurentXRequiredtoMoveTo, CurentYRequiredtoMoveTo);
  rotate_by(CurrentAngleRequiredtoMoveTo);

}

void rotate_by(float x) { //x,y are absolute co-ordinates
  int ConcversionTosteps = 0.0325;
  int NofstepsRotation = x / ConcversionTosteps;
  int n = 0
  while (n <= NofstepsRotation) {

    if (x > 0) {

      CWRotate();
      step_motors();
    } else {
      CCWRotate();
      step_motors();
    }
    //--tolerance of the error of 3 degrees   -- correction loop 
    while (abs(GlobalAngleCurrent - x) >= 3) {

      NofstepsRotation = abs(GlobalAngleCurrent - x) / ConcversionTosteps;
      n = 0

      if (GlobalAngleCurrent - x > 0) {
        for (int i = 0; i <= NofstepsRotation; 1++) {

          CWRotate();
          step_motors();
        }

      } else {

        for (int i = 0; i <= NofstepsRotation; 1++) {

          CCWRotate();
          step_motors();
        }

      }

    }

  }

}

void move_to(float x, float y) { //x,y are absolute co-ordinates

  // ----- apply scale factor
  THIS_X = round(x * STEPS_PER_MM * SCALE_FACTOR);
  THIS_Y = round(y * STEPS_PER_MM * SCALE_FACTOR_VERT * SCALE_FACTOR); //prescale y axis to compensate for angled wheels
  // ----- draw a line between these "scaled" co-ordinates
  draw_line(LAST_X, LAST_Y, THIS_X, THIS_Y);

  // ----- remember last "scaled" co-ordinate
  LAST_X = THIS_X;
  LAST_Y = THIS_Y;
}

// ------------------------------------------------------------------------
// DRAW LINE
// ------------------------------------------------------------------------
/*
  This routine assumes that motor1 controls the x-axis and that motor2 controls
  the y-axis.

  The algorithm automatically maps all "octants" to "octant 0" and
  automatically swaps the XY coordinates if dY is greater than dX. A swap
  flag determines which motor moves for any combination X,Y inputs. The swap
  algorithm is further optimised by realising that dY is always positive
  in quadrants 0,1 and that dX is always positive in "quadrants" 0,3.

  Each intermediate XY co-ordinate is plotted which results in a straight line
*/
void draw_line(int x1, int y1, int x2, int y2) { //these are "scaled" co-ordinates

  // ----- locals
  int
  x = x1, //current "scaled" X-axis position
    y = y1, //current "scaled" Y-axis position
    dy, //line slope
    dx,
    slope,
    longest, //axis lengths
    shortest,
    maximum,
    error, //bresenham thresholds
    threshold;

  // ----- find longest and shortest axis
  dy = y2 - y1; //vertical distance
  dx = x2 - x1; //horizontal distance
  longest = max(abs(dy), abs(dx)); //longest axis
  shortest = min(abs(dy), abs(dx)); //shortest axis

  // ----- scale Bresenham values by 2*longest
  error = -longest; //add offset to so we can test at zero
  threshold = 0; //test now done at zero
  maximum = (longest << 1); //multiply by two
  slope = (shortest << 1); //multiply by two ... slope equals (shortest*2/longest*2)

  // ----- initialise the swap flag
  /*
     The XY axes are automatically swapped by using "longest" in
     the "for loop". XYswap is used to decode the motors.
  */
  bool XYswap = true; //used for motor decoding
  if (abs(dx) >= abs(dy)) XYswap = false;

  // ----- pretend we are always in octant 0
  /*
     The current X-axis and Y-axis positions will now be incremented (decremented) each time
     through the loop. These intermediate steps are parsed to the plot(x,y) function which calculates
     the number of steps required to reach each of these intermediate co-ordinates. This effectively
     linearises the plotter and eliminates unwanted curves.
  */
  for (int i = 0; i < longest; i++) {

    // ----- move left/right along X axis
    if (XYswap) { //swap
      if (dy < 0) {
        y--;
        down(); //move   1 step down
      } else {
        y++;
        up(); //move   1 step up
      }
    } else { //no swap
      if (dx < 0) {
        x--;
        left(); //move   1 step left
      } else {
        x++;
        right(); //move   1 step right
      }
    }

    // ----- move up/down Y axis
    error += slope;
    if (error > threshold) {
      error -= maximum;

      // ----- move up/down along Y axis
      if (XYswap) { //swap
        if (dx < 0) {
          x--;
          left(); //move   1 step left
        } else {
          x++;
          right(); //move   1 step right
        }
      } else { //no swap
        if (dy < 0) {
          y--;
          down(); //move   1 step down
        } else {
          y++;
          up(); //move   1 step up
        }
      }
    }
  }
}
//--------------------------------------------------------------------
// CCWRotate()           (rotate about self ccw)
//--------- -----------------------------------------------------------

void CCWRotate() {
  ROTATE1 = true;
  DIR1 = CCW;
  DIR2 = CCW;
  DIR3 = CCW;
  step_motors();
}

//--------------------------------------------------------------------
// CWRotate()           (rotate about self cw)
//--------- -----------------------------------------------------------

void CWRotate() {
  ROTATE1 = true;
  DIR1 = CCW;
  DIR2 = CCW;
  DIR3 = CCW;
  step_motors();
}
//--------------------------------------------------------------------
// LEFT()           (move the   1 step left)
//--------- -----------------------------------------------------------
void left() {
  ROTATE1 = true;
  DIR1 = CW;
  DIR2 = CCW;
  DIR3 = CCW;
  step_motors();
}

//--------------------------------------------------------------------
// RIGHT()           (move the   1 step right)
//--------- -----------------------------------------------------------
void right() {
  ROTATE1 = true;
  DIR1 = CCW;
  DIR2 = CW;
  DIR3 = CW;
  step_motors();
}

//--------------------------------------------------------------------
// UP()               (move the   1 step up)
//--------- -----------------------------------------------------------
void up() {
  ROTATE1 = false;
  DIR1 = CW; // Ignored
  DIR2 = CCW;
  DIR3 = CW;
  step_motors();
}

//--------------------------------------------------------------------
// DOWN()             (move the   1 step down)
//--------- -----------------------------------------------------------
void down() {
  ROTATE1 = false;
  DIR1 = CW; // Ignored
  DIR2 = CW;
  DIR3 = CCW;
  step_motors();
}

//----------------------------------------------------------------------------------------
// STEP MOTORS
//----------------------------------------------------------------------------------------
void step_motors() {

  // ----- locals
  enum {
    dir3,
    step3,
    dir2,
    step2,
    dir1,
    step1
  }; //define bit positions
  byte pattern = PORTB; //read current state PORTB

  // ----- set motor directions
  /*
     Change CCW to CW if the robot moves in the wrong direction
  */
  (DIR1 == CCW) ? SET(pattern, dir1): CLR(pattern, dir1);
  (DIR2 == CCW) ? SET(pattern, dir2): CLR(pattern, dir2);
  (DIR3 == CCW) ? SET(pattern, dir3): CLR(pattern, dir3);
  PORTB = pattern;
  delayMicroseconds(PULSE_WIDTH); //wait for direction lines to stabilise

  // ----- create leading edge of step pulse(s)
  (ROTATE1 == true) ? SET(pattern, step1): CLR(pattern, step1);
  SET(pattern, step2);
  SET(pattern, step3);
  PORTB = pattern; //step the motors
  delayMicroseconds(PULSE_WIDTH); //mandatory delay

  // ----- create trailing-edge of step-pulse(s)
  pattern = CLR(pattern, step1);
  pattern = CLR(pattern, step2);
  pattern = CLR(pattern, step3);
  PORTB = pattern;

  // ----- determines plotting speed
  delayMicroseconds(DELAY);
}