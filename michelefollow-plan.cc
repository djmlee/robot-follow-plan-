/**
 * follow-plan.cc
 * 
 * Sample code for a robot that has two front bumpers and a laser, and
 * which is provided with localization data.
 *
 * The code also allows the controller to read and write a "plan", a sequence
 * of location that the robot should move to.
 *
 * Written by: Simon Parsons
 * Date:       10th November 2011
 *  
 **/

#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
#include <cmath>
#include <math.h>
using namespace PlayerCc;  


/**
 * Function headers
 *
 **/
player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);
double GetFrontLaserData(LaserProxy& sp);
double GetDistance(double, double);

int  readPlanLength(void);
void readPlan(double *, int);
void printPlan(double *,int);  
void writePlan(double *, int);

/**
 * main()
 *
 **/

int main(int argc, char *argv[])
{  

  // Variables
  int counter = 0;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  player_pose2d_t  pose;   // For handling localization data

  // The set of coordinates that makes up the plan

  int pLength;
  int planPos = 0;
  double *plan;
  double angleDiff;

  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
  PlayerClient    robot("localhost");  
  BumperProxy     bp(&robot,0);  
  Position2dProxy pp(&robot,0);
  LocalizeProxy   lp (&robot, 0);
  LaserProxy      sp (&robot, 0);
  
  // Allow the program to take charge of the motors (take care now)
  pp.SetMotorEnable(true);

  // Plan handling
  // 
  // A plan is an integer, n, followed by n doubles (n has to be
  // even). The first and second doubles are the initial x and y
  // (respectively) coordinates of the robot, the third and fourth
  // doubles give the first location that the robot should move to, and
  // so on. The last pair of doubles give the point at which the robot
  // should stop.
  pLength = readPlanLength(); // Find out how long the plan is from plan.txt
  plan = new double[pLength]; // Create enough space to store the plan
  readPlan(plan, pLength);    // Read the plan from the file plan.txt.
  printPlan(plan,pLength);    // Print the plan on the screen
  writePlan(plan, pLength);   // Write the plan to the file plan-out.txt


  // Main control loop 
  bool oriented = false;
  bool atCurrDest = false;
  bool blocked = false;
  double laserFront;
  while(true) 
    {    
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);
      // Print data on the robot to the terminal
      printRobotData(bp, pose);
      // Print information about the laser. Check the counter first to stop
      // problems on startup
      if(counter > 2){
	      printLaserData(sp);
        
        laserFront = GetFrontLaserData(sp);
        std::cout << "00000000 " << laserFront << std::endl;
          if(laserFront < 2){
            std::cout << "BLOCKED" << std::endl;
            blocked = true;
          } else 
            blocked = false;
      }

      // Print data on the robot to the terminal --- turned off for now.
      // printRobotData(bp, pose);
      // If either bumper is pressed, stop. Otherwise just go forwards
     
      double destx = plan[planPos];
      double desty = plan[planPos+1];
      double m = (desty - pose.py)/(destx - pose.px);
      double ang = atan(m);
      double xdiff = fabs(destx - pose.px); // michele: moved up
      double ydiff = fabs(desty - pose.py); // michele: moved up 
      double distance = GetDistance(xdiff, ydiff); // michele made
	
      std::cout << "Oriented " << oriented << std::endl;
      std::cout << "Destination " <<  destx << "," << desty << std::endl;
      std::cout << "angle to reach: " << ang << std::endl;
      std::cout << "Distance: " << distance << std::endl;
      
      // I think we need to rework the code
      // Check if line of sight with destination 
      // If angle is matching, and distance from 
      // destination (distance) is less than the 
      // front lasers range (laserFront)
      // then go with Phil's original code
      // But if there is no line of sight, and something 
      // is right in front, then we need to move around
      // until John Connor has line of sight with the
      // destination, at which point we revert to Phil's
      // code. 

	/* Dins:
		*On Bump
		*Retreat
		*If blocked on left side - turn right
			stop turning when MinRight() > 1
			go straight
			reestablish line of sight and go to goal
		*If blocked on right side - turn right
			stop turning when MinLeft() > 1
			go straight
			reestablish line of sight and go to goal
		
	*/
      if(bp[0] || bp[1]){
      	speed= -0.1;
      	turnrate= 0;
      	std::cout << "BUMPER HIT " << std::endl;
	
	if(sp.MinLeft() < 0.7){
		turnrate = 0.1;
		if(sp.MinRight() > 1.7){
			turnrate = 0;
			speed = 0.2;
		}
		/*
		if(lineofsight){

		}*/
	
	}
	else if(sp.MinRight() < 0.7){
		turnrate = -0.1;
		if(sp.MinLeft() > 1.7){
			turnrate = 0; 
			speed = 0.2;
		}
		/*
		if(lineofsight){
		
		}*/
	}
      } else {
	    //correct so if both are negative it doesnt sum to a large number when subracting
// did it take that path before? 
      	if (blocked || (distance > laserFront)){
          turnrate = 0.2;
          speed = .5; 
          oriented = false;
          std::cout << distance << " !!! " << laserFront << std::endl;
        } else{
          if(ang < 0)
            ang = fabs(ang);
            angleDiff = fabs(ang - pose.pa);
            std::cout << "angle diff: " << angleDiff << std::endl;
          if(angleDiff < .01){
            oriented = true;
          }
          else{
             turnrate = 0.1;
             speed = 0.0;
          }}
      }     
      //moving from things right in front
      /*if(front < 1.5){
        turnrate = 0.1;
        speed = 0.5;
      }*/
      if(oriented == true){
      	speed = .5;
      	turnrate = 0.0;
      }
      
      
      std::cout << "x diff: " << xdiff << " y diff: " << ydiff << std::endl;
      if((xdiff < 0.1) && (ydiff < 0.1)){
      	oriented = false;
      	planPos += 2;
      }

      // What are we doing?
      std::cout << "Speed: " << speed << std::endl;      
      std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      counter++;
    }
  
} // end of main()

/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose. 
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp)
{

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  if(hCount > 0){
    hypothesis = lp.GetHypoth(0);
    pose       = hypothesis.mean;
  }

  return pose;
} // End of readPosition()

void printLaserData(LaserProxy& sp)
{

  double maxRange, minLeft, minRight, range, bearing;
  int points;

  maxRange  = sp.GetMaxRange();
  minLeft   = sp.MinLeft();
  minRight  = sp.MinRight();
  range     = sp.GetRange(5);
  bearing   = sp.GetBearing(5);
  points    = sp.GetCount();

  //Uncomment this to print out useful laser data
  std::cout << "Laser says..." << std::endl;
  std::cout << "Maximum distance I can see: " << maxRange << std::endl;
  std::cout << "Number of readings I return: " << points << std::endl;
  std::cout << "Closest thing on left: " << minLeft << std::endl;
  std::cout << "Closest thing on right: " << minRight << std::endl;
  std::cout << "Range of a single point: " << range << std::endl;
  std::cout << "Bearing of a single point: " << bearing << std::endl;


  return;
} // End of printLaserData()

// Returns data from front laser
double GetFrontLaserData(LaserProxy& sp)
{
  double front;
  front = sp.GetRange(180);
  return front;
}

// Returns distance from destination 
// pythagorean 
double GetDistance(double x, double y){
  double distance;
  distance = sqrt((x*x)+(y*y));
  return distance;
}

/**
 *  printRobotData
 *
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 *
 **/

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{

  // Print out what the bumpers tell us:
  std::cout << "Left  bumper: " << bp[0] << std::endl;
  std::cout << "Right bumper: " << bp[1] << std::endl;
  // Can also print the bumpers with:
  //std::cout << bp << std::endl;

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;

  
} // End of printRobotData()

/**
 * readPlanLength
 *
 * Open the file plan.txt and read the first element, which should be
 * an even integer, and return it.
 *
 **/

int readPlanLength(void)
{
  int length;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> length;
  planFile.close();

  // Some minimal error checking
  if((length % 2) != 0){
    std::cout << "The plan has mismatched x and y coordinates" << std::endl;
    exit(1);
  }

  return length;

} // End of readPlanLength

/**
 * readPlan
 *
 * Given the number of coordinates, read them in from plan.txt and put
 * them in the array plan.
 *
 **/

void readPlan(double *plan, int length)
{
  int skip;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> skip;
  for(int i = 0; i < length; i++){
    planFile >> plan[i];
  }

  planFile.close();

} // End of readPlan

/**
 * printPlan
 *
 * Print the plan on the screen, two coordinates to a line, x then y
 * with a header to remind us which is which.
 *
 **/

void printPlan(double *plan , int length)
{
  std::cout << std::endl;
  std::cout << "   x     y" << std::endl;
  for(int i = 0; i < length; i++){
    std::cout.width(5);
    std::cout << plan[i] << " ";
    if((i > 0) && ((i % 2) != 0)){
      std::cout << std::endl;
    }
  }
  std::cout << std::endl;

} // End of printPlan


/**
 * writePlan
 * 
 * Send the plan to the file plan-out.txt, preceeded by the information
 * about how long it is.
 *
 **/

void writePlan(double *plan , int length)
{
  std::ofstream planFile;
  planFile.open("plan-out.txt");

  planFile << length << " ";
  for(int i = 0; i < length; i++){
    planFile << plan[i] << " ";
  }

  planFile.close();

} // End of writePlan
