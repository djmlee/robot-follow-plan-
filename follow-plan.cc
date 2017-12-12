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
using namespace PlayerCc;  


/**
 * Function headers
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);
double GetFrontLaserData(LaserProxy& sp);
double GetSoftLeftLaserData(LaserProxy& sp);
double GetSoftRightLaserData(LaserProxy& sp);
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
	bool blocked = false; //blocked is LaserFront > 2
  bool collided = false; //collided is LaserFront < 0.5
	double laserFront, laserLeft, laserRight;
  while(true) 
    {    
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);
      // Print information about the laser. Check the counter first to stop
      // problems on startup
      //if(counter > 2) printLaserData(sp);
    
		laserLeft = GetSoftLeftLaserData(sp);
		laserRight = GetSoftRightLaserData(sp);
		laserFront = GetFrontLaserData(sp);

	//if laserFront has obstacle in front, cout BLOCKED
		if(laserFront < 2){
      std::cout << "!BLOCKED!" << std::endl;
      blocked = true;
      if(laserFront < 0.2) { collided = true; }
    } else {blocked = false; collided = false;}


      // printRobotData(bp, pose);
     
      double destx = plan[planPos];
      double desty = plan[planPos+1];
      double m = (desty - pose.py)/(destx - pose.px);
      double ang = atan(m);

      double xdiff = fabs(destx - pose.px);
      double ydiff = fabs(desty - pose.py);  
      double distance = GetDistance(xdiff, ydiff); // michele made
/*
      std::cout << "Oriented " << oriented << std::endl;
      std::cout << "Destination " <<  destx << "," << desty << std::endl;
      std::cout << "angle to reach: " << ang << std::endl;    
      std::cout << "Distance: " << distance << std::endl;
*/

	/* Dins:
		*On Bump
		*Retreat
		*If blocked on left side - turn right
			stop turning when laserRight() > 1
			go straight
			reestablish line of sight and go to goal
		*If blocked on right side - turn right
			stop turning when laserLeft() > 1
			go straight
			reestablish line of sight and go to goal
		
	*/
      if(bp[0] || bp[1]){
        // to send commands to robot pp.SetSpeed(speed, turnrate);  
      	speed = 0;
      	turnrate= 0;
        collided = true;
        pp.SetSpeed(speed, turnrate);
      	std::cout << "BUMPER HIT " << std::endl;

      //Lasers and obstacles
    		if(laserLeft < 1.0 && laserRight < 1.0){
    		  if(blocked){ 
            //infinite loop
            //while path is blocked, move back until laserfront >2
            std::cout <<"LASERFRONT<2moving backwards"<< std::endl;
    			  speed= -1;
            pp.SetSpeed(speed, turnrate);
            if(laserFront > 1.0) {std::cout <<"BLOCK IS NOW FALSE" << std::endl; blocked = false; collided = false;}
    			}
          if(!collided){ 
            std::cout <<"NOT BLOCKED/COLLIDED - NOW TURNING"<< std::endl;
      			turnrate = 0.1;
            speed = 0;
            pp.SetSpeed(speed, turnrate);

            if(laserFront > 5) { std::cout <<"FRONT CLEAR. SPEEDING UP"<< std::endl; turnrate = 0; speed = 0.4; pp.SetSpeed(speed,turnrate);}
            std::cout << "DISTANCE! " << distance <<std::endl;
            if(distance > 13){
                        std::cout <<"NOW TURNING AFTER DISTANCE>13"<< std::endl;
              speed = 0; 
              turnrate = 0.1; 
              pp.SetSpeed(speed,turnrate);
              if(oriented == true){ turnrate = 0; speed = 0.4; pp.SetSpeed(speed,turnrate); }
            }//distance>13
          }//blocked=false
    		}/* laser if */ else { std::cout << "ELSE" << std::endl; 
if(!collided){ 
            std::cout <<"NOT BLOCKED/COLLIDED - NOW TURNING"<< std::endl;
            turnrate = 0.1;
            speed = 0;
            pp.SetSpeed(speed, turnrate);

            if(laserFront > 5) { std::cout <<"FRONT CLEAR. SPEEDING UP"<< std::endl; turnrate = 0; speed = 0.4; pp.SetSpeed(speed,turnrate);}
            std::cout << "DISTANCE! " << distance <<std::endl;
            if(distance > 13){
                        std::cout <<"NOW TURNING AFTER DISTANCE>13"<< std::endl;
              speed = 0; 
              turnrate = 0.1; 
              pp.SetSpeed(speed,turnrate);
              if(oriented == true){ turnrate = 0; speed = 0.4; pp.SetSpeed(speed,turnrate); }
            }//distance>13
          }//blocked=false
        }//else 


/*
		{
		turnrate = -0.1;
		if(bp[0] || bp[1]) speed = -0.5;
		if(laserFront > 5) turnrate = 0; 
		//speed=0.2;

			if(distance > 13)
			{
			turnrate = 0.1;
				if(oriented == true)
				{
				turnrate = 0;
				speed = 0.4;
				}
			}//if end
		}//else end
    */
}
      else { 
       //correct so if both are negative it doesnt sum to a large number when subracting
      	if(ang < 0) { ang = fabs(ang); }
      	
        angleDiff = fabs(ang - pose.pa);
      	std::cout << "angle diff: " << angleDiff << std::endl;
      	
        if(angleDiff < .01) oriented = true;
      	else{
      	 turnrate = 0.1;
      	 speed = 0.0;
      	}//else
      }//else   

      if(!collided && oriented == true && !bp[0] && !bp[1]){
        std::cout << " !!!PROCEEDING AS NORMAL!!! " << std::endl;
      	speed = 0.4;
      	turnrate = 0.0;
      }

      std::cout << "x diff: " << xdiff << " y diff: " << ydiff << std::endl;
      if((xdiff < 0.1) && (ydiff < 0.1)){
      	std::cout << "At Position" << std::endl;
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

player_pose2d_t readPosition(LocalizeProxy& lp){

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  if(hCount > 0)
  {
    hypothesis = lp.GetHypoth(0);
    pose       = hypothesis.mean;
  }

  return pose;
} // End of readPosition()


void printLaserData(LaserProxy& sp)
{
  //Uncomment this to print out useful laser data
  std::cout << "Laser says..." << std::endl;
  std::cout << "Maximum distance I can see: " << sp.GetMaxRange() << std::endl;
  std::cout << "Number of readings I return: " << sp.GetCount() << std::endl;
  std::cout << "Closest thing on left: " << sp.MinLeft() << std::endl;
  std::cout << "Closest thing on right: " << sp.MinRight() << std::endl;
  std::cout << "Range of a single point: " << sp.GetRange(5) << std::endl;
  std::cout << "Bearing of a single point: " << sp.GetBearing(5) << std::endl;

  return;
} // End of printLaserData()

// Returns data from front laser
double GetFrontLaserData(LaserProxy& sp)
{
  double front;
  front = sp.GetRange(180);
  return front
;}

double GetSoftLeftLaserData(LaserProxy& sp)
{
  double softleft;
  softleft = sp.GetRange(225); //diagonally left
  return softleft;
}

double GetSoftRightLaserData(LaserProxy& sp)
{
  double sright;
  sright = sp.GetRange(135); //diagonally right
  return sright;
}
// Returns distance from destination 
// pythagorean 
double GetDistance(double x, double y)
{
  double distance;
  distance = sqrt((x*x)+(y*y));
  return distance;
}

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
