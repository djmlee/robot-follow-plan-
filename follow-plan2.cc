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
using namespace PlayerCc;  
 
 
/**
 * Function headers
 *
 **/
 
player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);
 
int  readPlanLength(void);
void readPlan(double *, int);
void printPlan(double *,int);  
void writePlan(double *, int);
 
struct vector{
    double x;
    double y;
};
 
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
  double *plan;
 
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
 
 
double targetx = 0;
double targety = 0;
double targeta = 0;
bool targeta_reached = false;
bool waypoint_reached = false;
double initialy = 0;
double initialx = 0;
double distance = 0;
bool finished = false;
vector A;
vector B;
  // Main control loop
  while(!finished)
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
      }
 
      // Print data on the robot to the terminal --- turned off for now.
      // printRobotData(bp, pose);
     
      // If either bumper is pressed, stop. Otherwise just go forwards
 
      if(bp[0] || bp[1]){
            //speed= 0;
            //turnrate= 0;
            for(int i = 0; i < 10; i++){
                turnrate = 0;
                speed = -0.7;
                pp.SetSpeed(speed, turnrate);
            }
           
           
      }
      else {
            /*
            for i in waypoints
            go to waypoint
            */
            for(int i = 0; i < pLength; i = i + 2){
                waypoint_reached = false;
                targetx = plan[i];
                targety = plan[i+1];
                while(!waypoint_reached){
                    // if it hit something
                    if(bp[0] || bp[1]){
                        int i_f = 20;
                        // back up
                        for(int i = 0; i < i_f/3; i++){
                            robot.Read();
                            turnrate = 0;
                            speed = -0.5;
                            pp.SetSpeed(speed, turnrate);
                            std::cout << "i #1" << "\n";
                        }
                        // turn around
                        for(int i = 0; i < i_f; i++){
                            robot.Read();
                            turnrate = 0.5;
                            speed = 0;
                            pp.SetSpeed(speed, turnrate);
                            std::cout << "i #2" << "\n";
                        }
                        // do a semicircle
                        for(int i = 0; i < i_f; i++){
                            robot.Read();
                            turnrate = -0.8;
                            speed = 0.8;
                            pp.SetSpeed(speed, turnrate);
                            std::cout << "i #3" << "\n";
                        }
                       
                       
                       
                    }
                      // Update information from the robot.
                      robot.Read();
                      // Read new information about position
                      pose = readPosition(lp);
                      // Print data on the robot to the terminal
                      printRobotData(bp, pose);
                    initialx = pose.px;
                    initialy = pose.py;
                   
                   
                    /*
                    turn until we point in the right direction
                    use proportional control to go straight until waypoint is reached
                    */
                   
                    //targeta = std::atan(std::abs(targety-initialy)/std::abs(targetx-initialx));
                    targeta = std::acos((A.x*B.x+A.y*B.y)/(sqrt(A.x*A.x+A.y*A.y)*sqrt(B.x*B.x+B.y*B.y)));
                    // targeta is the relative angular displacement
                    distance = std::sqrt((initialx-targetx)*(initialx-targetx)+
                                         (initialy-targety)*(initialy-targety));
                    //targeta_reached = !(pose.pa > targeta + 0.15 || pose.pa < targeta - 0.15);
                    targeta_reached = targeta < 0.05 ;
                    A.x = std::cos(pose.pa);
                    A.y = std::sin(pose.pa);
                    B.x = targetx - initialx;
                    B.y = targety - initialy;          
                   
                    if(!targeta_reached){
                        //turnrate = pose.pa-targeta < 0 ? 0.25 : -0.25;
                        turnrate = A.x*B.y - B.x*A.y > 0 ? 0.25 : -0.25;
                        speed = 0;
                    } else {
                        turnrate = 0;
                        // PROPORTIONAL CONTROL
                        speed = distance > 1 ? distance / 2 : 0.7;        
                    }
                    waypoint_reached = std::abs(initialx-targetx)<0.3 && std::abs(initialy-targety)<0.3;
                   
                    // What are we doing?
                    std::cout << "Speed: " << speed << std::endl;      
                    std::cout << "Turn rate: " << turnrate << "\n";
                    std::cout << "TargetX: " << targetx << "\n";
                    std::cout << "TargetY: " << targety << "\n";
                    std::cout << "TargetA: " << targeta*180/3.14 << "\n\n";
                   
                   
                    counter++;
                    pp.SetSpeed(speed, turnrate);  
                } // end of while
            } // end of for
           
            /*stop*/
     
            //speed=0;
            //turnrate = 0;          
           
      }    
     
//   x     y
// -2.5    -6
// -2.5   1.5
// -1.5   2.5
//  2.5   3.5
//  6.5   6.5
 
     
 
      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      counter++;
     
      finished = true;
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
  //std::cout << "Laser says..." << std::endl;
  //std::cout << "Maximum distance I can see: " << maxRange << std::endl;
  //std::cout << "Number of readings I return: " << points << std::endl;
  //std::cout << "Closest thing on left: " << minLeft << std::endl;
  //std::cout << "Closest thing on right: " << minRight << std::endl;
  //std::cout << "Range of a single point: " << range << std::endl;
  //std::cout << "Bearing of a single point: " << bearing << std::endl;
 
  return;
} // End of printLaserData()
 
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
  std::cout << "A: " << pose.pa*180/3.14 << std::endl;
 
 
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
