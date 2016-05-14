#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/terminal_state.h>

#define THRESHOLD 0.005

using namespace std;

// TODO more robust key input method
char getch()
{
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios old = {0};
  if (tcgetattr(filedesc, &old) < 0)
    ROS_ERROR("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &old) < 0)
    ROS_ERROR("tcsetattr ICANON");

  if(rv == -1)
    ROS_ERROR("select");
  else if(rv == 0);
  else
    read(filedesc, &buff, len );

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
    ROS_ERROR ("tcsetattr ~ICANON");
  return (buff);
}

class ArmFsm
{
private:
  ros::Publisher cartesian_cmd_pub;
  ros::ServiceClient client_cartesian;
  wpi_jaco_msgs::GetCartesianPosition srv;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> acGripper;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> acLift;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> acTraj;

  vector<wpi_jaco_msgs::CartesianCommand> states;

public:
  ArmFsm(ros::NodeHandle n) : acGripper("jaco_arm/manipulation/gripper", true),
  acLift("jaco_arm/manipulation/lift", true),
  acTraj("jaco_arm/arm_controller/trajectory", true)
{
    client_cartesian = n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>("get_cartesian_position");
    cartesian_cmd_pub = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd",1);
    states.resize(8);

    ifstream fin("states.txt");
    if(!fin)
      perror ( "Stream Failed to open because: " );

    string name;
    int x, y, z, rx, ry, rz, lft, i=0;

    while (fin >> name >> x >> y >> z >> rx >> ry >> rz >> lft)
      {
	cout << name << " " << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << lft <<"\n";
	states[i].arm.linear.x = x;
	states[i].arm.linear.y = y;
	states[i].arm.linear.z = z;
	states[i].arm.angular.x = rx;
	states[i].arm.angular.y = ry;
	states[i].arm.angular.z = rz;
	states[i].fingerCommand = lft;
	i++;
      }

    ROS_INFO("Waiting for grasp, pickup, and home arm action servers...");
    acGripper.waitForServer();
    acLift.waitForServer();
    ROS_INFO("Finished waiting for action servers");

}
  void resetState(wpi_jaco_msgs::CartesianCommand &cmd, int &state)
  {
    cmd = states[0]; // Idle
    state = 1;
  }

  void forwardState(wpi_jaco_msgs::CartesianCommand &cmd, int &state)
  {
    switch(state)
    {
      case 1:
	state++;
	cmd = states[1]; // Above package
	break;
      case 2:
	state++;
	cmd = states[2]; // Surround package with gripper
	break;
      case 3:
	state++;
	cmd = states[3]; // Grip package
	break;
      case 4:
	state++;
	cmd = states[4]; // Lift package
	break;
      case 5:
	state++;
	cmd = states[5]; // Bring package to other side
	break;
      case 6:
	state++;
	cmd = states[6]; // Drop package
    }
  }

  bool checkStatus(wpi_jaco_msgs::CartesianCommand cmd, int state)
  {
    int type = cmd.fingerCommand;
    bool finished_before_timeout;
    geometry_msgs::Twist current, goal = cmd.arm;

    //return true; TESTING
    switch(type)
    {
      case 0:
	getPosition(current);
	if((abs(goal.linear.x - current.linear.x) < THRESHOLD) && (abs(goal.linear.y - current.linear.y) < THRESHOLD)
	    && (abs(goal.linear.z - current.linear.z) < THRESHOLD) && (abs(goal.angular.x - current.angular.x) < THRESHOLD)
	    && (abs(goal.angular.y - current.angular.y) < THRESHOLD) && (abs(goal.angular.z - current.angular.z) < THRESHOLD))
	  {
	    ROS_INFO("REACHED STATE %d", state);
	    return true;
	  }
	else
	  {
	    return false;
	  }
      case 1:
	finished_before_timeout = acGripper.waitForResult(ros::Duration(10.0));
	if(finished_before_timeout == true)
	  {
	    ROS_INFO_STREAM("Grip successful");
	    return true;
	  }
	else
	  {
	    ROS_INFO_STREAM("GRIP FAILED");
	    return false;
	  }
      case 2:
	finished_before_timeout = acLift.waitForResult(ros::Duration(10.0));
	if(finished_before_timeout == true)
	  {
	    ROS_INFO_STREAM("Lift successful");
	    return true;
	  }
	else
	  {
	    ROS_INFO_STREAM("LIFT FAILED");
	    return false;
	  }
      case 3:
	finished_before_timeout = acGripper.waitForResult(ros::Duration(10.0));
	if(finished_before_timeout == true)
	  {
	    ROS_INFO_STREAM("Release successful");
	    return true;
	  }
	else
	  {
	    ROS_INFO_STREAM("RELEASE FAILED");
	    return false;
	  }
    }
    return false;
  }

  void getPosition(geometry_msgs::Twist& position)
  {
    if (client_cartesian.call(srv))
      {
	position  = srv.response.pos;
      }
    else
      {
	ROS_ERROR("Failed to get current position!");
	ros::shutdown();
	throw ros::Exception("Failed to get current position!");
      }
  }

  void executeCommand(wpi_jaco_msgs::CartesianCommand cmd)
  {
    int type = cmd.fingerCommand;
    switch(type)
    {
      case 0:
	cmd.position = true;
	cmd.armCommand = true;
	cmd.fingerCommand = false;
	cmd.repeat = false;
	cartesian_cmd_pub.publish(cmd);
	break;
      case 1:
	executeGrip();
	break;
      case 2:
	executeLift();
	break;
      case 3:
	releaseGrip();
	break;
    }
  }

  void executeGrip()
  {
    rail_manipulation_msgs::GripperGoal gripperGoal;
    gripperGoal.close = true;
    acGripper.sendGoal(gripperGoal);
  }

  void releaseGrip()
  {
    rail_manipulation_msgs::GripperGoal gripperGoal;
    gripperGoal.close = false;
    acGripper.sendGoal(gripperGoal);
  }

  void executeLift()
  {
    rail_manipulation_msgs::LiftGoal liftGoal;
    acLift.sendGoal(liftGoal);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_fsm");
  ros::NodeHandle n;
  ros::Rate r(2);

  wpi_jaco_msgs::CartesianCommand cmd;
  ArmFsm ar(n);

  int state = 0;
  ROS_INFO_STREAM(state);

  while(ros::ok())
    {
      int c = getch();
      if ((state < 7) && (state > 1)) // If in cycle
	{
	  if(ar.checkStatus(cmd,state)) // Check if state transition finished
	    {
	      if((state == 3 || state == 4 || state == 6) && (c == 'r')) // If grip, lift or release
		{
		  ar.forwardState(cmd, state); // Set next state
		}
	      else if((state != 3) && (state != 4) && (state != 6))
		{
		  ar.forwardState(cmd, state); // Set next state
		}
	      ROS_INFO_STREAM(state);
	    }
	  ar.executeCommand(cmd); // Send old or new command
	}
      else if ((c == 'r') && (state == 7)) // If releasing package and user input
	{
	  if(ar.checkStatus(cmd,state)) // Check if package released
	    {
	      ar.resetState(cmd, state); // Back to idle
	      ROS_INFO_STREAM(state);
	    }
	}
      else if ((c == 'r') && (state == 0)) // Check if started and input
	{
	  ar.resetState(cmd, state); // Set to idle
	  //ROS_INFO("setting to idle");
	  ROS_INFO_STREAM(state);
	}
      else if ((c == 'r') && (state == 1)) // Check if idle and input
	{
	  if(ar.checkStatus(cmd,state)) // Check if idle
	    {
	      ar.forwardState(cmd, state); // Start cycle
	      //ROS_INFO("starting cycle");
	      ROS_INFO_STREAM(state);
	    }
	}
      ros::spinOnce();
      r.sleep();
    }

  return 0;
}
