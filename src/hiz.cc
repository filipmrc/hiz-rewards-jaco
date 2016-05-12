#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <wpi_jaco_msgs/HomeArmAction.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/terminal_state.h>

using namespace std;

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
  actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> acHome;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> acLift;

  vector<wpi_jaco_msgs::CartesianCommand> states;

public:
  ArmFsm(ros::NodeHandle n) : acGripper("jaco_arm/manipulation/gripper", true),
			      acHome("jaco_arm/home_arm", true),
			      acLift("jaco_arm/manipulation/lift", true)
  {
    client_cartesian = n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>("get_cartesian_position");
    cartesian_cmd_pub = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd",1);
    states.resize(8);

    ifstream fin("states.txt");
    if(!fin)
       perror ( "Stream Failed to open because: " );

    string name;
    int x, y, z, rx, ry, rz, i=0;

    while (fin >> name >> x >> y >> z >> rx >> ry >> rz)
    {
        cout << name << " " << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << "\n";
        states[i].arm.linear.x = x;
        states[i].arm.linear.y = y;
        states[i].arm.linear.z = z;
        states[i].arm.angular.x = rx;
        states[i].arm.angular.y = ry;
        states[i].arm.angular.z = rz;
	i++;
    }

    ROS_INFO("Waiting for grasp, pickup, and home arm action servers...");
    acGripper.waitForServer();
    acHome.waitForServer();
    ROS_INFO("Finished waiting for action servers");

  }
  void resetState(wpi_jaco_msgs::CartesianCommand &cmd, int &state)
  {
    cmd = states[0];
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

  bool checkStatus(wpi_jaco_msgs::CartesianCommand cmd)
  {
    return 1;
  }

  void get_position(geometry_msgs::Twist& position)
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
    cmd.position = true;
    cmd.armCommand = true;
    cmd.fingerCommand = false;
    cmd.repeat = false;
    cartesian_cmd_pub.publish(cmd);
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

  void liftLoad()
  {
    rail_manipulation_msgs::LiftGoal liftGoal;
    acLift.sendGoal(liftGoal);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_fsm");
  ros::NodeHandle n;
  ros::Rate r(1);

  wpi_jaco_msgs::CartesianCommand cmd;
  ArmFsm ar(n);

  int state = 0;

  while(ros::ok())
  {
    int c = getch();
    if ((state != 6) && (state != 0))
      {
	if(ar.checkStatus(cmd))
	{
	  ar.forwardState(cmd, state);
	}
        ar.executeCommand(cmd);
      }
    else if ((c == 'r') && (state == 6))
      {
	if(ar.checkStatus(cmd))
	  {
	    ar.resetState(cmd, state);
	  }
      }
    else if ((c == 'r') && (state == 0))
      {
	ar.resetState(cmd, state);
      }

    cout << endl << state;
    ros::spinOnce();
    r.sleep();
  }

return 0;
}
