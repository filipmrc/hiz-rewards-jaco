#include <ros/ros.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <iostream>
#include <fstream>
#include <termios.h>
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

public:
  ArmFsm(ros::NodeHandle n)
  {
    client_cartesian = n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>("get_cartesian_position");
    cartesian_cmd_pub = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd",1);

    ifstream fin("states.txt");
    if(!fin)
       perror ( "Stream Failed to open because: " );

    string name;
    int var1;
    int var2;
    int var3;

    while (fin >> name >> var1 >> var2 >> var3)
    {
        /* do something with name, var1 etc. */
        cout << name << var1 << var2 << var3 << "\n";
    }
  }
  void resetState(wpi_jaco_msgs::CartesianCommand &cmd, int &state)
  {
    cmd.armCommand = true;
    cmd.fingerCommand = false;
    cmd.repeat = false;
    cmd.position = true;
    cmd.arm.linear.x = 1;
    cmd.arm.linear.y = 1;
    cmd.arm.linear.z = 1;
    cmd.arm.angular.x = 1;
    cmd.arm.angular.y = 1;
    cmd.arm.angular.z = 1;

    state = 1;
  }

  void forwardState(wpi_jaco_msgs::CartesianCommand &cmd, int &state)
  {
    switch(state)
      {
	case 1:
	  state++;
	  break;
	case 2:
	  state++;
	  break;
	case 3:
	  state++;
	  break;
	case 4:
	  state++;
	  break;
      }
  }

  bool checkStatus(wpi_jaco_msgs::CartesianCommand cmd)
  {
    return 1;
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
    if ((state != 5) && (state != 0))
      {
	if(ar.checkStatus(cmd))
	{
	  ar.forwardState(cmd, state);
	}
      }
    else if ((c == 'r') && (state == 5))
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
