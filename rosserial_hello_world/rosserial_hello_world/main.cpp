#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <windows.h>
#include <opencv2\imgproc.hpp>	//opencv头文件
#include <opencv2\calib3d.hpp>
#include <opencv2\highgui.hpp>

using namespace cv;
using namespace std;

#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1:0) //必要的，我是背下来的

bool flag = false;

using std::string;


int main()
{
	ros::NodeHandle nh;
	char *ros_master = "192.168.137.5";

	printf("Connecting to server at %s\n", ros_master);
	nh.initNode(ros_master);

	printf("Advertising cmd_vel message\n");
	geometry_msgs::Twist twist_msg;
	ros::Publisher cmd_vel_pub("cmd_vel", &twist_msg);
	nh.advertise(cmd_vel_pub);

	printf("Go robot go!\n");
	while (1)
	{
		if (KEY_DOWN('W'))
		{
			twist_msg.linear.x = 0.5;
			twist_msg.linear.y = 0;
			twist_msg.linear.z = 0;
			twist_msg.angular.x = 0;
			twist_msg.angular.y = 0;
			twist_msg.angular.z = 0;
		}
		else if (KEY_DOWN('S'))
		{
			twist_msg.linear.x = -0.5;
			twist_msg.linear.y = 0;
			twist_msg.linear.z = 0;
			twist_msg.angular.x = 0;
			twist_msg.angular.y = 0;
			twist_msg.angular.z = 0;
		}
		else if (KEY_DOWN('A'))
		{
			twist_msg.linear.x = 0;
			twist_msg.linear.y = 0;
			twist_msg.linear.z = 0;
			twist_msg.angular.x = 0;
			twist_msg.angular.y = 0;
			twist_msg.angular.z = 0.5;
		}
		else if (KEY_DOWN('D'))
		{
			twist_msg.linear.x = 0;
			twist_msg.linear.y = 0;
			twist_msg.linear.z = 0;
			twist_msg.angular.x = 0;
			twist_msg.angular.y = 0;
			twist_msg.angular.z = -0.5;
		}
		else 
		{
			twist_msg.linear.x = 0;
			twist_msg.linear.y = 0;
			twist_msg.linear.z = 0;
			twist_msg.angular.x = 0;
			twist_msg.angular.y = 0;
			twist_msg.angular.z = 0;
		}

		cmd_vel_pub.publish(&twist_msg);

		nh.spinOnce();
		Sleep(100);
	}

	printf("All done!\n");
	return 0;
}