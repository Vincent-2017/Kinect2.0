#include "draw.h"

void    draw(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper)
{
	//用两个关节点来做线段的两端，并且进行状态过滤
	if (r_1.TrackingState == TrackingState_Tracked && r_2.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint t_point;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点
		Point   p_1, p_2;
		myMapper->MapCameraPointToColorSpace(r_1.Position, &t_point);
		p_1.x = t_point.X;
		p_1.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(r_2.Position, &t_point);
		p_2.x = t_point.X;
		p_2.y = t_point.Y;

		line(img, p_1, p_2, Vec3b(0, 255, 0), 5);
		circle(img, p_1, 10, Vec3b(255, 0, 0), -1);
		circle(img, p_2, 10, Vec3b(255, 0, 0), -1);
	}
}

void    drawhandstate(Mat & img, Joint & lefthand, Joint & righthand, IBody* myBodyArr, ICoordinateMapper * myMapper)
{
	if (lefthand.TrackingState == TrackingState_Tracked && righthand.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint l_point, r_point;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点
		Point   p_l, p_r;
		myMapper->MapCameraPointToColorSpace(lefthand.Position, &l_point);
		p_l.x = l_point.X;
		p_l.y = l_point.Y;
		myMapper->MapCameraPointToColorSpace(righthand.Position, &r_point);
		p_r.x = r_point.X;
		p_r.y = r_point.Y;
		cout << "p_l:" << p_l.x << "," << p_l.y << " ";
		cout << "p_r:" << p_r.x << "," << p_r.y << endl;

		//char* str1 = NULL, str2 = NULL;
		cvPutText(&IplImage(img), "LeftHand", Point(p_l.x + 50, p_l.y - 50), &font, CV_RGB(255, 0, 0));//红色字体注释
		cvPutText(&IplImage(img), "RightHand", Point(p_r.x + 50, p_r.y - 50), &font, CV_RGB(255, 0, 0));//红色字体注释

		HandState left;
		myBodyArr->get_HandLeftState(&left);
		HandState right;
		myBodyArr->get_HandRightState(&right);
		switch (left)
		{
		case HandState_Closed:
			circle(img, p_l, 10, Scalar(0, 0, 255, 1), 20); break;
		case HandState_Open:
			circle(img, p_l, 10, Scalar(0, 255, 0, 1), 20); break;
		case HandState_Lasso:
			circle(img, p_l, 10, Scalar(255, 0, 0, 1), 20); break;
		default:
			break;
		}
		switch (right)
		{
		case HandState_Closed:
			circle(img, p_r, 10, Scalar(0, 0, 255, 1), 20); break;
		case HandState_Open:
			circle(img, p_r, 10, Scalar(0, 255, 0, 1), 20); break;
		case HandState_Lasso:
			circle(img, p_r, 10, Scalar(255, 0, 0, 1), 20); break;
		default:
			break;
		}
	}
}