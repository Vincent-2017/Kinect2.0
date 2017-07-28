#include "util.h"
// 将关节点映射到彩色或者深度图像
// USEUSEColorSpace USEDepthSpace
Point MapCameraPointToSomeSpace(ICoordinateMapper * myMapper, Joint & joint, int SpaceFlag)
{
	Point   p;
	if (SpaceFlag == USEColorSpace)
	{
		ColorSpacePoint colorpoint;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点
		myMapper->MapCameraPointToColorSpace(joint.Position, &colorpoint);
		p.x = colorpoint.X;
		p.y = colorpoint.Y;
	}
	if (SpaceFlag == USEDepthSpace)
	{
		DepthSpacePoint depthpoint;    //要把关节点用的摄像机坐标下的点转换成深度空间的点
		myMapper->MapCameraPointToDepthSpace(joint.Position, &depthpoint);
		p.x = depthpoint.X;
		p.y = depthpoint.Y;
	}
	return p;
}

// 在映射的图像上画骨骼
void    drawline(Mat & img, Joint & joint0, Joint & joint1, ICoordinateMapper * myMapper, int SpaceFlag)
{
	TrackingState joint0State = joint0.TrackingState;
	TrackingState joint1State = joint1.TrackingState;

	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))		return;
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))	return;
	Point	p1 = MapCameraPointToSomeSpace(myMapper, joint0, SpaceFlag),
		p2 = MapCameraPointToSomeSpace(myMapper, joint1, SpaceFlag);
	circle(img, p1, 10 / SpaceFlag, Vec3b(255, 0, 0), -1);
	circle(img, p2, 10 / SpaceFlag, Vec3b(255, 0, 0), -1);
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked)) //非常确定的骨架，用白色直线
	{
		line(img, p1, p2, cvScalar(255, 255, 255));
	}
}

// 在映射的图像上画手的状态
void    drawhandstate(Mat & img, Joint & lefthand, Joint & righthand, IBody* myBodyArr, ICoordinateMapper * myMapper, int SpaceFlag)
{
	if (lefthand.TrackingState == TrackingState_Tracked)
	{
		Point   pl = MapCameraPointToSomeSpace(myMapper, lefthand, SpaceFlag);
		HandState left;
		myBodyArr->get_HandLeftState(&left);
		switch (left)
		{
		case HandState_Closed:
			circle(img, pl, 10, Scalar(0, 0, 255, 1), 20); 
			lhandstate = "Close";
			break;
		case HandState_Open:
			circle(img, pl, 10, Scalar(0, 255, 0, 1), 20); 
			lhandstate = "Open"; 
			break;
		case HandState_Lasso:
			circle(img, pl, 10, Scalar(255, 0, 0, 1), 20); 
			lhandstate = "Lasso"; 
			break;
		default:lhandstate = "Unknown";
			break;
		}
	}
	else
	{
		lhandstate = "NotTracked";
	}
	if (righthand.TrackingState == TrackingState_Tracked)
	{
		Point	pr = MapCameraPointToSomeSpace(myMapper, righthand, SpaceFlag);
		HandState right;
		myBodyArr->get_HandRightState(&right);
		switch (right)
		{
		case HandState_Closed:
			circle(img, pr, 10, Scalar(0, 0, 255, 1), 20); 
			rhandstate = "Close"; 
			break;
		case HandState_Open:
			circle(img, pr, 10, Scalar(0, 255, 0, 1), 20); 
			rhandstate = "Open"; 
			break;
		case HandState_Lasso:
			circle(img, pr, 10, Scalar(255, 0, 0, 1), 20); 
			rhandstate = "Lasso"; 
			break;
		default:lhandstate = "Unknown";
			break;
		}
	}
	else
	{
		rhandstate = "NotTracked";
	}
}

// 在映射的图像上画整个身体
void DrawBody(Mat & img, Joint *myJointArr, ICoordinateMapper * myMapper, int SpaceFlag)
{
	drawline(img, myJointArr[JointType_Head], myJointArr[JointType_Neck], myMapper, SpaceFlag);						// 头-颈
	drawline(img, myJointArr[JointType_Neck], myJointArr[JointType_SpineShoulder], myMapper, SpaceFlag);				// 颈-脊柱肩

	drawline(img, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderLeft], myMapper, SpaceFlag);		// 脊柱肩-左肩膀
	drawline(img, myJointArr[JointType_SpineShoulder], myJointArr[JointType_SpineMid], myMapper, SpaceFlag);			// 脊柱肩-脊柱中
	drawline(img, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderRight], myMapper, SpaceFlag);		// 脊柱肩-右肩膀

	drawline(img, myJointArr[JointType_ShoulderLeft], myJointArr[JointType_ElbowLeft], myMapper, SpaceFlag);			// 左肩膀-左手肘
	drawline(img, myJointArr[JointType_SpineMid], myJointArr[JointType_SpineBase], myMapper, SpaceFlag);				// 脊柱中-脊柱底
	drawline(img, myJointArr[JointType_ShoulderRight], myJointArr[JointType_ElbowRight], myMapper, SpaceFlag);		// 右肩膀-右手肘

	drawline(img, myJointArr[JointType_ElbowLeft], myJointArr[JointType_WristLeft], myMapper, SpaceFlag);				// 左手肘-左手腕
	drawline(img, myJointArr[JointType_SpineBase], myJointArr[JointType_HipLeft], myMapper, SpaceFlag);				// 脊柱底-左胯部
	drawline(img, myJointArr[JointType_SpineBase], myJointArr[JointType_HipRight], myMapper, SpaceFlag);				// 脊柱底-右胯部
	drawline(img, myJointArr[JointType_ElbowRight], myJointArr[JointType_WristRight], myMapper, SpaceFlag);			// 右手肘-右手腕

	drawline(img, myJointArr[JointType_WristLeft], myJointArr[JointType_ThumbLeft], myMapper, SpaceFlag);				// 左手腕-左拇指
	drawline(img, myJointArr[JointType_WristLeft], myJointArr[JointType_HandLeft], myMapper, SpaceFlag);				// 左手腕-左手掌
	drawline(img, myJointArr[JointType_HipLeft], myJointArr[JointType_KneeLeft], myMapper, SpaceFlag);				// 左胯部-左膝盖
	drawline(img, myJointArr[JointType_HipRight], myJointArr[JointType_KneeRight], myMapper, SpaceFlag);				// 右胯部-右膝盖
	drawline(img, myJointArr[JointType_WristRight], myJointArr[JointType_ThumbRight], myMapper, SpaceFlag);			// 右手腕-右拇指
	drawline(img, myJointArr[JointType_WristRight], myJointArr[JointType_HandRight], myMapper, SpaceFlag);			// 右手腕-右手掌

	drawline(img, myJointArr[JointType_HandLeft], myJointArr[JointType_HandTipLeft], myMapper, SpaceFlag);			// 左手掌-手指尖
	drawline(img, myJointArr[JointType_KneeLeft], myJointArr[JointType_FootLeft], myMapper, SpaceFlag);				// 左膝盖-左脚
	drawline(img, myJointArr[JointType_KneeRight], myJointArr[JointType_FootRight], myMapper, SpaceFlag);				// 右膝盖-右脚
	drawline(img, myJointArr[JointType_HandRight], myJointArr[JointType_HandTipRight], myMapper, SpaceFlag);			// 右手掌-手指尖
}

// 判断骨骼追踪情况：包括骨骼追踪完好且手部位置在肘上面  
bool WaveGestureDetection(Joint & elbow, Joint & hand, ICoordinateMapper * myMapper)
{
	// 骨骼追踪完好
	if (elbow.TrackingState == TrackingState_Tracked && hand.TrackingState == TrackingState_Tracked)
	{
		// 要把关节点用的摄像机坐标下的点转换成彩色空间的点
		Point	p_e = MapCameraPointToSomeSpace(myMapper, elbow, USEColorSpace),
			p_h = MapCameraPointToSomeSpace(myMapper, hand, USEColorSpace);
		// 手掌高于手肘
		if (p_h.y < p_e.y)
		{
			int elbow_range = 0, hand_range = 0;
			int elbow_MAX = INT_MIN, elbow_MIN = INT_MAX, hand_MAX = INT_MIN, hand_MIN = INT_MAX;
			// 保存近10帧的手掌和手肘位置，并进行数据更新
			for (int i = 0; i < 9; i++)
			{
				data[i][0] = data[i + 1][0];
				data[i][1] = data[i + 1][1];
			}
			data[9][0] = p_e.x;
			data[9][1] = p_h.x;
			// 计算手掌和手肘的运动范围
			for (int i = 0; i < 10; i++)
			{
				if (data[i][0]>elbow_MAX) elbow_MAX = data[i][0];
				if (data[i][0]<elbow_MIN) elbow_MIN = data[i][0];
				if (data[i][1]>hand_MAX) hand_MAX = data[i][1];
				if (data[i][1] < hand_MIN) hand_MIN = data[i][1];
			}
			elbow_range = elbow_MAX - elbow_MIN;
			hand_range = hand_MAX - hand_MIN;
			//cout << "R_e:" << p_e.x << "," << p_e.y << "  手肘运动范围：" << elbow_range << endl;
			//cout << "R_h:" << p_h.x << "," << p_h.y << "  手掌运动范围：" << hand_range << endl;
			// 手肘、手掌运动阈值判定
			if (hand_range >= 250 && elbow_range <= 100)
			{
				cout << "识别到挥手动作" << endl;
				return true;
			}
			else //cout << "未识别" << endl;
				return false;
		}
		else
		{
			//cout << "不满足手掌高于手肘条件" << endl;
			return false;
		}
	}
	else return false;
}

//获得某点像素值  
int get_pixel(Mat & img, Point pt) {
	int width = img.cols; //图片宽度  
	int height = img.rows; //图片宽度t;//图片高度  
	uchar* ptr = (uchar*)img.data + pt.y * width; //获得灰度值数据指针  
	int intensity = ptr[pt.x];
	return intensity;
}
