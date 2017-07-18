#include "global.h"

CvFont font;

int main(void)
{

	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.5f, 1.5f, 0, 2, CV_AA);//设置显示的字体

	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IColorFrameSource   * myColorSource = nullptr;
	mySensor->get_ColorFrameSource(&myColorSource);

	IColorFrameReader   * myColorReader = nullptr;
	myColorSource->OpenReader(&myColorReader);

	int colorHeight = 0, colorWidth = 0;
	IFrameDescription   * myDescription = nullptr;
	myColorSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&colorHeight);
	myDescription->get_Width(&colorWidth);

	IColorFrame * myColorFrame = nullptr;
	Mat original(colorHeight, colorWidth, CV_8UC4);

	//**********************以上为ColorFrame的读取前准备**************************

	IBodyFrameSource    * myBodySource = nullptr;
	mySensor->get_BodyFrameSource(&myBodySource);

	IBodyFrameReader    * myBodyReader = nullptr;
	myBodySource->OpenReader(&myBodyReader);

	int myBodyCount = 0;
	myBodySource->get_BodyCount(&myBodyCount);

	IBodyFrame  * myBodyFrame = nullptr;

	ICoordinateMapper   * myMapper = nullptr;
	mySensor->get_CoordinateMapper(&myMapper);

	//**********************以上为BodyFrame以及Mapper的准备***********************
	while (1)
	{
		while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);
		myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, original.data, ColorImageFormat_Bgra); // 图像大小、目标地址、图像格式
		Mat copy = original.clone();        //读取彩色图像并输出到矩阵

		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK); //读取身体图像
		IBody   **  myBodyArr = new IBody *[myBodyCount];       //为存身体数据的数组做准备
		for (int i = 0; i < myBodyCount; i++)
			myBodyArr[i] = nullptr;

		if (myBodyFrame->GetAndRefreshBodyData(myBodyCount, myBodyArr) == S_OK)     //把身体数据输入数组
		for (int i = 0; i < myBodyCount; i++)
		{
			BOOLEAN     result = false;
			if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) //先判断是否侦测到
			{
				Joint   myJointArr[JointType_Count];
				if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)   //如果侦测到就把关节数据输入到数组并画图
				{
					draw(copy, myJointArr[JointType_Head], myJointArr[JointType_Neck], myMapper);						// 头-颈
					draw(copy, myJointArr[JointType_Neck], myJointArr[JointType_SpineShoulder], myMapper);				// 颈-脊柱肩

					draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderLeft], myMapper);		// 脊柱肩-左肩膀
					draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_SpineMid], myMapper);			// 脊柱肩-脊柱中
					draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderRight], myMapper);		// 脊柱肩-右肩膀

					draw(copy, myJointArr[JointType_ShoulderLeft], myJointArr[JointType_ElbowLeft], myMapper);			// 左肩膀-左手肘
					draw(copy, myJointArr[JointType_SpineMid], myJointArr[JointType_SpineBase], myMapper);				// 脊柱中-脊柱底
					draw(copy, myJointArr[JointType_ShoulderRight], myJointArr[JointType_ElbowRight], myMapper);		// 右肩膀-右手肘

					draw(copy, myJointArr[JointType_ElbowLeft], myJointArr[JointType_WristLeft], myMapper);				// 左手肘-左手腕
					draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipLeft], myMapper);				// 脊柱底-左胯部
					draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipRight], myMapper);				// 脊柱底-右胯部
					draw(copy, myJointArr[JointType_ElbowRight], myJointArr[JointType_WristRight], myMapper);			// 右手肘-右手腕

					draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_ThumbLeft], myMapper);				// 左手腕-左拇指
					draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_HandLeft], myMapper);				// 左手腕-左手掌
					draw(copy, myJointArr[JointType_HipLeft], myJointArr[JointType_KneeLeft], myMapper);				// 左胯部-左膝盖
					draw(copy, myJointArr[JointType_HipRight], myJointArr[JointType_KneeRight], myMapper);				// 右胯部-右膝盖
					draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_ThumbRight], myMapper);			// 右手腕-右拇指
					draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_HandRight], myMapper);			// 右手腕-右手掌

					draw(copy, myJointArr[JointType_HandLeft], myJointArr[JointType_HandTipLeft], myMapper);			// 左手掌-手指尖
					draw(copy, myJointArr[JointType_KneeLeft], myJointArr[JointType_FootLeft], myMapper);				// 左膝盖-左脚
					draw(copy, myJointArr[JointType_KneeRight], myJointArr[JointType_FootRight], myMapper);				// 右膝盖-右脚
					draw(copy, myJointArr[JointType_HandRight], myJointArr[JointType_HandTipRight], myMapper);			// 右手掌-手指尖

					drawhandstate(copy, myJointArr[JointType_HandLeft], myJointArr[JointType_HandRight], myBodyArr[i], myMapper);
				}
			}
		}
		delete[]myBodyArr;
		myBodyFrame->Release();
		myColorFrame->Release();

		imshow("TEST", copy);
		if (waitKey(30) == VK_ESCAPE)
			break;
	}
	myMapper->Release();

	myDescription->Release();
	myColorReader->Release();
	myColorSource->Release();

	myBodyReader->Release();
	myBodySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}

