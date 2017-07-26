#include <Kinect.h>		//Kinect的头文件
#include <iostream>
#include <opencv2\imgproc.hpp>	//opencv头文件
#include <opencv2\calib3d.hpp>
#include <opencv2\highgui.hpp>

using   namespace   std;
using   namespace   cv;

//获得某点像素值  
int get_pixel(Mat & img, Point pt) {
	int width = img.cols; //图片宽度  
	int height = img.rows; //图片宽度t;//图片高度  
	uchar* ptr = (uchar*)img.data + pt.y * width; //获得灰度值数据指针  
	int intensity = ptr[pt.x];
	return intensity;
}

int main(void)
{
	IKinectSensor* KinectSensor = nullptr;
	GetDefaultKinectSensor(&KinectSensor);  //获取感应器
	KinectSensor->Open();           //打开感应器
	cout << "打开Kinect传感器" << endl;
	cout << endl;

	IDepthFrameSource   * DepthSource = nullptr;   //取得深度数据
	KinectSensor->get_DepthFrameSource(&DepthSource);
	int height = 0, width = 0;
	IFrameDescription   * myDescription = nullptr;  //取得深度数据的分辨率
	DepthSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	cout <<"深度图像大小是："<< height << " * " << width << endl;
	myDescription->Release();

	IDepthFrameReader   * DepthReader = nullptr;
	DepthSource->OpenReader(&DepthReader);    //打开深度数据的Reader

	IDepthFrame * myFrame = nullptr;
	Mat temp(height, width, CV_16UC1);    //建立图像矩阵
	Mat img(height, width, CV_8UC1);
	while (1)
	{
		if (DepthReader->AcquireLatestFrame(&myFrame) == S_OK) //通过Reader尝试获取最新的一帧深度数据，放入深度帧中,并判断是否成功获取
		{
			//原始UINT16 深度图像不适合用来显示，所以需要砍成8位的就可以了
			myFrame->CopyFrameDataToArray(height * width, (UINT16 *)temp.data); //先把数据存入16位的图像矩阵中
			temp.convertTo(img, CV_8UC1, 255.0 / 4500);   //再把16位转换为8位
			cout << get_pixel(img, Point(200, 200)) << endl;
			imshow("Depth", img);
			myFrame->Release();
		}
		//POINT pt;
		//GetCursorPos(&pt);
		//cout << "当前的鼠标坐标为：" << pt.x << "," << pt.y << endl;
		if (waitKey(30) == VK_ESCAPE)
			break;
	}
	DepthReader->Release();        //释放不用的变量并且关闭感应器
	DepthSource->Release();
	KinectSensor->Close();
	KinectSensor->Release();

	return  0;
}

