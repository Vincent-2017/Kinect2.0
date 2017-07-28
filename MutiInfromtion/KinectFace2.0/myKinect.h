#ifndef MYKINECT_H
#define MYKINECT_H
#include "global.h"

HRESULT hr = FALSE;
IKinectSensor   * mySensor = nullptr;

ICoordinateMapper   * myMapper = nullptr;

IColorFrameSource   * myColorSource = nullptr;
IColorFrameReader   * myColorReader = nullptr;
IFrameDescription   * mycolorDescription = nullptr;
int colorHeight = 0, colorWidth = 0;
IColorFrame * myColorFrame = nullptr;
Mat color(colorHeight, colorWidth, CV_8UC4);
Mat colormuti(colorHeight, colorWidth, CV_8UC4);


IDepthFrameSource   * myDepthSource = nullptr;   //取得深度数据
IDepthFrameReader   * myDepthReader = nullptr;
IFrameDescription   * mydepthDescription = nullptr;
int depthHeight = 0, depthWidth = 0;
IDepthFrame * myDepthFrame = nullptr;
Mat depth(depthHeight, depthWidth, CV_16UC1);    //建立图像矩阵
Mat showdepth(depthHeight, depthWidth, CV_8UC1);
Mat depthmuti(depthHeight, depthWidth, CV_8UC1);

IInfraredFrameSource * myInfraredSource = nullptr;
IInfraredFrameReader  * myInfraredReader = nullptr;
IFrameDescription   * myinfraredDescription = nullptr;
int infraredHeight = 0, infraredWidth = 0;
IInfraredFrame  * myInfraredFrame = nullptr;
Mat infrared(infraredHeight, infraredWidth, CV_16UC1);

IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;//读取背景二值图
IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//用于背景二值图读取
IFrameDescription   * mybodyindexDescription = nullptr;
int bodyindexHeight = 0, bodyindexWidth = 0;
IBodyIndexFrame  * myBodyIndexFrame = nullptr;
Mat bodyindex(bodyindexHeight, bodyindexWidth, CV_8UC3);
Mat bodyindexmuti(bodyindexHeight, bodyindexWidth, CV_8UC3);

IBodyFrameSource    * myBodySource = nullptr;
IBodyFrameReader    * myBodyReader = nullptr;
IBodyFrame  * myBodyFrame = nullptr;

IFaceFrameSource* facesource[BODY_COUNT];
IFaceFrameReader* facereader[BODY_COUNT];
DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

void InitAll();

#endif
