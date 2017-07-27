#ifndef GLOBAL_H
#define GLOBAL_H

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <Kinect.h>  
#include <Kinect.Face.h>
#include <Kinect.VisualGestureBuilder.h>
#pragma comment ( lib, "kinect20.lib" )  
#pragma comment ( lib, "Kinect20.face.lib" )  
#pragma comment ( lib, "Kinect20.VisualGestureBuilder.lib" )  

using namespace cv;
using namespace std;

#include "util.h" //放在后面才能使用cv和std空间

#define USEColorSpace 1
#define USEDepthSpace 2

extern int data[10][2];
extern Point leftdepthpoint, rightdepthpoint, leftcolorpoint, rightcolorpoint;
extern int leftdepth, rightdepth;
extern string lhandstate, rhandstate;

#endif