#ifndef UTIL_H
#define UTIL_H

#include "global.h"

Point MapCameraPointToSomeSpace(ICoordinateMapper * myMapper, Joint & joint, int SpaceFlag);
void drawline(cv::Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper, int SpaceFlag);
void drawhandstate(cv::Mat & img, Joint & lefthand, Joint & righthand, IBody* myBodyArr, ICoordinateMapper * myMapper, int SpaceFlag);
void DrawBody(cv::Mat & img, Joint *myJointArr, ICoordinateMapper * myMapper, int SpaceFlag);
bool WaveGestureDetection(Joint & elbow, Joint & hand, ICoordinateMapper * myMapper);
int get_pixel(Mat & img, Point pt);


#endif