#ifndef DRAW_H
#define DRAW_H

#include "global.h"

void drawline(cv::Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper);
void drawhandstate(cv::Mat & img, Joint & lefthand, Joint & righthand, IBody* myBodyArr, ICoordinateMapper * myMapper);
void DrawBody(cv::Mat & img, Joint *myJointArr, ICoordinateMapper * myMapper);
#endif