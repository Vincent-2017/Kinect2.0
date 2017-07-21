#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <Kinect.h>  
#include<Kinect.Face.h>
#pragma comment ( lib, "kinect20.lib" )  
#pragma comment ( lib, "Kinect20.face.lib" )  

using namespace cv;
using namespace std;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


int main()
{

	IKinectSensor*kinect;
	GetDefaultKinectSensor(&kinect);
	kinect->Open();
	HRESULT hResult;

	IColorFrameSource*colorsource;
	kinect->get_ColorFrameSource(&colorsource);
	IColorFrameReader*colorreader;
	colorsource->OpenReader(&colorreader);
	IBodyFrameSource*bodysource;
	kinect->get_BodyFrameSource(&bodysource);
	IBodyFrameReader*bodyreader;
	bodysource->OpenReader(&bodyreader);
	ICoordinateMapper* coordinatemapper;
	kinect->get_CoordinateMapper(&coordinatemapper);

	IHighDefinitionFaceFrameSource* hdfacesource[BODY_COUNT];
	IHighDefinitionFaceFrameReader* hdfacereader[BODY_COUNT];
	IFaceModelBuilder* facemodelbuilder[BODY_COUNT];
	bool produce[BODY_COUNT] = { false };
	IFaceAlignment* facealignment[BODY_COUNT];
	IFaceModel* facemodel[BODY_COUNT];
	vector<vector<float>> deformations(BODY_COUNT, vector<float>(FaceShapeDeformations_Count));

	for (int count = 0; count < BODY_COUNT; count++)
	{
		CreateHighDefinitionFaceFrameSource(kinect, &hdfacesource[count]);
		hdfacesource[count]->OpenReader(&hdfacereader[count]);
		hdfacesource[count]->OpenModelBuilder(FaceModelBuilderAttributes_None, &facemodelbuilder[count]);
		facemodelbuilder[count]->BeginFaceDataCollection();
		CreateFaceAlignment(&facealignment[count]);
		CreateFaceModel(1.0f, FaceShapeDeformations_Count, &deformations[count][0], &facemodel[count]);
	}

	UINT32 vertex = 0;
	GetFaceModelVertexCount(&vertex);

	while (1)
	{
		Mat asd(1080, 1920, CV_8UC4);

		IColorFrame* colorframe = nullptr;
		hResult = colorreader->AcquireLatestFrame(&colorframe);
		if (colorframe == nullptr)
			continue;
		if (SUCCEEDED(hResult))
		{
			colorframe->CopyConvertedFrameDataToArray(1920 * 1080 * 4, reinterpret_cast<BYTE*>(asd.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
		SafeRelease(colorframe);

		IBodyFrame* bodyframe = nullptr;
		hResult = bodyreader->AcquireLatestFrame(&bodyframe);
		if (SUCCEEDED(hResult))
		{
			IBody* body[BODY_COUNT] = { 0 };
			hResult = bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
			if (SUCCEEDED(hResult))
			{
				//#pragma omp parallel for 
				for (int i = 0; i < BODY_COUNT; i++)
				{
					BOOLEAN tracked = false;
					hResult = body[i]->get_IsTracked(&tracked);
					if (SUCCEEDED(hResult) && tracked)
					{
						UINT64 trackingId = _UI64_MAX;
						hResult = body[i]->get_TrackingId(&trackingId);
						if (SUCCEEDED(hResult))
						{
							hdfacesource[i]->put_TrackingId(trackingId);
						}
					}
				}
			}
			for (int i = 0; i < BODY_COUNT; i++)
			{
				SafeRelease(body[i]);
			}
		}
		SafeRelease(bodyframe);
		//#pragma omp parallel for 
		for (int i = 0; i < BODY_COUNT; i++)
		{
			IHighDefinitionFaceFrame* hdfaceframe = nullptr;
			hResult = hdfacereader[i]->AcquireLatestFrame(&hdfaceframe);
			if (SUCCEEDED(hResult) && hdfaceframe != nullptr)
			{
				BOOLEAN tracked = false;
				hResult = hdfaceframe->get_IsFaceTracked(&tracked);
				if (SUCCEEDED(hResult) && tracked)
				{
					hResult = hdfaceframe->GetAndRefreshFaceAlignmentResult(facealignment[i]);
					if (SUCCEEDED(hResult) && facealignment[i] != nullptr)
					{
						if (!produce[i])
						{
							FaceModelBuilderCollectionStatus collection;
							hResult = facemodelbuilder[i]->get_CollectionStatus(&collection);
							if (collection == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete)
							{
								cout << "Status : Complete" << endl;
								putText(asd, "Status : Complete", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
								IFaceModelData* facemodeldata = nullptr;
								hResult = facemodelbuilder[i]->GetFaceData(&facemodeldata);
								if (SUCCEEDED(hResult) && facemodeldata != nullptr)
								{
									hResult = facemodeldata->ProduceFaceModel(&facemodel[i]);
									if (SUCCEEDED(hResult) && facemodel[i] != nullptr)
									{
										produce[i] = true;
									}
								}
								SafeRelease(facemodeldata);
							}
							else
							{
								if (collection >= FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded)
								{
									cout << "Need : Tilted Up Views" << endl;
									putText(asd, "Need : Tilted Up Views", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
								}
								else if (collection >= FaceModelBuilderCollectionStatus_RightViewsNeeded)
								{
									cout << "Need : Right Views" << endl;
									putText(asd, "Need : Right Views", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
								}
								else if (collection >= FaceModelBuilderCollectionStatus_LeftViewsNeeded)
								{
									cout << "Need : Left Views" << endl;
									putText(asd, "Need : Left Views", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
								}
								else if (collection >= FaceModelBuilderCollectionStatus_FrontViewFramesNeeded)
								{
									cout << "Need : Front ViewFrames" << endl;
									putText(asd, "Need : Front ViewFrames", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
								}

								FaceModelBuilderCaptureStatus capture;
								hResult = facemodelbuilder[i]->get_CaptureStatus(&capture);
								switch (capture)
								{
								case FaceModelBuilderCaptureStatus_FaceTooFar:
									cout << "Error : Face Too Far" << endl;
									putText(asd, "Error : Face Too Far", Point(50, 150), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
									break;
								case FaceModelBuilderCaptureStatus_FaceTooNear:
									cout << "Error : Face Too Near" << endl;
									putText(asd, "Error : Face Too Near", Point(50, 150), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
									break;
								case FaceModelBuilderCaptureStatus_MovingTooFast:
									cout << "Error : Moving Too Fast" << endl;
									putText(asd, "Error : Moving Too Fast", Point(50, 150), FONT_HERSHEY_SIMPLEX, 1.0f, Scalar(0, 0, 255, 255), 2);
									break;
								default:break;
								}
							}
						}

						vector<CameraSpacePoint> facePoints(vertex);
						hResult = facemodel[i]->CalculateVerticesForAlignment(facealignment[i], vertex, &facePoints[0]);
						if (SUCCEEDED(hResult))
						{
							//#pragma omp parallel for
							for (int point = 0; point < vertex; point++)
							{
								ColorSpacePoint colorSpacePoint;
								coordinatemapper->MapCameraPointToColorSpace(facePoints[point], &colorSpacePoint);
								circle(asd, Point(colorSpacePoint.X, colorSpacePoint.Y), 2, Scalar(0, 0, 255, 255), -1);
							}
						}
					}
				}
			}
			SafeRelease(hdfaceframe);
		}

		Mat faceimg;
		cv::resize(asd, faceimg, cv::Size(), 0.5, 0.5);
		cv::imshow("Face", faceimg);

		if (cv::waitKey(34) == VK_ESCAPE)
		{
			break;
		}
	}
	SafeRelease(colorsource);
	SafeRelease(bodysource);
	SafeRelease(colorreader);
	SafeRelease(bodyreader);
	SafeRelease(coordinatemapper);
	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(hdfacesource[i]);
		SafeRelease(hdfacereader[i]);
	}
	if (kinect)
	{
		kinect->Close();
	}
	SafeRelease(kinect);
	cv::destroyAllWindows();

}