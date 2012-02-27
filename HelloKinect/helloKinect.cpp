#include "helloKinect.h"
#include "XnCppWrapper.h"
#include <highgui.h>
#include <cv.h>
#include <stdio.h>
#include <iostream>

using namespace xn;
using namespace std;

/*
Perform serveral tasks; enumerates all the Kinect sensors connected, activates one of the Kinect, and both.
Press 'esc' to quit.
No input parameters needed.
*/

helloKinect::helloKinect(void)
{
	XnStatus nRetVal = XN_STATUS_OK;
	bool bShouldrun = true;
	xn::Context context;
	int c;
	IplImage* kinectDepthImage;
	IplImage* kinectRGBImage;
	CvMat* depthMetersMat = cvCreateMat(480, 640, CV_8UC1);
	XnUInt32 nMiddleIndex = XN_VGA_X_RES * XN_VGA_Y_RES/2 + XN_VGA_X_RES/2;

	//Initialize context object
	nRetVal = context.Init();
	
	// Create a DepthGenerator node
	xn::DepthGenerator depth;
	nRetVal = depth.Create(context);

	// Create a RGBGenerator node
	xn::ImageGenerator rgb;
	nRetVal = rgb.Create(context);

	nRetVal = context.StartGeneratingAll();
	cvNamedWindow("Depth", 1);
	cvNamedWindow("RGB", 1);

	//Main loop
	while (bShouldrun)
	{
		kinectRGBImage = cvCreateImage(cvSize(640,480),8,3);
		// Wait for new data to be available
		nRetVal = context.WaitOneUpdateAll(depth);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Failed updating data: %s\n");
			xnGetStatusString(nRetVal);
			continue;
		}
		nRetVal = context.WaitOneUpdateAll(rgb);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Failed updating data: %s\n");
			xnGetStatusString(nRetVal);
			continue;
		}
		// Take current depth map
		const XnDepthPixel* pDepthMap = depth.GetDepthMap();
		printf("Middle pixel is %u millimeters away\n", pDepthMap[nMiddleIndex]);
		// Take current rgb map
		const XnRGB24Pixel* pImageMap = rgb.GetRGB24ImageMap();
		
		for (int y=0; y<XN_VGA_Y_RES; y++)
		{
			uchar *ptr = (uchar*)kinectRGBImage->imageData + y*kinectRGBImage->widthStep;
			for (int x=0; x<XN_VGA_X_RES; x++)
			{
				depthMetersMat->data.ptr[y*XN_VGA_X_RES+x]=pDepthMap[y*XN_VGA_X_RES+x];
				ptr[3*x] = pImageMap->nBlue;
				ptr[3*x + 1] = pImageMap->nGreen;
				ptr[3*x + 2] = pImageMap->nRed;
				pImageMap++;
			}
		}
		kinectDepthImage = cvCreateImage(cvSize(640,480),8,1);		
		cvGetImage(depthMetersMat, kinectDepthImage);
		cvShowImage("Depth", kinectDepthImage);
		cvShowImage("RGB", kinectRGBImage);
		cvReleaseImageHeader(&kinectDepthImage);
		cvReleaseImageHeader(&kinectRGBImage);
		c = cvWaitKey(1);
		if (c == 27)
			bShouldrun = false;
	}

	// Clean-up
	context.Shutdown();
}


helloKinect::~helloKinect(void)
{
}


void mixRGB_Depth()
{
	bool bShouldRun = true;
	int c;

	XnStatus nRetVal = XN_STATUS_OK;
	Context context;

	// Initialize context object
	nRetVal = context.Init();

	// Check error code
	if (nRetVal)
		printf("Error: %s", xnGetStatusString(nRetVal));

	context.SetGlobalMirror(true);

	//Create Depth generator node
	DepthGenerator depth;
	nRetVal = depth.Create(context);
	// Check error code
	if (nRetVal)
		printf("Error: %s", xnGetStatusString(nRetVal));

	// Create an ImageGenetor node
	ImageGenerator image;
	nRetVal = image.Create(context);
	if (nRetVal)
		printf("Error: %s", xnGetStatusString(nRetVal));

	// Sync the DepthGenerator with the ImageGenerator
	nRetVal = depth.GetAlternativeViewPointCap().SetViewPoint(image);
	if (nRetVal)
		printf("Error: %s", xnGetStatusString(nRetVal));

	//Set it to VGA maps at 30 fps
	XnMapOutputMode mapMode;
	mapMode.nXRes = XN_VGA_X_RES;
	mapMode.nYRes = XN_VGA_Y_RES;
	mapMode.nFPS = 30;
	nRetVal = depth.SetMapOutputMode(mapMode);

	// Make it start generating data
	nRetVal = context.StartGeneratingAll();
	if (nRetVal)
		printf("Error: %s", xnGetStatusString(nRetVal));

	// Create an OpenCv matrix
	CvMat* depthMetersMat = cvCreateMat(480, 640, CV_16UC1);
	IplImage *kinectDepthImage;
	kinectDepthImage = cvCreateImage(cvSize(640,480), 16, 1);

	IplImage *rgbimg = cvCreateImageHeader(cvSize(640,480), 8,3);

	// Main loop
	while (bShouldRun)
	{
		//wait for new data to be available
		nRetVal = context.WaitOneUpdateAll(depth);
		if (nRetVal)
		{
			printf("Error: %s", xnGetStatusString(nRetVal));
			continue;
		}
		//Take current depth map
		const XnDepthPixel* pDepthMap = depth.GetDepthMap();

		for (int y=0; y<XN_VGA_Y_RES; y++)
		{
			for (int x=0; x<XN_VGA_X_RES; x++)
			{
				depthMetersMat->data.s[y*XN_VGA_X_RES+x]=10*pDepthMap[y*XN_VGA_X_RES+x];
			}
		}

		cvGetImage(depthMetersMat, kinectDepthImage);

		//take current image
		const XnRGB24Pixel* pImage = image.GetRGB24ImageMap();
		//process image data
		XnRGB24Pixel* ucpImage = const_cast<XnRGB24Pixel*>(pImage);
		cvSetData(rgbimg, ucpImage, 640*3);
		cvShowImage("RGB", kinectDepthImage);

		c = cvWaitKey(1);
		if (c == 27)
			bShouldRun = false;
	}

	cvReleaseImageHeader(&kinectDepthImage);
	context.Shutdown();
}


void enumerateDevices(int initType)
{
	NodeInfoList device_node_info_list;
	NodeInfoList depth_node_info_list;
	NodeInfoList image_node_info_list;
	Context context;
	XnStatus status;

	if (initType == 0)
		status = context.InitFromXmlFile("D:\\initXml.xml");
	else
		status = context.Init();
	// TODO: check errors

	printf("devices:\n");
	
	status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, device_node_info_list);
	if (status != XN_STATUS_OK && device_node_info_list.Begin() != device_node_info_list.End())
	{
		printf("Enumerating devices failed. Reason: %s", xnGetStatusString(status));
	}
	for (NodeInfoList::Iterator nodeIt = device_node_info_list.Begin(); nodeIt != device_node_info_list.End(); ++nodeIt)
	{
		const xn::NodeInfo& info = *nodeIt;
		const XnProductionNodeDescription& description = info.GetDescription();
		printf("device: vendor %s name %s, instance %s\n", description.strVendor, description.strName, info.GetInstanceName());
		
		unsigned short vendor_id; 
		unsigned short product_id; 
		unsigned char bus; 
		unsigned char address; 
		sscanf(info.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address); 
		string connection_string = info.GetCreationInfo(); 
		transform (connection_string.begin (), connection_string.end (), connection_string.begin (), tolower); 
		printf("vendor_id %i product_id %i bus %i address %i connection %s \n", vendor_id, product_id, bus, address, connection_string.c_str()); 

	}

	//enumerate depth nodes
	printf("depth nodes:\n");
	status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, depth_node_info_list, NULL);
	if (status != XN_STATUS_OK)
	{
		printf("enumerating depth generators failed. Reason: %s\n", xnGetStatusString(status));
	}
	else
	{
		for (NodeInfoList::Iterator nodeIt = depth_node_info_list.Begin(); nodeIt != depth_node_info_list.End(); ++ nodeIt)
		{
			const NodeInfo& info = *nodeIt;
			const XnProductionNodeDescription& description = info.GetDescription();
			printf("depth: vendor %s name %s, instance %s\n", description.strVendor, description.strName, info.GetInstanceName());

			unsigned short vendor_id; 
			unsigned short product_id; 
			unsigned char bus; 
			unsigned char address; 
			sscanf(info.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address); 
			string connection_string = info.GetCreationInfo(); 
			transform (connection_string.begin (), connection_string.end (), connection_string.begin (), tolower); 
			printf("vendor_id %i product_id %i bus %i address %i connection %s \n", vendor_id, product_id, bus, address, connection_string.c_str()); 
		}
	}

	 // enumerate image nodes 
     printf("image nodes:\n"); 
     status = context.EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, NULL, image_node_info_list, NULL); 
     if (status != XN_STATUS_OK) 
	 { 
		 printf("enumerating image generators failed. Reason: %s\n", xnGetStatusString (status)); 
	 } else 
	 {
		 for (NodeInfoList::Iterator nodeIt = image_node_info_list.Begin(); nodeIt != image_node_info_list.End(); ++nodeIt) 
		 { 
			 const NodeInfo& info = *nodeIt; 
			 const XnProductionNodeDescription& description = info.GetDescription(); 
			 printf("image: vendor %s name %s, instance %s\n", description.strVendor, description.strName, info.GetInstanceName()); 
		 } 
	 } 
}

void OneKinect(int type)
{
	NodeInfoList image_node_info_list;
	NodeInfoList depth_node_info_list;
	XnStatus status;
	Context context;
	int c;
	IplImage* kinectRGBImage;
	bool bShouldrun = true;

	context.Init();
//	status = context.InitFromXmlFile("D:\\initXml.xml");
	Query query;
	
	switch (type) {

	case 0: 
		status = query.SetVendor("PrimeSense");
		status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, &query, depth_node_info_list, NULL);
		//status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, depth_node_info_list, NULL);
		if (status != XN_STATUS_OK)
			printf("Enumerating devices failed. Reason: %s", xnGetStatusString(status));
		else
		{
			NodeInfoList::Iterator nodeIt = depth_node_info_list.Begin();
//			NodeInfo& selectedNode = *depth_node_info_list.Begin();
//			nodeIt++;
			
			NodeInfo& selectedNode = *nodeIt;

			printf("instance %s\n", selectedNode.GetInstanceName());

			DepthGenerator depth;
			status = selectedNode.GetInstance(depth);
			status = context.CreateProductionTree(selectedNode);
			status = depth.Create(context);
			status = context.StartGeneratingAll();
			cvNamedWindow("Depth", 1);

			// Create an OpenCv matrix
			CvMat* depthMetersMat = cvCreateMat(480, 640, CV_16UC1);
			IplImage *kinectDepthImage;

			while (bShouldrun)
			{
				status = context.WaitOneUpdateAll(depth);
				if (status)
				{
					printf("Error: %s", xnGetStatusString(status));
					continue;
				}
				//Take current depth map
				const XnDepthPixel* pDepthMap = depth.GetDepthMap();
				for (int y=0; y<XN_VGA_Y_RES; y++)
				{
					for (int x=0; x<XN_VGA_X_RES; x++)
					{
						depthMetersMat->data.s[y*XN_VGA_X_RES+x]=10*pDepthMap[y*XN_VGA_X_RES+x];
					}
				}
				kinectDepthImage = cvCreateImage(cvSize(640,480),8,1);
				cvGetImage(depthMetersMat, kinectDepthImage);
				cvShowImage("Depth", kinectDepthImage);
				cvReleaseImageHeader(&kinectDepthImage);
				c = cvWaitKey(1);
				if (c == 27)
					bShouldrun = false;
			}

		}
		break;
		
	case 1:
		status = context.EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, NULL, image_node_info_list, NULL); 
		if (status != XN_STATUS_OK)
			printf("Enumerating devices failed. Reason: %s", xnGetStatusString(status));
		else
		{
			NodeInfo& selectedNode = *image_node_info_list.Begin();
			xn::ImageGenerator rgb;
			status = selectedNode.GetInstance(rgb);
			status = context.CreateProductionTree(selectedNode);
			status = rgb.Create(context);
			
			status = context.StartGeneratingAll();
			cvNamedWindow("RGB", 1);

			while (bShouldrun)
			{
				kinectRGBImage = cvCreateImage(cvSize(640,480),8,3);
				// Wait for new data to be available
				status = context.WaitOneUpdateAll(rgb);
				if (status != XN_STATUS_OK)
				{
					printf("Failed updating data: %s\n");
					xnGetStatusString(status);
					continue;
				}
				// Take current rgb map
				const XnRGB24Pixel* pImageMap = rgb.GetRGB24ImageMap();
				
				for (int y=0; y<XN_VGA_Y_RES; y++)
				{
					uchar *ptr = (uchar*)kinectRGBImage->imageData + y*kinectRGBImage->widthStep;
					for (int x=0; x<XN_VGA_X_RES; x++)
					{
						ptr[3*x] = pImageMap->nBlue;
						ptr[3*x + 1] = pImageMap->nGreen;
						ptr[3*x + 2] = pImageMap->nRed;
						pImageMap++;
					}
				}
				cvShowImage("RGB", kinectRGBImage);
				cvReleaseImageHeader(&kinectRGBImage);
				c = cvWaitKey(1);
				if (c == 27)
					bShouldrun = false;
			}
		}
		break;

	default:
		cout << "Incorrect number" << endl;

	} // end switch

	// Clean-up
	context.Shutdown();

}

void multipleKinects_DepthNodes()
{
	NodeInfoList depth_node_info_list;
	XnStatus status;
	Context context1, context2;
	int c;
	bool bShouldrun = true;

	context1.Init();
	context2.Init();
	Query query;
	status = query.SetVendor("PrimeSense");
	status = context1.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, &query, depth_node_info_list, NULL);
	//status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, depth_node_info_list, NULL);
	if (status != XN_STATUS_OK)
		printf("Enumerating devices failed. Reason: %s", xnGetStatusString(status));
	else
	{
		NodeInfoList::Iterator nodeIt = depth_node_info_list.Begin();
		NodeInfo& firstNode = *nodeIt;
		nodeIt++;
		NodeInfo& secondNode = *nodeIt;

		DepthGenerator depth1, depth2;
		status = firstNode.GetInstance(depth1);
		status = secondNode.GetInstance(depth2);
		status = context1.CreateProductionTree(firstNode);
		status = context2.CreateProductionTree(secondNode);
		status = depth1.Create(context1);
		status = depth2.Create(context2);
		status = context1.StartGeneratingAll();
		status = context2.StartGeneratingAll();
		cvNamedWindow("Depth1", 1);
		cvNamedWindow("Depth2", 1);

		// Create an OpenCv matrix
		CvMat* firstDepthMetersMat = cvCreateMat(480, 640, CV_16UC1);
		CvMat* seocndDepthMetersMat = cvCreateMat(480, 640, CV_16UC1);
		IplImage *kinectFirstDepthImage;
		IplImage *kinectSecondDepthImage;

		while (bShouldrun)
		{
			status = context1.WaitOneUpdateAll(depth1);
			status = context2.WaitOneUpdateAll(depth2);
			if (status)
			{
				printf("Error: %s", xnGetStatusString(status));
				continue;
			}
			
			//Take current depth map
			const XnDepthPixel* pFirstDepthMap = depth1.GetDepthMap();
			const XnDepthPixel* pSecondDepthMap = depth2.GetDepthMap();
			for (int y=0; y<XN_VGA_Y_RES; y++)
			{
				for (int x=0; x<XN_VGA_X_RES; x++)
				{
					firstDepthMetersMat->data.s[y*XN_VGA_X_RES+x]=10*pFirstDepthMap[y*XN_VGA_X_RES+x];
					seocndDepthMetersMat->data.s[y*XN_VGA_X_RES+x]=10*pSecondDepthMap[y*XN_VGA_X_RES+x];
				}
			}
			kinectFirstDepthImage = cvCreateImage(cvSize(640,480),8,1);
			kinectSecondDepthImage = cvCreateImage(cvSize(640,480),8,1);
			cvGetImage(firstDepthMetersMat, kinectFirstDepthImage);
			cvGetImage(seocndDepthMetersMat, kinectSecondDepthImage);
			cvShowImage("Depth1", kinectFirstDepthImage);
			cvShowImage("Depth2", kinectSecondDepthImage);
			cvReleaseImageHeader(&kinectFirstDepthImage);
			cvReleaseImageHeader(&kinectSecondDepthImage);
			c = cvWaitKey(1);
			if (c == 27)
				bShouldrun = false;
		}
	}
	// Clean-up
	context1.Shutdown();
	context2.Shutdown();
}


void multipleKinects_ImageNodes()
{
	NodeInfoList image_node_info_list;
	IplImage* kinectFirstRGBImage;
	IplImage* kinectSecondRGBImage;
	XnStatus status;
	Context context1, context2;
	int c;
	bool bShouldrun = true;

	context1.Init();
	context2.Init();
	Query query;
	status = query.SetVendor("PrimeSense");
	status = context1.EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, &query, image_node_info_list, NULL);
	//status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, depth_node_info_list, NULL);
	if (status != XN_STATUS_OK)
		printf("Enumerating devices failed. Reason: %s", xnGetStatusString(status));
	else
	{
		NodeInfoList::Iterator nodeIt = image_node_info_list.Begin();
		NodeInfo& firstNode = *nodeIt;
		nodeIt++;
		NodeInfo& secondNode = *nodeIt;

		ImageGenerator rgb1, rgb2;
		status = firstNode.GetInstance(rgb1);
		status = secondNode.GetInstance(rgb2);
		status = context1.CreateProductionTree(firstNode);
		status = context2.CreateProductionTree(secondNode);
		status = rgb1.Create(context1);
		status = rgb2.Create(context2);
		status = context1.StartGeneratingAll();
		status = context2.StartGeneratingAll();
		cvNamedWindow("RGB1", 1);
		cvNamedWindow("RGB2", 1);

		while (bShouldrun)
		{
				kinectFirstRGBImage = cvCreateImage(cvSize(640,480),8,3);
				kinectSecondRGBImage = cvCreateImage(cvSize(640,480),8,3);
				// Wait for new data to be available
				status = context2.WaitOneUpdateAll(rgb2);
				status = context1.WaitOneUpdateAll(rgb1);
				if (status != XN_STATUS_OK)
				{
					printf("Failed updating data: %s\n");
					xnGetStatusString(status);
					continue;
				}
				// Take current rgb map
				const XnRGB24Pixel* pFirstImageMap = rgb1.GetRGB24ImageMap();
				const XnRGB24Pixel* pSecondImageMap = rgb2.GetRGB24ImageMap();
				
				for (int y=0; y<XN_VGA_Y_RES; y++)
				{
					uchar *ptr1 = (uchar*)kinectFirstRGBImage->imageData + y*kinectFirstRGBImage->widthStep;
					uchar *ptr2 = (uchar*)kinectSecondRGBImage->imageData + y*kinectSecondRGBImage->widthStep;
					for (int x=0; x<XN_VGA_X_RES; x++)
					{
						ptr1[3*x] = pFirstImageMap->nBlue;
						ptr1[3*x + 1] = pFirstImageMap->nGreen;
						ptr1[3*x + 2] = pFirstImageMap->nRed;
						pFirstImageMap++;

						ptr2[3*x] = pSecondImageMap->nBlue;
						ptr2[3*x + 1] = pSecondImageMap->nGreen;
						ptr2[3*x + 2] = pSecondImageMap->nRed;
						pSecondImageMap++;
					}
				}
				cvShowImage("RGB1", kinectFirstRGBImage);
				cvShowImage("RGB2", kinectSecondRGBImage);
				cvReleaseImageHeader(&kinectFirstRGBImage);
				cvReleaseImageHeader(&kinectSecondRGBImage);
				c = cvWaitKey(1);
				if (c == 27)
					bShouldrun = false;
			}
	}
	// Clean-up
	context1.Shutdown();
	context2.Shutdown();
}

int main()
{
	int option;
	bool showMenu = true;
	while (showMenu)
	{
		cout << "Please choose one of the following options: " << endl;;
		cout << "1.- Kinect depth + RGB registration." << endl;
		cout << "2.- Kinect devices connected enumeration." << endl;
		cout << "3.- One Kinect depth sensor sample." << endl;
		cout << "4.- Two Kinect depth sensors simultaneously." << endl;
		cout << "5.- Two Kinect rgb sensors simultaneously." << endl;
		cin >> option;
		switch (option) {
		case 1:
			{
				//Check the depth and the distance using a roi
				mixRGB_Depth();
				showMenu=false;
				break;
			}
		case 2:
			{
				//Check the depth and the distance. Using cvFindChessboardCorners function
				enumerateDevices(1);
				showMenu=false;
				break;
			}
		case 3:
			{
				//compare the % of pixels corrupted and the ammount of noise
				//added by the other camera
				OneKinect(0);
				showMenu=false;
				break;
			}
		case 4:
			{
				multipleKinects_DepthNodes();
				showMenu=false;
				break;
			}
		case 5:
			{
				multipleKinects_ImageNodes();
				showMenu=false;
				break;
			}
		default:
			cout << "Option not allowed" << endl;
		}
	}
	return 0;

}
