//include system in/out libraries
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>


//include ros libraries
#include<ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"


//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/legacy/compat.hpp>
#include "opencv2/ml/ml.hpp"
#include "opencv2/objdetect/objdetect.hpp"

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

#define THRESHOLD 80
#define SCALE 2
/* ****CONST DEFINITION**** */
//
//      * ------------------->  x
//      |
//      |
//      |
//     \/ y
// below are the positions of markers relative to home point

cv_bridge::CvImagePtr cv_ptr;
Mat InImage;
geometry_msgs::Vector3 OutMsg;

String SearchCascadeName = "/home/mylove/ws/src/vision_uav/try.xml";//Looking for training data

bool search_bool = true;

vector<Rect> target;

const static Scalar colors[] =
              { CV_RGB(0,128,255),
                CV_RGB(0,255,128),
                CV_RGB(255,128,0),
                CV_RGB(255,255,0),
                CV_RGB(255,0,0),
                CV_RGB(0,255,0),
                CV_RGB(0,0,255),
                CV_RGB(255,0,255)};//Represent different targets in different colors
int i = 0;
int center_x = 0,center_y = 0;
//Histogram of the image data
int nHistImageWidth = 255;
int nHistImageHeight = 150;
int nScale = 2;
Size imageSize;

double scale = 1.4;//Scaling detected


Mat Image, ImageGRAY, mid,
    imageEqualize;

IplImage* piplImageGRAY;
IplImage* piplimageEqualize;

void FillWhite(IplImage *pImage);
CvHistogram* CreateGrayImageHist(IplImage **ppImage);
IplImage* CreateHisogramImage(int nImageWidth, int nScale, int nImageHeight, CvHistogram *pcvHistogram);
void Detect( Mat& img,CascadeClassifier& cascade,double scale);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

ros::Publisher pub;


int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_haar");
    ros::NodeHandle nh;
    cout<< "here is the 1st."<<endl;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;

    imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
    pub = nh.advertise<geometry_msgs::Vector3>("img_pos",200);

    ros::spin();
    return 0;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cout<< "something wrong?"<<endl;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    Image = cv_ptr->image;
    imageSize = Image.size();

    resize(Image,Image,Size(320,240));
    cvtColor(Image, Image, CV_GRAY2RGB);

    imshow("Image",Image);
    waitKey(1);

    double t = 0;
    t = (double)cvGetTickCount();//Used to calculate the execution time of the algorithm

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Image.copyTo(mid);
    cvtColor(Image, ImageGRAY, CV_BGR2GRAY);

    //IplImage* piplImageGRAY ;
    ImageGRAY.copyTo(imageEqualize);
    IplImage iplImageGRAY= IplImage(ImageGRAY);
    IplImage iplimageEqualize= IplImage(imageEqualize);

    piplImageGRAY = &iplImageGRAY;
    piplimageEqualize = &iplimageEqualize ;


        //Grayscale image histogram and histogram
        //CvHistogram *pcvHistogram = CreateGrayImageHist(&piplImageGRAY);
        //IplImage *pHistImage = CreateHisogramImage(nHistImageWidth, nScale, nHistImageHeight, pcvHistogram);

    cvEqualizeHist(piplImageGRAY, piplimageEqualize);

        //Histogram and histogram equalization after
        //CvHistogram *pcvHistogramEqualize = CreateGrayImageHist(&piplimageEqualize);
        //IplImage *pHistEqualizeImage = CreateHisogramImage(nHistImageWidth, nScale, nHistImageHeight, pcvHistogramEqualize);

        //cvShowImage("Grayscale", pHistImage);//Show Histogram
        //cvShowImage("Histogram - after equalization", pHistEqualizeImage);//After the display gray histogram equalization

    Mat imageEqualize(piplimageEqualize,0);


        //imshow("Undistored View", Image);
        //imshow("ImageGRAY View", ImageGRAY);
        //imshow("imageEqualize View", imageEqualize);
        //imwrite("imageEqualize View.jpg", imageEqualize);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if( search_bool==true)
    {
        ImageGRAY = imageEqualize;
        CascadeClassifier cascade;//Create a cascade classifier object

        if( !cascade.load( SearchCascadeName ) )//Load cascade classifier from the specified file directory
        {
             cerr << "ERROR: Could not load classifier cascade" << endl;
             goto FUNC_END;
        }

        if( !ImageGRAY.empty() )//Read picture data can not be empty
        {
            Detect( ImageGRAY, cascade, scale );
            printf("Detect successfully\n");
        }

        center_x = 0;
        center_y = 0;

        for( vector<Rect>::const_iterator r = target.begin(); r != target.end(); r++, i++ )
        {
            Point center;
            int radius;

            Scalar color = colors[i%8];

            center.x = cvRound((r->x + r->width*0.5));//Restored to its original size
            center.y = cvRound((r->y + r->height*0.5));
            radius = cvRound((r->width + r->height)*0.25);

            center_x = center.x -160;
            center_y = center.y -120;
            printf("target:\n-center.x=%d\n-center.y=%d\n\n",center.x,center.y);

            circle( mid, center, radius, color, 3, 8, 0 );
        }

        imshow( "debug", mid );

    }

    std::cout<<"Frame Info: "<<"height-"<<Image.rows<<" width-"<<Image.cols<<std::endl;

    t = (double)cvGetTickCount() - t;//Subtraction algorithm execution time for
    printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );


    OutMsg.x = center_x;
    OutMsg.y = center_y;
    OutMsg.z = 0;

FUNC_END:
    pub.publish( OutMsg);
}


void FillWhite(IplImage *pImage)
{
    cvRectangle(pImage, cvPoint(0, 0),
                cvPoint(pImage->width, pImage->height),
                CV_RGB(255, 255, 255),
                CV_FILLED);
}

//Create a histogram of the gray-scale image
CvHistogram* CreateGrayImageHist(IplImage **ppImage)
{
    int nHistSize = 256;
    float fRange[] = {0, 255};  //Gray level range
    float *pfRanges[] = {fRange};
    CvHistogram *pcvHistogram = cvCreateHist(1, &nHistSize, CV_HIST_ARRAY, pfRanges);
    cvCalcHist(ppImage, pcvHistogram);

    return pcvHistogram;
}

//Create a histogram based on the histogram
IplImage* CreateHisogramImage(int nImageWidth,
                              int nScale, int nImageHeight,
                              CvHistogram *pcvHistogram)
{
    IplImage *pHistImage = cvCreateImage(cvSize(nImageWidth * nScale, nImageHeight), IPL_DEPTH_8U, 1);
    FillWhite(pHistImage);

    //Histogram maximum straight box
    float fMaxHistValue = 0;
    cvGetMinMaxHistValue(pcvHistogram, NULL, &fMaxHistValue, NULL, NULL);

    //The value of each block were drawn directly into the diagram
    int i;
    for(i = 0; i < nImageWidth; i++)
    {
        float fHistValue = cvQueryHistValue_1D(pcvHistogram, i); //i straight square pixel size
        int nRealHeight = cvRound((fHistValue / fMaxHistValue) * nImageHeight);  //To draw height
        cvRectangle(pHistImage,
                    cvPoint(i * nScale, nImageHeight - 1),
                    cvPoint((i + 1) * nScale - 1, nImageHeight - nRealHeight),
                    cvScalar(i, 0, 0, 0),
                    CV_FILLED
                    );
    }

    return pHistImage;
}

void Detect( Mat& img,
             CascadeClassifier& cascade,
             double scale)
{
    /*The Zoom Out, speed detection speed*/
    Mat smallImg_search( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

    /*The reduced size to 1/scale, with linear interpolation*/
    resize( img, smallImg_search, smallImg_search.size(), 0, 0, INTER_LINEAR );
    GaussianBlur( smallImg_search, smallImg_search, Size( 3, 3 ), 0);

    cascade.detectMultiScale( smallImg_search, target,
                              1.1, 4  /*Detection level for the n + 1*/
                              , 0
                            //|CV_HAAR_FIND_BIGGEST_OBJECT  /*Detect only the largest object*/
                            //|CV_HAAR_DO_ROUGH_SEARCH  /*Only a little early detection*/
                            |CV_HAAR_SCALE_IMAGE  /*Proportionally normal detection*/
                            //|CV_HAAR_DO_CANNY_PRUNING
                              /*Use Canny edge detector to exclude some of the edge of the image area with little or a lot*/
                            ,
                            Size(30, 35)  //Minimum and maximum size of the object
                            );
}
