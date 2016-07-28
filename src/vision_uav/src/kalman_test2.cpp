#include <iostream>

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/Float32.h"

#include "KF_lib/KF.hpp"

using namespace cv;
using namespace ros;
using namespace std;


KF kf;
Mat img(640, 640, CV_8UC3);
float R = 270, r = 210, rr =150;
Point2f center;

// kalman
Mat state(6, 1, CV_32F); /* (phi, delta_phi) */ //状态值
Mat processNoise(6, 1, CV_32F);
Mat measurement = Mat::zeros(3, 1, CV_32F); // measure value

ros::Publisher pub;
ros::Subscriber sub;

void CallBack( const std_msgs::Float32::ConstPtr& msg );
Mat KFupdate( Mat _state, Mat _measurement, Mat _processNoise );
static inline Point calcPoint(Point2f center, double R, double angle);

int main(int argc,char **argv)
{
    center.x = 300;
    center.y = 300;

    ros::init(argc, argv, "kalman_test_on_ros2");
    ros::NodeHandle nh;

    /*********************
     * first,
     * init kalman filter
     *********************/
    kf.init( 6, 3, 0, 1e-5, 0.1, 1, CV_32F);

    state.at<float>(0,0) = 0;
    state.at<float>(1,0) = 0;
    state.at<float>(2,0) = 0;
    state.at<float>(3,0) = 0;
    state.at<float>(4,0) = 0;
    state.at<float>(5,0) = 0;

    kf.statePost.at<float>(0,0) = 0;
    kf.statePost.at<float>(1,0) = 0;
    kf.statePost.at<float>(2,0) = 0;
    kf.statePost.at<float>(3,0) = 0;
    kf.statePost.at<float>(4,0) = 0;
    kf.statePost.at<float>(5,0) = 0;

    cout<< "state init done." <<endl;

    cout<< kf.transitionMatrix <<endl;
    cout<< "kf ready."<<endl<<endl;

    pub = nh.advertise <std_msgs::Float32>("kalman_pub", 20);
    sub = nh.subscribe <std_msgs::Float32>("kalman_sub", 20, CallBack);

    ros::spin();
    return 0;
}

void CallBack( const std_msgs::Float32::ConstPtr& msg )
{
    float num1, num2, num3;
    num1 = msg -> data;
    num2 = num1 *2;
    num3 = num1 *3;
    cout<<endl<<"get measure data | " << num1 << " | " <<  num2 <<" | "<<num3 <<endl;
    measurement.at<float>(0, 0) = num1;
    measurement.at<float>(1, 0) = num2;
    measurement.at<float>(2, 0) = num3;

    state = KFupdate( state, measurement, processNoise );

}

Mat KFupdate( Mat _state, Mat _measurement, Mat _processNoise )
{
    cout<<"call kf updata"<<endl;
    std_msgs::Float32 outMsg;

    double stateValue1, stateValue2, stateValue3;
    stateValue1 = _state.at<float>(0, 0);
    stateValue2 = _state.at<float>(1, 0);
    stateValue3 = _state.at<float>(2, 0);

    /************************
     * 1.5th
     * input the measurement
     ************************/
    kf.measure( _measurement );

    double measAngle1, measAngle2, measAngle3;
    measAngle1 = _measurement.at<float>(0, 0);
    measAngle2 = _measurement.at<float>(1, 0);
    measAngle3 = _measurement.at<float>(2, 0);

    /*************************************************
     * second,
     * use kf.predict() to get state prediction
     * predicted _state (x'(k)): x'(k)=A*x(k-1)+B*u(k)
     *************************************************/
    Mat prediction = kf.predict();

    double predictAngle1, predictAngle2, predictAngle3;
    predictAngle1 = prediction.at<float>(0,0);
    predictAngle2 = prediction.at<float>(1,0);
    predictAngle3 = prediction.at<float>(2,0);

        cout<<" | predict: "<< predictAngle1<<" | "<<predictAngle2<<" | "<<predictAngle3<<endl
        <<" | measure: "<< measAngle1 <<" | "<< measAngle2<<" | " <<measAngle3 <<endl
        <<" | state  : "<< stateValue1<<" | "<<stateValue2<<" | "<<stateValue3 <<endl;
        img = Scalar::all(0);
        // Calculate or Merge & plot points
        Point statePt1 = calcPoint(center, R, stateValue1);
        Point predictPt1 = calcPoint(center, R, predictAngle1);
        Point measPt1 = calcPoint(center, R, measAngle1);
        Point statePt2 = calcPoint(center, r, stateValue2);
        Point predictPt2 = calcPoint(center, r, predictAngle2);
        Point measPt2 = calcPoint(center, r, measAngle2);
        Point statePt3 = calcPoint(center, rr, stateValue3);
        Point predictPt3 = calcPoint(center, rr, predictAngle3);
        Point measPt3 = calcPoint(center, rr, measAngle3);
        #define drawCross( center, color, d )  \
                line( img, Point( center.x - d, center.y - d ), \
                           Point( center.x + d, center.y + d ), color, 1, CV_AA, 0); \
                line( img, Point( center.x + d, center.y - d ), \
                           Point( center.x - d, center.y + d ), color, 1, CV_AA, 0 )
        drawCross( statePt1, Scalar(255,255,255), 4 );
        drawCross( measPt1, Scalar(0,0,255), 4 );
        drawCross( predictPt1, Scalar(255,255,0), 4 );
        drawCross( statePt2, Scalar(255,255,255), 4 );
        drawCross( measPt2, Scalar(0,0,255), 4 );
        drawCross( predictPt2, Scalar(255,255,0), 4 );
        drawCross( statePt3, Scalar(255,255,255), 4 );
        drawCross( measPt3, Scalar(0,0,255), 4 );
        drawCross( predictPt3, Scalar(255,255,0), 4 );
        line( img, statePt1, measPt1, Scalar(0,0,255), 2, CV_AA, 0 );
        line( img, statePt1, predictPt1, Scalar(0,255,255), 1, CV_AA, 0 );
        line( img, statePt2, measPt2, Scalar(0,255,0), 2, CV_AA, 0 );
        line( img, statePt2, predictPt2, Scalar(255,0,255), 1, CV_AA, 0 );
        line( img, statePt3, measPt3, Scalar(255,0,0), 2, CV_AA, 0 );
        line( img, statePt3, predictPt3, Scalar(255,255,0), 1, CV_AA, 0 );

    outMsg.data = predictAngle1;
    pub.publish(outMsg);

    /*********************************************************
     * third,
     * use kf.correct() to get state with mearsurement
     * corrected _state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
     * *******************************************************/

    //if(theRNG().uniform(0,4) != 0)
    _state = kf.correct( _measurement );

    randn( _processNoise, Scalar(0), Scalar::all(sqrt(kf.processNoiseCov.at<float>(0, 0))));

    _state = kf.transitionMatrix*_state + _processNoise;

    imshow( "Kalman", img );
    waitKey(5);

    return _state;
}

static inline Point calcPoint(Point2f center, double R, double angle)
{
    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R ;
}
