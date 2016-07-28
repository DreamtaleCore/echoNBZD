#include <iostream>

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

#include "KF_lib/KF.hpp"

#include "std_msgs/Float32.h"

using namespace cv;
using namespace ros;
using namespace std;


KF kf;
Mat img(640, 640, CV_8UC3);
float R = 290, r = 200;
Point2f center;

// kalman
Mat state(2, 1, CV_32F); /* (phi, delta_phi) */ //状态值
Mat processNoise(2, 1, CV_32F);
Mat measurement = Mat::zeros(1, 1, CV_32F); // measure value

ros::Publisher pub;
ros::Subscriber sub;

void CallBack( const std_msgs::Float32::ConstPtr& msg );
Mat KFupdate( Mat _state, Mat _measurement, Mat _processNoise );
static inline Point calcPoint(Point2f center, double R, double angle);

int main(int argc,char **argv)
{
    center.x = 320;
    center.y = 320;

    ros::init(argc, argv, "kalman_test_on_ros1");
    ros::NodeHandle nh;

    cout<< "init kf."<<endl;
    /*********************
     * first,
     * init kalman filter
     *********************/
    kf.init( 2, 1, 0, 1e-5, 0.1, 1, CV_32F);

    //randn( state, Scalar::all(0), Scalar::all(0.2) );
    state.at<float>(0,0) = 0;
    state.at<float>(0,1) = 0;
    //randn(kf.statePost, Scalar::all(0), Scalar::all(0.1));
    kf.statePost.at<float>(0,0) = 0;
    kf.statePost.at<float>(0,1) = 0;
    cout<<"state init"<<endl;

    kf.transitionMatrix = *(Mat_<float>(2, 2) << 1, 1,
                                                 0, 1 );

    cout<< "kf ready."<<endl<<endl;

    pub = nh.advertise <std_msgs::Float32>("kalman_pub",200);
    sub = nh.subscribe <std_msgs::Float32>("kalman_sub", 200, CallBack);

    ros::spin();
    return 0;
}

void CallBack( const std_msgs::Float32::ConstPtr& msg )
{
    float num;
    num = msg -> data;
    cout<<endl<<"get measure data: "<<num<<endl;
    measurement.at<float>(0, 0) = num;

    state = KFupdate( state, measurement, processNoise );

}

Mat KFupdate( Mat _state, Mat _measurement, Mat _processNoise )
{
    Mat measurementNoise(1,1,CV_32F);

    cout<<"call kf updata"<<endl;
    std_msgs::Float32 outMsg;

    double stateValue = _state.at<float>(0);

    /************************
     * 1.5th
     * input the measurement
     ************************/
    kf.measure( _measurement );

    double measAngle = _measurement.at<float>(0);

    /*********************************************************
     * second,
     * use kf.predict() to get state prediction
     * predicted _state (x'(k)): x'(k)=A*x(k-1)+B*u(k)
     *********************************************************/
    Mat prediction = kf.predict();
    double predictAngle =   prediction.at<float>(0,0);

    cout<<" | predict: "<< predictAngle <<endl
        <<" | measure: "<< measAngle <<endl
        <<" | state  : "<< stateValue <<endl;

    // Calculate or Merge
    Point statePt = calcPoint(center, R, stateValue);
    Point predictPt = calcPoint(center, R, predictAngle);
    Point measPt = calcPoint(center, R, measAngle);

    // plot points
    #define drawCross( center, color, d )  \
            line( img, Point( center.x - d, center.y - d ), \
                       Point( center.x + d, center.y + d ), color, 1, CV_AA, 0); \
            line( img, Point( center.x + d, center.y - d ), \
                       Point( center.x - d, center.y + d ), color, 1, CV_AA, 0 )
    img = Scalar::all(0);
    drawCross( statePt, Scalar(255,255,255), 4 );
    drawCross( measPt, Scalar(0,0,255), 4 );
    drawCross( predictPt, Scalar(255,255,0), 4 );
    line( img, statePt, measPt, Scalar(0,0,255), 2, CV_AA, 0 );
    line( img, statePt, predictPt, Scalar(0,255,255), 1, CV_AA, 0 );

    outMsg.data = predictAngle;
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
