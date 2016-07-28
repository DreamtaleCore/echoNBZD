#include <iostream>

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

#include "KF_lib/KF.hpp"

#include "serial_to_uav/api_sensor_data.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"

using namespace cv;
using namespace ros;
using namespace std;


KF kf;
Mat img(640, 640, CV_8UC3);
float R;
Point2f center;

// kalman
Mat state(2, 1, CV_32F); /* (phi, delta_phi) */ //状态值
Mat processNoise(2, 1, CV_32F);
Mat measurement = Mat::zeros(1, 1, CV_32F); // measure value

ros::Publisher pub;
ros::Subscriber sub;

void CallBack( const serial_to_uav::api_sensor_data::ConstPtr& msg );
Mat KFupdate( Mat _state, Mat _measurement, Mat _processNoise );
static inline Point calcPoint(Point2f center, double R, double angle);

int main(int argc,char **argv)
{
    cout <<endl
         <<"    I will rewrite this file about kalman filter"<<endl
         <<"    to get the prediction of the measurement"<<endl;
    center.x = 320;
    center.y = 320;
    R = 200;

    ros::init(argc, argv, "kalman_test_imu");
    ros::NodeHandle nh;

    cout<< "init kf."<<endl;
    /*********************
     * first,
     * init kalman filter
     *********************/
    kf.init(2,1,0,CV_32F);

    //randn( state, Scalar::all(0), Scalar::all(0.2) );
    state.at<float>(0,0) = 0;
    state.at<float>(0,1) = 0;
    //randn(kf.statePost, Scalar::all(0), Scalar::all(0.1));
    kf.statePost.at<float>(0,0) = 0;
    kf.statePost.at<float>(0,1) = 0;
    cout<<"state init"<<endl;

    kf.transitionMatrix = *(Mat_<float>(2, 2) << 1, 1,
                                                 0, 1 );

    //setIdentity( kf.measurementMatrix );
    setIdentity( kf.processNoiseCov, Scalar::all(1e-5) );
    setIdentity( kf.measurementNoiseCov, Scalar::all(0.1) );
    setIdentity( kf.errorCovPost, Scalar::all(1) );
    cout<< "kf ready."<<endl<<endl;


    pub = nh.advertise <geometry_msgs::Vector3>("kalman_pub",200);
    sub = nh.subscribe <serial_to_uav::api_sensor_data>("/uav_imu", 20, CallBack);

    ros::spin();
    return 0;
}

void CallBack( const serial_to_uav::api_sensor_data::ConstPtr& msg )
{
    float num;
    num = msg -> gps.x;

    cout<<endl<<"get measure data: "<<num<<endl;
    measurement.at<float>(0, 0) = num;

    state = KFupdate( state, measurement, processNoise );

}

Mat KFupdate( Mat _state, Mat _measurement, Mat _processNoise )
{
    Mat measurementNoise = Mat::zeros(1, 1, CV_32F); // measure value

    cout<<"call kf updata"<<endl;
    geometry_msgs::Vector3 outMsg;

    double stateValue = _state.at<float>(0);

    /************************
     * 1.5th
     * input the measurement
     ************************/
    //randn( measurementNoise, Scalar::all(0), Scalar::all(kf.measurementNoiseCov.at<float>(0)));
    //_measurement.at<float>(0,0) += measurementNoise.at<float>(0,0);

    cout<<"measure1"<<_measurement<<endl;

    kf.measure( _measurement );

    // generate measuremen
    //_measurement += kf.measurementMatrix * _state;

    cout <<"measure2"<<_measurement<<endl
         <<"state"<< _state <<endl
         <<"measureMat"<<kf.measurementMatrix<<endl
         <<endl;

    /*********************************************************
     * second,
     * use kf.predict() to get state prediction
     * predicted _state (x'(k)): x'(k)=A*x(k-1)+B*u(k)
     *********************************************************/

    Mat prediction = kf.predict();
    double predictAngle = prediction.at<float>(0,0);

    double measAngle = _measurement.at<float>(0);
    cout<<" | predictAngle: "<< measAngle <<endl
        <<" | measAngle   : "<< predictAngle <<endl
        <<" | delta       : "<< predictAngle - measAngle<<endl;

    // Calculate or Merge
    Point statePt = calcPoint(center, R, stateValue);
    Point predictPt = calcPoint(center, R, predictAngle);
    Point measPt = calcPoint(center, R, measAngle);

    // plot points
    #define drawCross( center, color, d )                                 \
            line( img, Point( center.x - d, center.y - d ),                \
                       Point( center.x + d, center.y + d ), color, 1, CV_AA, 0); \
            line( img, Point( center.x + d, center.y - d ),                \
                       Point( center.x - d, center.y + d ), color, 1, CV_AA, 0 )
    img = Scalar::all(0);
    drawCross( statePt, Scalar(255,255,255), 4 );
    drawCross( measPt, Scalar(0,0,255), 4 );
    drawCross( predictPt, Scalar(255,255,0), 4 );
    line( img, statePt, measPt, Scalar(0,0,255), 2, CV_AA, 0 );
    line( img, statePt, predictPt, Scalar(0,255,255), 1, CV_AA, 0 );

    outMsg.x = predictPt.x;
    outMsg.y = predictPt.y;
    pub.publish(outMsg);

    /*********************************************************
     * third,
     * use kf.correct() to get state with mearsurement
     * corrected _state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
     * *******************************************************/

    //if(theRNG().uniform(0,4) != 0)
    kf.correct( _measurement );

    randn( _processNoise, Scalar(0), Scalar::all(sqrt(kf.processNoiseCov.at<float>(0, 0))));

    _state = kf.statePost;
    //_state = kf.transitionMatrix*_state + _processNoise;

    imshow( "Kalman", img );
    waitKey(100);

    return _state;
}

static inline Point calcPoint(Point2f center, double R, double angle)
{
    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R ;
}
