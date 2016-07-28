#include <iostream>

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/Float32.h"

#include "EKF_lib/EKF.hpp"

using namespace cv;
using namespace ros;
using namespace std;

#define g 9.8f
#define k_x 0.01f
#define k_y 0.05f

EKF ekf;
Mat img(640, 640, CV_8UC3);
float R = 270, r = 210, rr =150;
Point2f center;
// kalman
Mat state(4, 1, CV_32F),statePost(4, 1, CV_32F); /* (x, y, v_x, v_y) */ // state value
Mat processNoise(4, 1, CV_32F);
Mat measurement = Mat::zeros(2, 1, CV_32F); // measure value
Mat jacob_f, jacob_f;

ros::Publisher pub;
ros::Subscriber sub;

void CallBack( const std_msgs::Float32::ConstPtr& msg );
Mat ekfupdate( Mat _state, Mat _measurement, Mat _processNoise );
void process_func(Mat& x_t, const Mat& x_t_1, const Mat& u_t);
void measure_func(Mat& z_t, const Mat& x_t);


int main(int argc,char **argv)
{
    ros::init(argc, argv, "ekf_test_on_ros");
    ros::NodeHandle nh;

    void (*f_func)(Mat&, const Mat&, const Mat&);   // function pointer to process function
    void (*h_func)(Mat&, const Mat&);               // function pointer to measurement function
    f_func = &process_func;
    h_func = &measure_func;

    /*********************
     * first,
     * init kalman filter
     *********************/
    ekf.init( 4, 2, 0, 1e-5, 0.1, 1, CV_32F);

    state.at<float>(0,0) = 0;
    state.at<float>(1,0) = 0;
    state.at<float>(2,0) = 0;
    state.at<float>(3,0) = 0;


    ekf.statePost.at<float>(0,0) = 0;
    ekf.statePost.at<float>(1,0) = 0;
    ekf.statePost.at<float>(2,0) = 0;
    ekf.statePost.at<float>(3,0) = 0;




    cout<< "state init done." <<endl;

    cout<< ekf.transitionMatrix <<endl;
    cout<< "ekf ready."<<endl<<endl;

    pub = nh.advertise <std_msgs::Float32>("kalman_pub", 20);
    sub = nh.subscribe <std_msgs::Float32>("kalman_sub", 20, CallBack);

    ros::spin();
    return 0;
}
/*
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

    state = ekfupdate( state, measurement, processNoise );

}*/

Mat ekfupdate( Mat _state, Mat _measurement, Mat _processNoise )
{
    cout<<"call ekf updata"<<endl;
    std_msgs::Float32 outMsg;

    double stateValue1, stateValue2, stateValue3;
    stateValue1 = _state.at<float>(0, 0);
    stateValue2 = _state.at<float>(1, 0);
    stateValue3 = _state.at<float>(2, 0);

    /************************
     * 1.5th
     * input the measurement
     ************************/
    ekf.measure( _measurement );

    double measAngle1, measAngle2, measAngle3;
    measAngle1 = _measurement.at<float>(0, 0);
    measAngle2 = _measurement.at<float>(1, 0);
    measAngle3 = _measurement.at<float>(2, 0);

    /*************************************************
     * second,
     * use ekf.predict() to get state prediction
     * predicted _state (x'(k)): x'(k)=A*x(k-1)+B*u(k)
     *************************************************/
    Mat prediction = ekf.predict();

    double predictAngle1, predictAngle2, predictAngle3;
    predictAngle1 = prediction.at<float>(0,0);
    predictAngle2 = prediction.at<float>(1,0);
    predictAngle3 = prediction.at<float>(2,0);

        cout<<" | predict: "<< predictAngle1<<" | "<<predictAngle2<<" | "<<predictAngle3<<endl
        <<" | measure: "<< measAngle1 <<" | "<< measAngle2<<" | " <<measAngle3 <<endl
        <<" | state  : "<< stateValue1<<" | "<<stateValue2<<" | "<<stateValue3 <<endl;
        img = Scalar::all(0);

    outMsg.data = predictAngle1;
    pub.publish(outMsg);

    /*********************************************************
     * third,
     * use ekf.correct() to get state with mearsurement
     * corrected _state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
     * *******************************************************/

    //if(theRNG().uniform(0,4) != 0)
    _state = ekf.correct( _measurement );

    randn( _processNoise, Scalar(0), Scalar::all(sqrt(ekf.processNoiseCov.at<float>(0, 0))));

    _state = ekf.transitionMatrix*_state + _processNoise;

    imshow( "Kalman", img );
    waitKey(5);

    return _state;
}

void process_func(Mat& _state, const Mat& _statePost ,float d_t)
{
    // updata process data
    float _x,_y,_v_x,_v_y;
    _x = _statePost.at<float>(0,0);
    _y = _statePost.at<float>(1,0);
    _v_x = _statePost.at<float>(2,0);
    _v_y = _statePost.at<float>(3,0);

    // updata the jacobian matrix
    ekf.JacobianF = *(Mat_<float>(4, 4) << 0, 0, 1, 0,
                                           0, 0, 0, 1,
                                           0, 0, -2*k_x*_v_x,0,
                                           0, 0, 0, 2*k_y*_v_y);
    ekf.JacobianH = *(Mat_<float>(2, 4) << 1/sqrt(_x*_x+_y*_y),1/sqrt(_x*_x+_y*_y), 0, 0,
                                           1/(_y+_x*_x/_y),-_x/(_y*_y+_x*_x), 0, 0 ;

            // this is a test function
    _state.at<float>(0,0) = _x + _v_x*d_t;
    _state.at<float>(1,0) = _y + _v_y*d_t;
    _state.at<float>(2,0) = _v_x + (-k_x*_v_x*_v_x)*d_t;
    _state.at<float>(3,0) = _v_y + ( g - k_y*_v_y*_v_y)*d_t;
}

void measure_func(Mat& _measure, const Mat& _state)
{
    // updata measure data
    float _x = _state.at<float>(0,0);
    float _y = _state.at<float>(1,0);
    float _r,_a;

    _r = sqrt(_x*_x + _y*_y);
    _a = atan(_y,_x);

    _measure.at<float>(0,0) = _r;
    _measure.at<float>(1,0) = _a;
}
