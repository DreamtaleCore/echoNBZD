/**
  ***************************************************************************************
  * @file    EKF.hpp
  * @author  gaowenliang
  * @version V1.0.0
  * @date    02-April-2015
  * @brief   This file is a implementation of EKF
  * @History modified from openCV kalman filter
  ***************************************************************************************
  */

#ifndef __EKF_HPP
#define __EKF_HPP

#include <iostream>
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

/*!
 Extended Kalman filter.

 The class implements standard Extended Kalman filter.
 I have already modified KalmanFilter::transitionMatrix, KalmanFilter::controlMatrix and
 KalmanFilter::measurementMatrix IN "KF.hpp"&"KF.cpp" to get the extended Kalman filter functionality.
 THE Correspondence is:
 transitionMatrix  : JacobianF;
 controlMatrix     : ;
 measurementMatrix : JacobianH;
*/

class  EKF
{

public:

    EKF();

    // the full constructor taking the dimensionality of the state, of the measurement and of the control vector
    EKF(int dynamParams, int measureParams, int controlParams ,
        void (*_f_func)(Mat&, const Mat&),
        void (*_h_func)(Mat&, const Mat&),
        double processNoiseCovariance, double measurementNoiseCovariance, double errorCovariancePost,
        int type=CV_32F );

    void init(int dynamParams, int measureParams, int controlParams ,
              double processNoiseCovariance,double measurementNoiseCovariance,double errorCovariancePost,
              int type=CV_32F );

    // computes predicted state
    const Mat& predict(const Mat& _state, const Mat& control=Mat());
    // updates the predicted state from the measurement
    const Mat& correct(const Mat& measurement);

    const Mat& measure(const Mat& ValueMat);

    const void setPNC(const float setValue);     // set Process Noise Covariance
    const void setMNC(const float setValue);     // set Measurement Noise Covariance
    const void setECP(const float setValue);     // set Error Covariance Post
    const void setMFC(const float setValue);     // set Measurement Function


    Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))

    Mat transitionMatrix;   //!< state transition matrix (A)
    Mat measurementMatrix;  //!< measurement matrix (H)
    Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)

    Mat JacobianF;          //!< state transition matrix in EKF (A)
    Mat JacobianH;          //!< measurement matrix in EKF (H)

    Mat measurementValue;   //!< measurement value (z)

    Mat processNoiseCov;    //!< process noise covariance matrix (Q)
    Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)

    Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

private:

    Size measureSize;

    // function pointer to process function
    void (*f_func)(Mat&, const Mat&, const Mat a, const float dt);
    // function pointer to measurement function
    void (*h_func)(Mat&, const Mat&);

    // temporary matrices
    Mat temp1;
    Mat temp2;
    Mat temp3;
    Mat temp4;
    Mat temp5;



};

#endif
