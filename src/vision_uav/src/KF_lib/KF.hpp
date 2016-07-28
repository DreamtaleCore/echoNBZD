/**
  ***************************************************************************************
  * @file    kalman.hpp
  * @author  gaowenliang
  * @version V1.0.0
  * @date    02-April-2015
  * @brief   This file is a implementation of KF
  * @History modified from openCV kalman filter
  ***************************************************************************************
  */

#ifndef __KF_HPP
#define __KF_HPP

#include <iostream>
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

/*!
 Kalman filter.

 The class implements standard Kalman filter.
 However, you can modify KF::transitionMatrix, KF::controlMatrix and
 KF::measurementMatrix to get the extended Kalman filter functionality.
*/

class  KF
{

public:

    KF();

    // the full constructor taking the dimensionality of the state, of the measurement and of the control vector
    KF(int dynamParams, int measureParams, int controlParams ,
                 double processNoiseCovariance,double measurementNoiseCovariance,double errorCovariancePost,
                 int type=CV_32F );

    void init(int dynamParams, int measureParams, int controlParams ,
              double processNoiseCovariance,double measurementNoiseCovariance,double errorCovariancePost,
              int type=CV_32F );

    // computes predicted state
    const Mat& predict(const Mat& control=Mat());
    // updates the predicted state from the measurement
    const Mat& correct(const Mat& measurement);

    const Mat& measure(const Mat& ValueMat);

    const void setProcessNoiseCov(const float setValue);
    const void setMeasurementNoiseCov(const float setValue);
    const void setErrorCovPost(const float setValue);


    Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))

    Mat transitionMatrix;   //!< state transition matrix (A)
    Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)
    Mat measurementMatrix;  //!< measurement matrix (H)

    Mat measurementValue;   //!< measurement value (z)

    Mat processNoiseCov;    //!< process noise covariance matrix (Q)
    Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)

    Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

private:

    Size measureSize;

    // temporary matrices
    Mat temp1;
    Mat temp2;
    Mat temp3;
    Mat temp4;
    Mat temp5;



};

#endif
