/**
  ***************************************************************************************
  * @file    kalman.cpp
  * @author  gaowenliang
  * @version V1.0.0
  * @date    02-April-2015
  * @brief   This file is a implementation of KF
  ***************************************************************************************
  */

#include "KF.hpp"

KF::KF()
{
    ;
}

KF::KF(int dynamParams, int measureParams, int controlParams,
       double processNoiseCovariance,double measurementNoiseCovariance,double errorCovariancePost,
       int type)
{
    // must run KF::init
    init( dynamParams, measureParams, controlParams,
          processNoiseCovariance, measurementNoiseCovariance,errorCovariancePost,
          type);
}

void KF::init(int DP, int MP, int controlParams,
              double processNoiseCovariance ,double measurementNoiseCovariance,double errorCovariancePost,
              int type)
{
    CV_Assert( DP > 0 && MP > 0 );
    CV_Assert( type == CV_32F || type == CV_64F );
    controlParams = std::max(controlParams, 0);

    statePre = Mat::zeros(DP, 1, type);
    statePost = Mat::zeros(DP, 1, type);
    transitionMatrix = Mat::eye(DP, DP, type);

    processNoiseCov = Mat::eye(DP, DP, type);
    measurementMatrix = Mat::zeros(MP, DP, type);
    measurementValue = Mat::zeros(MP, 1, type);
    measurementNoiseCov = Mat::eye(MP, MP, type);

    measureSize = measurementValue.size();

    errorCovPre = Mat::zeros(DP, DP, type);
    errorCovPost = Mat::zeros(DP, DP, type);
    gain = Mat::zeros(DP, MP, type);

    if( controlParams > 0 )
        controlMatrix = Mat::zeros(DP, controlParams, type);
    else
        controlMatrix.release();

    temp1.create(DP, DP, type);
    temp2.create(MP, DP, type);
    temp3.create(MP, MP, type);
    temp4.create(MP, DP, type);
    temp5.create(MP, 1, type);

    setIdentity( measurementMatrix);
    setIdentity( processNoiseCov, Scalar::all( processNoiseCovariance ) );
    setIdentity( measurementNoiseCov, Scalar::all( measurementNoiseCovariance ) );
    setIdentity( errorCovPost, Scalar::all( errorCovariancePost ) );

    if(DP == 2 && MP == 1)
    {
        transitionMatrix = *(Mat_<float>(2, 2) << 1, 1,
                                                  0, 1 );
        cout<<"set transition Matrix done."<<endl;
    }
    else if(DP == 4 && MP == 2)
    {
        transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0,
                                                  0, 1, 0, 1,
                                                  0, 0, 1, 0,
                                                  0, 0, 0, 1 );
        cout<<"set transition Matrix done."<<endl;
    }
    else if(DP == 6 && MP == 3)
    {
        transitionMatrix = *(Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
                                                  0, 1, 0, 0, 1, 0,
                                                  0, 0, 1, 0, 0, 1,
                                                  0, 0, 0, 1, 0, 0,
                                                  0, 0, 0, 0, 1, 0,
                                                  0, 0, 0, 0, 0, 1 );
        cout<<"set transition Matrix done."<<endl;
    }
    else if(DP == 8 && MP == 4)
    {
        transitionMatrix = *(Mat_<float>(8, 8) << 1, 0, 0, 0, 1, 0, 0, 0,
                                                  0, 1, 0, 0, 0, 1, 0, 0,
                                                  0, 0, 1, 0, 0, 0, 1, 0,
                                                  0, 0, 0, 1, 0, 0, 0, 1,
                                                  0, 0, 0, 0, 1, 0, 0, 0,
                                                  0, 0, 0, 0, 0, 1, 0, 0,
                                                  0, 0, 0, 0, 0, 0, 1, 0,
                                                  0, 0, 0, 0, 0, 0, 0, 1 );
        cout<<"set transition Matrix done."<<endl;
    }
    cout<< "init KF done."<<endl;


}

const Mat& KF::predict(const Mat& control)
{
    //predicted _state (x'(k)): x'(k)=A*x(k-1)+B*u(k)

    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix * statePost;
    // x'(k) = x'(k) + B*u(k)
    if( control.data )
        statePre += controlMatrix*control;
    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix*errorCovPost;
	
    // update the state: P'(k) = A*P(k)*At + Q
    // P'(k) = temp1*At + Q
    gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, GEMM_2_T);
    // handle the case when there will be measurement before the next predict.
    statePre.copyTo(statePost);
    errorCovPre.copyTo(errorCovPost);

    // return predicted state
    return statePre;
}

const Mat& KF::correct(const Mat& measurement)
{
    // corrected Kt : Kt(k) =  H*P'(k)/(H*P'(k)*Ht + R)
    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;
    // temp3 = temp2*Ht + R = H*P'(k)*Ht + R
    gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, GEMM_2_T);
    // Kt(k) = temp4 = inv(temp3)*temp2 = H*P'(k)/(H*P'(k)*Ht + R)
    solve(temp3, temp2, temp4, DECOMP_SVD);
    gain = temp4.t();
    
    // corrected _state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurementMatrix * statePre;
    // x(k) = x'(k) + K(k)*temp5 =x'(k) + K(k)*[z(k) - H*x'(k)]
    statePost = statePre + gain * temp5;

    // P(k) = P'(k) - K(k)*H*P'(k)
    // P(k) = P'(k) - K(k)*temp2 = P'(k) - K(k)*H*P'(k)
    errorCovPost = errorCovPre - gain*temp2;

    return statePost;
}

const Mat& KF::measure(const Mat& ValueMat)
{
    //cout<<"size_x "<<measureSize.width<<endl;
    //cout<<"size_y "<<measureSize.height<<endl;

    for(int i=0; i <= measureSize.height - 1;)
    {
        measurementValue.at<float>(i,0) = ValueMat.at<float>(i);
        i++;
    }

}

const void KF::setProcessNoiseCov(const float setValue)
{
    setIdentity( processNoiseCov, Scalar::all( setValue ) );
    cout<<"ProcessNoiseCov set done";
}

const void KF::setMeasurementNoiseCov(const float setValue)
{
    setIdentity( measurementNoiseCov, Scalar::all( setValue ) );
    cout<<"MeasurementNoiseCov set done";
}

const void KF::setErrorCovPost(const float setValue)
{
    setIdentity( errorCovPost, Scalar::all( setValue ) );
    cout<<"ErrorCovPost set done";
}

