/**
  ***************************************************************************************
  * @file    kalman.cpp
  * @author  gaowenliang
  * @version V1.0.0
  * @date    02-April-2015
  * @brief   This file is a implementation of KF
  ***************************************************************************************
  */

#include "EKF.hpp"

EKF::EKF()
{
    //cout<<"EKF run"<<endl;
    ;
}

EKF::EKF(int dynamParams, int measureParams, int controlParams,
         void (*_f_func)(Mat&, const Mat&),
         void (*_h_func)(Mat&, const Mat&),
         double processNoiseCovariance,double measurementNoiseCovariance,double errorCovariancePost,
         int type)
{
    // Call external function
    f_func = _f_func;
    h_func = _h_func;
    //EKF::setMFC(_h_func);

    // auto run EKF::init
    init( dynamParams, measureParams, controlParams,
          processNoiseCovariance, measurementNoiseCovariance,errorCovariancePost,
          type);
}

void EKF::init(int DP, int MP, int CP,
               double processNoiseCovariance ,double measurementNoiseCovariance,double errorCovariancePost,
               int type)
{
    CV_Assert( DP > 0 && MP > 0 );
    CV_Assert( type == CV_32F || type == CV_64F );
    CP = std::max(CP, 0);

    statePre = Mat::zeros(DP, 1, type);
    statePost = Mat::zeros(DP, 1, type);
///////////////////////////////////////////////////////////////
    JacobianF = Mat::zeros(DP, DP, type);
    JacobianH = Mat::zeros(MP, DP, type);

    //for(;;)
    //{
        //JacobianF.at<float>(0,0) = ;
    //}
    //JacobianH = Mat::zeros(MP, DP, type);
    //for(;;)
    //{
        //JacobianH.at<float>(0,0) = ;
    //}
    /////////////////////////////////////////////////////////////////////////

    transitionMatrix = JacobianF;
    measurementMatrix = JacobianH;

    processNoiseCov = Mat::eye(DP, DP, type);
    measurementValue = Mat::zeros(MP, 1, type);
    measurementNoiseCov = Mat::eye(MP, MP, type);

    measureSize = measurementValue.size();

    errorCovPre = Mat::zeros(DP, DP, type);
    errorCovPost = Mat::zeros(DP, DP, type);
    gain = Mat::zeros(DP, MP, type);

    if( CP > 0 )
        controlMatrix = Mat::zeros(DP, CP, type);
    else
        controlMatrix.release();

    temp1.create(DP, DP, type);
    temp2.create(MP, DP, type);
    temp3.create(MP, MP, type);
    temp4.create(MP, DP, type);
    temp5.create(MP, 1, type);

    setIdentity( processNoiseCov, Scalar::all( processNoiseCovariance ) );
    setIdentity( measurementNoiseCov, Scalar::all( measurementNoiseCovariance ) );
    setIdentity( errorCovPost, Scalar::all( errorCovariancePost ) );

    cout<< "init KF done."<<endl;


}

const Mat& EKF::predict(const Mat& _state, const Mat& control)
{
    // update predicted _state x'(k): x'(k)=f( x'(k) )
    statePre = f_func(_state,  );////////////////////////////////////////////////////////

    /*
    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix * statePost;
    // x'(k) = x'(k) + B*u(k)
    if( control.data )
        statePre += controlMatrix*control;
    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix*errorCovPost;
    */

    // update transitionMatrix JacobianF
    JacobianF.at<float>();
    transitionMatrix = JacobianF;


    // update the state: P'(k) = A*P(k)*At + Q
    // P'(k) = temp1*At + Q
    gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, GEMM_2_T);
    // handle the case when there will be measurement before the next predict.
    statePre.copyTo(statePost);
    errorCovPre.copyTo(errorCovPost);

    // return predicted state
    return statePre;
}

const Mat& EKF::correct(const Mat& measurement)
{
    // corrected Kt : Kt(k) =  H*P'(k)/(H*P'(k)*Ht + R)
    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;
    // temp3 = temp2*Ht + R = H*P'(k)*Ht + R
    gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, GEMM_2_T);
    // Kt(k) = temp4 = inv(temp3)*temp2 = H*P'(k)/(H*P'(k)*Ht + R)
    solve(temp3, temp2, temp4, DECOMP_SVD);
    gain = temp4.t();

    // update measurementMatrix JacobianH
    JacobianH.at<float>();
    measurementMatrix = JacobianH;

    // corrected _state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurementMatrix * statePre;
    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain * temp5;

    // P(k) = P'(k) - K(k)*H*P'(k)
    // P(k) = P'(k) - K(k)*temp2 = P'(k) - K(k)*H*P'(k)
    errorCovPost = errorCovPre - gain*temp2;

    return statePost;
}

const Mat& EKF::measure(const Mat& ValueMat)
{
    //cout<<"size_x "<<measureSize.width<<endl;
    //cout<<"size_y "<<measureSize.height<<endl;

    for(int i=0; i <= measureSize.height - 1;)
    {
        measurementValue.at<float>(i,0) = ValueMat.at<float>(i);
        i++;
    }

}

const void EKF::setPNC(const float setValue)
{
    setIdentity( processNoiseCov, Scalar::all( setValue ) );
    cout<<"ProcessNoiseCov set done";
}

const void EKF::setMNC(const float setValue)
{
    setIdentity( measurementNoiseCov, Scalar::all( setValue ) );
    cout<<"MeasurementNoiseCov set done";
}

const void EKF::setECP(const float setValue)
{
    setIdentity( errorCovPost, Scalar::all( setValue ) );
    cout<<"ErrorCovPost set done";
}

const void EKF::setMFC(void (*_h_func)(Mat&, const Mat&))
{
    h_func=_h_func;
}
