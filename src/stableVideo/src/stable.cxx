//include system in/out libraries
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int16MultiArray.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <math.h>

//include ros libraries
#include <ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

//include opencv libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/compat.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

// The defintions of ros
ros::NodeHandle nh;
image_transport::Publisher pub;
cv_bridge::CvImagePtr cvPtr;
cv_bridge::CvImage outImg;
ros::Rate loopRate(30);
// Rosing

// The start code of stable video =======>

#define SEGSIZE 500        //每次处理帧数
#define WINDOWSIZE 30       //计算帧匹配代价窗口
#define GAPSIZE 5         //首关键帧与尾关键帧分别在段前GAPSIZE帧和后GAPSIZE帧中
#define TAUS 200
#define TAUA 200
#define VELOCITY 30
//是否去抖振幅判断阈值
#define SHIFT_SD 0.4
#define ROTATE_SD 0.017
#define SCALE_SD 0.1

int MIN_DISTANCE = 20;   //KLT参数

Size videoSize;
bool needShakeDetect = false;

vector<Mat> frameVec;//存储视频帧序列的容器
vector<vector<float> > costMap;//存储帧匹配代价
int processCount;//表示第一次处理，还是中间次数处理，还是最后一次处理

//判断特征点坐标是否超出图像范围
bool outOfImg(const Point2f &point, const Size &size)
{
    return (point.x <= 0 || point.y <= 0 || point.x >= size.width - 1 || point.y >= size.height - 1 );
}

//判断两个特征点是否为同一点，当其坐标十分接近时认为是同一点
bool isTheSame(const Point2f &point, const Point2f &pt)
{
    return (point.x - pt.x)*(point.x - pt.x)+(point.y - pt.y)*(point.y - pt.y) < MIN_DISTANCE*MIN_DISTANCE/10;
}

//使用opencv封装好的KLT算法进行运动估计，即构造特征点轨迹，从start处开始
void constructTrajectory( vector<Mat>& frameVec , vector<vector<Point2f> >& features , vector<vector<int> >& featIndex )
{
    vector<Mat> grayList;
    Mat gray;
    vector<Point2f> preFeats;
    vector<Point2f> curFeats;
    vector<uchar> status;
    vector<float> err;
    cvtColor( frameVec[0] , gray , CV_RGB2GRAY );//转化为灰度图
    grayList.push_back( gray );
    goodFeaturesToTrack( gray , preFeats , 200 , 0.1 , MIN_DISTANCE );//检测特征点
    features.push_back( preFeats );
    int feature_count = preFeats.size();
    vector<Point2f> tmpFeat;
    vector<int> tmpIndex;
    for( int i = 0 ; i < preFeats.size() ; i++ )
    {
        tmpIndex.push_back( i );
    }
    featIndex.push_back( tmpIndex );
    for( int i = 1 ; i < frameVec.size() ; i++ )
    {
        curFeats.clear();
        status.clear();
        err.clear();
        Mat gray;
        cvtColor( frameVec[i] , gray , CV_RGB2GRAY );
        grayList.push_back( gray );
        calcOpticalFlowPyrLK( grayList[0] , grayList[1] , features[i-1] , curFeats , status , err );//根据已检测到的前一帧特征点在后一帧查找匹配的特征点

        /*消除特征点误匹配*/
        vector<Point2f> prefeatures = features[i-1];
        vector<Point2f> curfeatures = curFeats;
        int ptCount = status.size();
        Mat p1(ptCount, 2, CV_32F);
        Mat p2(ptCount, 2, CV_32F);
        for (int j = 0 ; j < ptCount ; j++ )
        {
            p1.at<float>(j,0) = prefeatures[j].x;
            p1.at<float>(j,1) = prefeatures[j].y;

            p2.at<float>(j,0) = curfeatures[j].x;
            p2.at<float>(j,1) = curfeatures[j].y;
        }
        Mat m_Fundamental;
        vector<uchar> m_RANSACStatus;
        m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, FM_RANSAC);
        for (int j = 0 ; j < ptCount ; j++ )
        {
            if (m_RANSACStatus[j] == 0) // 状态为0表示野点(误匹配)
            {
                status[j] = 0;
            }
        }
        /**/

        tmpFeat.clear();
        tmpIndex.clear();
        for( int j = 0 ; j < curFeats.size() ; j++ )
        {
            if(status[j] == 0 || err[j] > 20 || outOfImg(curFeats[j], videoSize))
            {
                continue;
            }
            tmpFeat.push_back( curFeats[j] );
            tmpIndex.push_back( featIndex[i-1][j] );
        }
        features.push_back( tmpFeat );
        featIndex.push_back( tmpIndex );

        if( tmpFeat.size() < 60 )//若找到的特征点数目过少，则重新检测特征点
        {
            curFeats.clear();
            goodFeaturesToTrack( grayList[1] , curFeats , 500 , 0.1 , MIN_DISTANCE );
            for( int j = 0 ; j < curFeats.size() ; j++ )
            {
                bool flag = true;
                for( int k = 0 ; k < tmpFeat.size() ; k++ )
                {
                    if(isTheSame(curFeats[j], tmpFeat[k]))
                    {
                        flag = false;
                        break;
                    }
                }
                if( flag == true )
                {
                    features[i].push_back( curFeats[j] );
                    featIndex[i].push_back( feature_count++ );
                }
            }
        }

        grayList.erase( grayList.begin() );
    }
}

//计算两帧之间的匹配代价
float computeCostBetweenTwoImages( vector<Point2f> feat1 , vector<int> index1 , vector<Point2f> feat2 , vector<int> index2 , Size size )
{
    /*构建特征点匹配对，且如果匹配对数目少于3，则认为两帧之间的匹配代价为无穷大*/
    vector<Point2f> tmpFeat1 , tmpFeat2;
    for( int i = 0 ; i < feat1.size() ; i++ )
    {
        for( int j = 0 ; j < feat2.size() ; j++ )
        {
            if( index1[i] == index2[j] )
            {
                tmpFeat1.push_back( feat1[i] );
                tmpFeat2.push_back( feat2[j] );
                break;
            }
        }
    }
    if( tmpFeat1.size() < 3 )
        return 10000000;
    /**/

    /*特征点坐标中心化*/
    Point2f avg1, avg2, sum1=Point2f(0,0), sum2=Point2f(0,0);
    for( int i = 0 ; i < tmpFeat1.size() ; i++ )
    {
        sum1 += tmpFeat1[i];
        sum2 += tmpFeat2[i];
    }
    avg1.x = sum1.x / (float)tmpFeat1.size();
    avg1.y = sum1.y / (float)tmpFeat1.size();
    avg2.x = sum2.x / (float)tmpFeat1.size();
    avg2.y = sum2.y / (float)tmpFeat1.size();
    for( int i = 0 ; i < tmpFeat1.size() ; i++ )
    {
        tmpFeat1[i] -= avg1;
        tmpFeat2[i] -= avg2;
    }
    /**/

    //计算仿射矩阵
    Mat affine;
    affine = estimateRigidTransform( tmpFeat1 , tmpFeat2 , true );
    if( affine.empty() )
    {
        return 10000000;
    }

    //通过SVD分解求得旋转矩阵，通过特征点几何中心之差求得平移量
    Mat W, U, VT, R;
    SVD thissvd;
    thissvd.compute(affine.colRange(0,2),W,U,VT,SVD::FULL_UV);
    R = U*VT;
    Point2f shift = avg2 - avg1;

    /*计算代价costR和costO*/
    float sum = 0.0;
    for( int i = 0 ; i < tmpFeat1.size() ; i++ )
    {
        double newx, newy;
        Mat pt = Mat::zeros(2,1,CV_64FC1);
        pt.at<double>(0,0) = tmpFeat1[i].x;
        pt.at<double>(1,0) = tmpFeat1[i].y;
        pt = R * pt;
        newx = pt.at<double>(0,0) + shift.x;
        newy = pt.at<double>(1,0) + shift.y;

        sum += sqrt( pow( tmpFeat2[i].x - newx , 2 ) + pow( tmpFeat2[i].y - newy , 2 ) );
    }
    float costR , costO;
    costR = sum / (float)tmpFeat1.size();

    Point2f center;
    center.x = size.width / 2;
    center.y = size.height / 2;
    double newx, newy;
    Mat pt = Mat::zeros(2,1,CV_64FC1);
    pt.at<double>(0,0) = center.x;
    pt.at<double>(1,0) = center.y;
    pt = R * pt;
    newx = pt.at<double>(0,0) + shift.x;
    newy = pt.at<double>(1,0) + shift.y;
    costO = sqrt( pow( center.x - newx , 2 ) + pow( center.y - newy , 2 ) );
    /**/

    //返回最终代价
    float d , tauC , gamma;
    d = sqrt( pow( size.height , 2 ) + pow( size.width , 2 ) );
    tauC = 0.1 * d;
    gamma = 0.5 * d;
    if( costR < tauC )
    {
        return costO;
    }
    else
    {
        return gamma;
    }
}

//动态规划寻找一条代价最小的路径，路径上的帧即为关键帧
void pathSelection( vector<vector<float> >& costMap , vector<int>& path , int segLength )
{
    float lamdaS , lamdaA;//权重
    int v;//路径上相邻两帧之间的跨度
    lamdaS = 10;
    lamdaA = 5;
    v = VELOCITY;

    /*初始化*/
    vector<vector<float> > mapD;//存储所有可能路径的代价信息
    vector<vector<int> > mapT;//存储路径信息，回溯时使用
    for( int i = 0 ; i < GAPSIZE ; i++ )
    {
        vector<float> tmpD;
        vector<int> tmpT;
        for( int j = 0 ; j < WINDOWSIZE && ( i + j + 1 ) < segLength ; j++ )
        {
            float vCost , c;
            vCost = ( j + 1 - v ) * ( j + 1 - v );//速度代价
            if( vCost > TAUS )
                vCost = TAUS;
            c = costMap[i][j] + lamdaS * vCost;
            tmpD.push_back( c );
        }
        mapD.push_back( tmpD );
        mapT.push_back( tmpT );
    }
    /**/

    /*构造mapD，mapT*/
    for( int i = GAPSIZE ; i < segLength ; i++ )
    {
        vector<float> tmpD;
        vector<int> tmpT;
        for( int j = 0 ; j < WINDOWSIZE && ( i + j + 1 ) < segLength ; j++ )
        {
            float min = 1000000000;
            int pre;
            for( int k = 1 ; k <= WINDOWSIZE && ( i - k ) >= 0 ; k++ )
            {
                float costA , tmp;
                costA = ( j + 1 - k ) * ( j + 1 - k );//加速度代价
                if( costA > TAUA )
                    costA = TAUA;
                tmp = mapD[i-k][k-1] + lamdaA * costA;
                if( min > tmp )
                {
                    min = tmp;
                    pre = i - k;
                }
            }
            float vCost , c;
            vCost = ( j + 1 - v ) * ( j + 1 - v );//速度代价
            if( vCost > TAUS )
                vCost = TAUS;
            c = costMap[i][j] + lamdaS * vCost;
            tmpD.push_back( min + c );
            tmpT.push_back( pre );
        }
        mapD.push_back( tmpD );
        mapT.push_back( tmpT );
    }
    /**/

    /*在指定范围内查找代价之和最小的终止结点，然后通过回溯构造出完整路径*/
    //定位终止结点
    float min = 1000000000;
    int s , d;//s为代价最小路径上终止结点的前置结点(即路径倒数第二个结点)，d为终止结点
    for( int i = segLength - GAPSIZE - WINDOWSIZE ; i < segLength - 1 ; i++ )
    {
        for( int j = 0 ; j < mapD[i].size() ; j++ )
        {
            if( i + j + 1 < segLength - GAPSIZE )//终止结点只能在最后GAPSIZE个结点中选择
                continue;
            if( min > mapD[i][j] )
            {
                min = mapD[i][j];
                s = i;
                d = i + j + 1;
            }
        }
    }
    path.push_back( d );

    //回溯构造路径
    while( s >= GAPSIZE )
    {
        path.push_back( s );
        int b;
        b = mapT[s][d-s-1];
        d = s;
        s = b;
    }
    path.push_back( s );
    /**/
}


//查找以start为起点，end为终点的所有特征点轨迹，存储在容器trj中，flag为0表示首尾两片段，flag为1表示中间片段
void findCommonFeatures( int flag , vector<vector<Point2f> > &features , vector<vector<int> > &featIndex , int start , int end , vector<vector<Point2f> > &trj , vector<Point2f> &avgFeatPos )
{
    avgFeatPos.clear();
    if( flag == 0 && start == end )//若是首尾两片段，且段长为1
        return;
    if( flag == 1 && end - start == 1 )//若是中间片段，且段长为2
        return;

    /*查找轨迹*/
    for( int i = 0 ; i < featIndex[start].size() ; i++ )
    {
        int index = featIndex[start][i];
        vector<Point2f> tmp;
        tmp.push_back( features[start][i] );
        for( int j = start + 1 ; j <= end ; j++ )
        {
            bool isFound = false;
            for( int k = 0 ; k < featIndex[j].size() ; k++ )
            {
                if( index == featIndex[j][k] )
                {
                    tmp.push_back( features[j][k] );
                    isFound = true;
                    break;
                }
                if( index < featIndex[j][k] )
                {
                    isFound = false;
                    break;
                }
            }
            if( isFound == false )
            {
                break;
            }
        }
        if( tmp.size() == end - start + 1 )
        {
            trj.push_back( tmp );
        }
    }
    /**/

    if( trj.size() == 0 )
        return;

    /*计算段内每帧的几何中心*/
    for( int j = 0 ; j < end - start + 1 ; j++ )
    {
        float sum_x = 0 , sum_y = 0;
        for( int i = 0 ; i < trj.size() ; i++ )
        {
            sum_x += trj[i][j].x;
            sum_y += trj[i][j].y;
        }
        Point2f tmp;
        tmp.x = sum_x / (float)trj.size();
        tmp.y = sum_y / (float)trj.size();
        avgFeatPos.push_back( tmp );
    }
    /**/
}

//flag为0表示第1段，flag为1表示最后段，flag为2表示中间段
void computeAffine( int flag , vector<vector<Point2f> > &features , vector<vector<int> > &featIndex , int start , int end , vector<Point2f> avgFeatPos , vector<vector<Point2f> > trj , vector<Mat> &affineMatrix , vector<Mat> &affineMatrixForShakeDetect )
{
    affineMatrix.clear();
    affineMatrixForShakeDetect.clear();
    if( flag == 0 && start == end )//即第1段只有关键帧，段长为1
        return;
    if( flag == 1 && start == end )//即最后1段都只有关键帧，段长为1
        return;
    if( flag == 2 && end - start == 1 )//即中间段只有前后关键帧，段长为2
        return;
    if( trj.size() < 3 )//计算仿射矩阵至少需要三个特征点对
        return;

    //轨迹坐标中心化
    for( int i = 0 ; i < avgFeatPos.size() ; i++ )
    {
        for( int j = 0 ; j < trj.size() ; j++ )
        {
            trj[j][i] -= avgFeatPos[i];
        }
    }

    //对trj做转置，便于后续编码
    vector<vector<Point2f> > normalTrj_T;
    for( int i = 0 ; i < avgFeatPos.size() ; i++ )
    {
        vector<Point2f> tmp;
        for( int j = 0 ; j < trj.size() ; j++ )
        {
            tmp.push_back( trj[j][i] );
        }
        normalTrj_T.push_back( tmp );
    }

    if( flag == 0 )//计算其余帧到关键帧的仿射变换
    {
        for( int i = 0 ; i < avgFeatPos.size() - 1 ; i++ )
        {
            Mat affine;
            affine = estimateRigidTransform( normalTrj_T[i] , normalTrj_T[avgFeatPos.size()-1] , false );
            affineMatrix.push_back( affine );
        }
    }
    else if( flag == 1 )//计算其余帧到关键帧的仿射变换
    {
        for( int i = 1 ; i < avgFeatPos.size() ; i++ )
        {
            Mat affine;
            affine = estimateRigidTransform( normalTrj_T[i] , normalTrj_T[0] , false );
            affineMatrix.push_back( affine );
        }
    }
    else//计算非关键帧到前关键帧及前关键帧到后关键帧的仿射变换
    {
        Mat affine;
        for( int i = 1 ; i < avgFeatPos.size() - 1 ; i++ )
        {
            affine = estimateRigidTransform( normalTrj_T[i] , normalTrj_T[0] , false );
            affineMatrix.push_back( affine );
        }
        affine = estimateRigidTransform( normalTrj_T[0] , normalTrj_T[avgFeatPos.size()-1] , false );
        affineMatrix.push_back( affine );
    }

    for( int h = 0 ; h < affineMatrix.size() ; h++ )
    {
        if( affineMatrix[h].empty() )
        {
            //affineMatrix.clear();
            //break;
        }
    }

    //抖动检测所用代码，计算所有相邻两帧关键帧之间的仿射变换
    for( int i = 0 ; i < avgFeatPos.size() - 1 ; i++ )
    {
        Mat affine;
        affine = estimateRigidTransform( normalTrj_T[i] , normalTrj_T[i+1] , false );
        affineMatrixForShakeDetect.push_back( affine );
    }
}

//SIFT特征点检测与匹配
void findSiftFeatureMatches( Mat &gray1 , Mat &gray2 , vector<Point2f> &ps , vector<Point2f> &pe )
{
    const float minRatio=1.f/1.5f;//用于删除不可靠的特征点对

    //SIFT特征点检测
    SiftFeatureDetector detector;
    std::vector<KeyPoint> kp1, kp2;
    detector.detect( gray1, kp1 );
    detector.detect( gray2, kp2 );

    //提取特征点描述子
    SiftDescriptorExtractor extractor;
    Mat des1,des2;//descriptor
    extractor.compute(gray1,kp1,des1);
    extractor.compute(gray2,kp2,des2);

    //匹配特征点，并删除不可靠的特征点对
    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    vector<vector<DMatch> > m_knnMatches;
    matcher.knnMatch(des1,des2,m_knnMatches,2);
    for (int i=0; i<m_knnMatches.size(); i++)
    {
        const DMatch& bestMatch=m_knnMatches[i][0];
        const DMatch& betterMatch=m_knnMatches[i][1];

        float distanceRatio=bestMatch.distance/betterMatch.distance;

        if (distanceRatio<minRatio)
        {
            matches.push_back(bestMatch);
        }
    }

    for( int j = 0 ; j < matches.size() ; j++ )
    {
        ps.push_back(kp1[matches[j].queryIdx].pt);
        pe.push_back(kp2[matches[j].trainIdx].pt);
    }
}

//BRIEF特征点检测与匹配
void findBriefFeatureMatches( Mat &gray1 , Mat &gray2 , vector<Point2f> &ps , vector<Point2f> &pe )
{
    // -- Step 1: Detect the keypoints using STAR Detector
    std::vector<KeyPoint> keypoints_1,keypoints_2;
    StarDetector detector;
    detector.detect(gray1, keypoints_1);
    detector.detect(gray2, keypoints_2);

    // -- Stpe 2: Calculate descriptors (feature vectors)
    BriefDescriptorExtractor brief;
    Mat descriptors_1, descriptors_2;
    brief.compute(gray1, keypoints_1, descriptors_1);
    brief.compute(gray2, keypoints_2, descriptors_2);

    //-- Step 3: Matching descriptor vectors with a brute force matcher
    BFMatcher matcher(NORM_HAMMING);
    std::vector<DMatch> matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    vector<Point2f> tmpps,tmppe;
    for( int j = 0 ; j < matches.size() ; j++ )
    {
        tmpps.push_back(keypoints_1[matches[j].queryIdx].pt);
        tmppe.push_back(keypoints_2[matches[j].trainIdx].pt);
    }

    /*消除特征点误匹配*/
    vector<Point2f> prefeatures = tmpps;
    vector<Point2f> curfeatures = tmppe;
    int ptCount = matches.size();
    Mat p1(ptCount, 2, CV_32F);
    Mat p2(ptCount, 2, CV_32F);
    for (int j = 0 ; j < ptCount ; j++ )
    {
        p1.at<float>(j,0) = prefeatures[j].x;
        p1.at<float>(j,1) = prefeatures[j].y;

        p2.at<float>(j,0) = curfeatures[j].x;
        p2.at<float>(j,1) = curfeatures[j].y;
    }
    Mat m_Fundamental;
    vector<uchar> m_RANSACStatus;
    m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, FM_RANSAC);
    for (int j = 0 ; j < ptCount ; j++ )
    {
        if (m_RANSACStatus[j] != 0) // 状态为0表示野点(误匹配)
        {
            ps.push_back( tmpps[j] );
            pe.push_back( tmppe[j] );
        }
    }
    /**/
}

//若找不到足够数量(>=3)的横跨整个片段的轨迹，则重新用SIFT特征匹配估计帧与帧之间的仿射变换
void computeAffineForZeroTrj( int flag , vector<Mat> &frameVec , int start , int end , vector<Mat> &affineMatrixForZeroTrj , vector<Point2f> &shiftVecForZeroTrj , vector<Point2f> &avgFeatPosForZeroTrj )
{
    affineMatrixForZeroTrj.clear();
    shiftVecForZeroTrj.clear();
    avgFeatPosForZeroTrj.clear();
    if( flag == 0 && start == end )//即第1段只有关键帧
        return;
    if( flag == 1 && start == end )//即最后1段都只有关键帧
        return;
    if( flag == 2 && end - start == 1 )//即中间段只有前后关键帧
        return;

    if( flag == 0 )//计算其余帧到关键帧的平移量和仿射变换
    {
        Mat img1 , img2 , gray1 , gray2;
        img2 = frameVec[end].clone();
        cvtColor( img2 , gray2 , CV_RGB2GRAY );
        for( int i = start ; i < end ; i ++ )
        {
            img1 = frameVec[i].clone();
            cvtColor( img1 , gray1 , CV_RGB2GRAY );

            vector<Point2f> ps , pe;
            //sift特征点检测与匹配
            //findSiftFeatureMatches( gray1 , gray2 , ps , pe );

            //BRIEF特征点检测与匹配
            findBriefFeatureMatches( gray1 , gray2 , ps , pe );

            if( !ps.empty() )
            {
                //特征点坐标中心化
                Point2f sum_ps , sum_pe , avg_ps , avg_pe;
                sum_ps = Point2f(0,0);
                sum_pe = Point2f(0,0);
                for( int j = 0 ; j < ps.size() ; j++ )
                {
                    sum_ps += ps[j];
                    sum_pe += pe[j];
                }
                avg_ps.x = sum_ps.x / (float)ps.size();
                avg_ps.y = sum_ps.y / (float)ps.size();
                avg_pe.x = sum_pe.x / (float)pe.size();
                avg_pe.y = sum_pe.y / (float)pe.size();

                shiftVecForZeroTrj.push_back( avg_pe - avg_ps );

                for( int j = 0 ; j < ps.size() ; j++ )
                {
                    ps[j] = ps[j] - avg_ps;
                    pe[j] = pe[j] - avg_pe;
                }
                Mat affine = estimateRigidTransform( ps , pe , false );//计算仿射变换
                affineMatrixForZeroTrj.push_back( affine );
                avgFeatPosForZeroTrj.push_back( avg_ps );
            }
            else
            {
                shiftVecForZeroTrj.push_back( Point2f(0,0) );
                affineMatrixForZeroTrj.push_back( Mat::eye(2,2,CV_64FC1) );
                avgFeatPosForZeroTrj.push_back( Point2f(0,0) );
            }
        }
    }
    else if( flag == 1 )//计算其余帧到关键帧的平移量和仿射变换
    {
        Mat img1 , img2 , gray1 , gray2;
        img2 = frameVec[start].clone();
        cvtColor( img2 , gray2 , CV_RGB2GRAY );
        for( int i = start + 1 ; i <= end ; i ++ )
        {
            img1 = frameVec[i].clone();
            cvtColor( img1 , gray1 , CV_RGB2GRAY );

            vector<Point2f> ps , pe;
            //sift特征点检测与匹配
            //findSiftFeatureMatches( gray1 , gray2 , ps , pe );

            //BRIEF特征点检测与匹配
            findBriefFeatureMatches( gray1 , gray2 , ps , pe );

            if( !ps.empty() )
            {
                //特征点坐标中心化
                Point2f sum_ps , sum_pe , avg_ps , avg_pe;
                sum_ps = Point2f(0,0);
                sum_pe = Point2f(0,0);
                for( int j = 0 ; j < ps.size() ; j++ )
                {
                    sum_ps += ps[j];
                    sum_pe += pe[j];
                }
                avg_ps.x = sum_ps.x / (float)ps.size();
                avg_ps.y = sum_ps.y / (float)ps.size();
                avg_pe.x = sum_pe.x / (float)pe.size();
                avg_pe.y = sum_pe.y / (float)pe.size();
                shiftVecForZeroTrj.push_back( avg_pe - avg_ps );

                for( int j = 0 ; j < ps.size() ; j++ )
                {
                    ps[j] = ps[j] - avg_ps;
                    pe[j] = pe[j] - avg_pe;
                }
                Mat affine = estimateRigidTransform( ps , pe , false );//计算仿射变换
                affineMatrixForZeroTrj.push_back( affine );
                avgFeatPosForZeroTrj.push_back( avg_ps );
            }
            else
            {
                shiftVecForZeroTrj.push_back( Point2f(0,0) );
                affineMatrixForZeroTrj.push_back( Mat::eye(2,2,CV_64FC1) );
                avgFeatPosForZeroTrj.push_back( Point2f(0,0) );
            }
        }
    }
    else//计算非关键帧到插值后位置的平移量，计算非关键帧到前关键帧及前关键帧到后关键帧的仿射变换
    {
        vector<Point2f> i2sShift;
        Mat img1 , img2 , gray1 , gray2;
        img2 = frameVec[start].clone();
        cvtColor( img2 , gray2 , CV_RGB2GRAY );
        for( int i = start + 1 ; i < end ; i ++ )
        {
            img1 = frameVec[i].clone();
            cvtColor( img1 , gray1 , CV_RGB2GRAY );

            vector<Point2f> ps , pe;
            //sift特征点检测与匹配
            //findSiftFeatureMatches( gray1 , gray2 , ps , pe );

            //BRIEF特征点检测与匹配
            findBriefFeatureMatches( gray1 , gray2 , ps , pe );

            if( !ps.empty() )
            {
                //特征点坐标中心化
                Point2f sum_ps , sum_pe , avg_ps , avg_pe;
                sum_ps = Point2f(0,0);
                sum_pe = Point2f(0,0);
                for( int j = 0 ; j < ps.size() ; j++ )
                {
                    sum_ps += ps[j];
                    sum_pe += pe[j];
                }
                avg_ps.x = sum_ps.x / (float)ps.size();
                avg_ps.y = sum_ps.y / (float)ps.size();
                avg_pe.x = sum_pe.x / (float)pe.size();
                avg_pe.y = sum_pe.y / (float)pe.size();
                i2sShift.push_back( avg_pe - avg_ps );

                for( int j = 0 ; j < ps.size() ; j++ )
                {
                    ps[j] = ps[j] - avg_ps;
                    pe[j] = pe[j] - avg_pe;
                }
                Mat affine = estimateRigidTransform( ps , pe , false );//计算仿射变换
                affineMatrixForZeroTrj.push_back( affine );
                avgFeatPosForZeroTrj.push_back( avg_ps );
            }
            else
            {
                i2sShift.push_back( Point2f(0,0) );
                affineMatrixForZeroTrj.push_back( Mat::eye(2,2,CV_64FC1) );
                avgFeatPosForZeroTrj.push_back( Point2f(0,0) );
            }
        }
        img1 = frameVec[start].clone();
        img2 = frameVec[end].clone();
        cvtColor( img1 , gray1 , CV_RGB2GRAY );
        cvtColor( img2 , gray2 , CV_RGB2GRAY );

        vector<Point2f> ps , pe;
        //sift特征点检测与匹配
        //findSiftFeatureMatches( gray1 , gray2 , ps , pe );

        //BRIEF特征点检测与匹配
        findBriefFeatureMatches( gray1 , gray2 , ps , pe );

        if( !ps.empty() )
        {
            //特征点坐标中心化
            Point2f sum_ps , sum_pe , avg_ps , avg_pe;
            sum_ps = Point2f(0,0);
            sum_pe = Point2f(0,0);
            for( int j = 0 ; j < ps.size() ; j++ )
            {
                sum_ps += ps[j];
                sum_pe += pe[j];
            }
            avg_ps.x = sum_ps.x / (float)ps.size();
            avg_ps.y = sum_ps.y / (float)ps.size();
            avg_pe.x = sum_pe.x / (float)pe.size();
            avg_pe.y = sum_pe.y / (float)pe.size();
            Point2f s2eShift = avg_pe - avg_ps;
            for( int j = 0 ; j < i2sShift.size() ; j++ )
            {
                Point2f tmp;
                tmp.x = (float)( j + 1 ) * s2eShift.x / (float)( end - start );
                tmp.y = (float)( j + 1 ) * s2eShift.y / (float)( end - start );
                shiftVecForZeroTrj.push_back( i2sShift[j] + tmp );
            }

            for( int j = 0 ; j < ps.size() ; j++ )
            {
                ps[j] = ps[j] - avg_ps;
                pe[j] = pe[j] - avg_pe;
            }
            Mat affine = estimateRigidTransform( ps , pe , false );//计算仿射变换
            affineMatrixForZeroTrj.push_back( affine );
        }
        else
        {
            shiftVecForZeroTrj.push_back( Point2f(0,0) );
            affineMatrixForZeroTrj.push_back( Mat::eye(2,2,CV_64FC1) );
        }

    }
    int count1 = 0;
    for( int i = 0 ; i < affineMatrixForZeroTrj.size() ; i++ )
    {
        if( affineMatrixForZeroTrj[i].empty() )
        {
            count1++;
        }
    }
    if( count1 > 0 )
	{

	}
}

/*为抖动检测计算平移、旋转和尺度振幅*/
void computeAmplitudeForShakeDetect( vector<Point2f> &avgFeatPos ,
                                     vector<Mat> &affineMatrixForShakeDetect ,
                                     int m ,
                                     vector<float> &shift_SD ,
                                     vector<double> &rotate_SD ,
                                     vector<double> &scale_SD )
{
    float cosinlim = 0.996;
    /*平移*/
    Point2f d1 , d2;
    d1 = avgFeatPos[m+1] - avgFeatPos[m];
    d2 = avgFeatPos[m+2] - avgFeatPos[m+1];
    //判断d1与d2方向是否一致（向量夹角小于一阈值），若一致，输出0
    float d1_mu , d2_mu;
    d1_mu = sqrt( d1.x * d1.x + d1.y * d1.y );
    d2_mu = sqrt( d2.x * d2.x + d2.y * d2.y );
    float cosin = ( d1.x * d2.x + d1.y * d2.y ) / ( d1_mu * d2_mu );
    if( cosin > cosinlim )
    {
        shift_SD.push_back(0);
    }
    else
    {
        Point2f d = d2 - d1;
        d.x = d.x / 2.0;
        d.y = d.y / 2.0;
        float d_mu;
        d_mu = sqrt( d.x * d.x + d.y * d.y );
        shift_SD.push_back(d_mu);
    }
    /**/
    /*旋转和尺度*/
    if( !affineMatrixForShakeDetect[m].empty() && !affineMatrixForShakeDetect[m+1].empty() )
    {
        Mat affine1 = affineMatrixForShakeDetect[m].colRange(0,2);
        Mat affine2 = affineMatrixForShakeDetect[m+1].colRange(0,2);

        SVD thissvd;
        Mat W1, U1, VT1, R1 , S1 , WW1;
        thissvd.compute(affine1,W1,U1,VT1,SVD::FULL_UV);
        R1 = U1*VT1;
        WW1 = Mat::zeros(2,2,CV_64FC1);
        WW1.at<double>(0,0) = W1.at<double>(0,0);
        WW1.at<double>(1,1) = W1.at<double>(1,0);
        S1 = VT1.t()*WW1*VT1;//对角矩阵，且对角线上数相等
        double theta1;
        theta1 = asinf( R1.at<double>(0,1) );
        double s1;
        s1 = S1.at<double>(0,0);

        Mat W2, U2, VT2, R2 , S2 , WW2;
        thissvd.compute(affine2,W2,U2,VT2,SVD::FULL_UV);
        R2 = U2*VT2;
        WW2 = Mat::zeros(2,2,CV_64FC1);
        WW2.at<double>(0,0) = W2.at<double>(0,0);
        WW2.at<double>(1,1) = W2.at<double>(1,0);
        S2 = VT2.t()*WW2*VT2;//对角矩阵，且对角线上数相等
        double theta2;
        theta2 = asinf( R2.at<double>(0,1) );
        double s2;
        s2 = S2.at<double>(0,0);

        //旋转
        if( ( theta1 > 0 && theta2 > 0 ) || ( theta1 < 0 && theta2 < 0 ) )
        {
            rotate_SD.push_back(0);
        }
        else
        {
            rotate_SD.push_back(abs(theta1-theta2));
        }

        //尺度
        if( ( s1 > 1 && s2 > 1 ) || ( s1 < 1 && s2 < 1 ) )
        {
            scale_SD.push_back(0);
        }
        else
        {
            scale_SD.push_back(abs( s2/s1 - 1.0 ));
        }
    }
    /**/
}

//运动补偿计算，processCount为0表示第一次处理，为1表示中间次数处理，为2表示最后一次处理
void frameSmooth( vector<Mat>& frameVec , vector<vector<float> >& costMap , Size videoSize , int processCount )
{
    vector<vector<Point2f> > features;//存储每一帧特征点坐标
    vector<vector<int> > featIndex;//存储每个特征点对应轨迹的编号
    //shiftVec.clear();
    //affineVec.clear();
    constructTrajectory( frameVec , features , featIndex );//运动估计,构造特征点轨迹

    int gap;//前一次处理最后一个关键帧与下一次处理第一个关键帧之间至少间隔gap帧
    if( processCount == 0 )
    {
        gap = 0;
    }
    else
    {
        gap = VELOCITY;
    }
    for( int i = gap ; i < frameVec.size() ; i++ )//帧与其后续若干帧计算帧匹配代价
    {
        vector<float> tmp;
        for( int j = i + 1 ; j <= i + WINDOWSIZE && j < frameVec.size() ; j++ )
        {
            float cost;
            cost = computeCostBetweenTwoImages( features[i] , featIndex[i] , features[j] , featIndex[j] , videoSize );
            tmp.push_back( cost );
        }
        costMap.push_back( tmp );
    }

    //根据帧匹配代价利用动态规划寻找一条代价最小的路径(路径上的帧即为关键帧)
    vector<int> path;
    pathSelection( costMap , path , frameVec.size() - gap );//得到的path顺序是反的

    if( processCount != 0 )//关键帧索引矫正使其与frameVec对应，同时将第一帧（前次处理最后一个关键帧）作为首关键帧
    {
        for( int i = 0 ; i < path.size() ; i++ )
        {
            path[i] += gap;
        }
        path.push_back(0);
    }

    for( int i = path.size() - 1 ; i >= 0 ; i-- )
    {
        int frame_index = path[i];
    }
    float tap = (float)frameVec.size()/(float)path.size();

    /*按平滑路径稳定视频帧*/
    int start , end;//每一段的起点和终点（包括关键帧）
    int j = path.size() - 1;
    vector<Point2f> avgFeatPos;//存储段内每帧的几何中心坐标
    vector<Mat> affineMatrix;//存储帧与帧之间的仿射变换
    /*在KLT特征轨迹数不足时使用SIFT特征点重新计算*/
    vector<Mat> affineMatrixForZeroTrj;//存储帧与帧之间的仿射变换
    vector<Point2f> shiftVecForZeroTrj;//存储非关键帧到插值位置的平移量
    vector<Point2f> avgFeatPosForZeroTrj;//存储非关键帧的几何中心坐标
    /**/
    vector<Mat> affineMatrixForShakeDetect;//存储相邻两帧之间的抖动变换

    int seg_num;//除非是最后一次处理，否则不处理最后一段
    if( processCount == 2 )
    {
        seg_num = path.size() + 1;
    }
    else
    {
        seg_num = path.size();
    }
    for( int i = 0 ; i < seg_num ; i++ )
    {
        int flag;//段类型标记，0表示第1段，1表示最后1段，2表示中间段
        if( i == 0 )//第1段，起始帧到第一个关键帧
        {
            flag = 0;
            start = 0;
            end = path[j];
            vector<vector<Point2f> > trj;
            findCommonFeatures( 0 , features , featIndex , start , end , trj , avgFeatPos );

            /*为旋转插值准备数据，即计算仿射矩阵，计算本段其他帧到关键帧的仿射矩阵*/
            computeAffine( 0 , features , featIndex , start , end , avgFeatPos , trj , affineMatrix , affineMatrixForShakeDetect);
            if( affineMatrix.size() == 0 )
            {
                //file<<"SIFT"<<endl;
                computeAffineForZeroTrj( 0 , frameVec , start , end ,affineMatrixForZeroTrj , shiftVecForZeroTrj , avgFeatPosForZeroTrj );
            }
            /**/
        }
        else if( i == path.size() )//最后1段，最后一个关键帧到终止帧
        {
            flag = 1;
            start = path[j];
            end = features.size() - 1;
            vector<vector<Point2f> > trj;
            findCommonFeatures( 0 , features , featIndex , start , end , trj , avgFeatPos );

            /*为旋转插值准备数据，即计算仿射矩阵，计算本段其他帧到关键帧的仿射矩阵*/
            computeAffine( 1 , features , featIndex , start , end , avgFeatPos , trj , affineMatrix , affineMatrixForShakeDetect);
            if( affineMatrix.size() == 0 )
            {
                //file<<"SIFT"<<endl;
                computeAffineForZeroTrj( 1 , frameVec , start , end ,affineMatrixForZeroTrj , shiftVecForZeroTrj , avgFeatPosForZeroTrj );
            }
            /**/
        }
        else//中间段，相邻两个关键帧之间的帧组成的段
        {
            flag = 2;
            start = path[j];
            end = path[j-1];
            vector<vector<Point2f> > trj;
            findCommonFeatures( 1 , features , featIndex , start , end , trj , avgFeatPos );

            /*为旋转插值准备数据，即计算仿射矩阵，计算本段非关键帧到前关键帧的仿射矩阵与前关键帧到后关键帧的仿射矩阵*/
            computeAffine( 2 , features , featIndex , start , end , avgFeatPos , trj , affineMatrix ,affineMatrixForShakeDetect);
            if( affineMatrix.size() == 0 )
            {
                //file<<"SIFT"<<endl;
                computeAffineForZeroTrj( 2 , frameVec , start , end ,affineMatrixForZeroTrj , shiftVecForZeroTrj , avgFeatPosForZeroTrj );
            }
            /**/
            j--;
        }

        int s , e;//每一段起点和终点（不包括关键帧）
        if( flag == 0 )
        {
            s = start;
            e = end - 1;
        }
        if( flag == 1 )
        {
            s = start + 1;
            e = end;
        }
        if( flag == 2 )
        {
            s = start + 1;
            e = end - 1;
        }
        vector<Point2f> preF;
        Mat tmpimg;

        vector<float> shift_SD;//抖动检测参数存储
        vector<double> rotate_SD;//抖动检测参数存储
        vector<double> scale_SD;//抖动检测参数存储
        vector<Mat> segAffine;//在决定是否去抖前临时存储帧变换
        vector<Mat> segImg;//在决定是否去抖前临时存储变换后的帧
        vector<Mat> segImg_ori;//在决定是否去抖前临时存储原始帧

        for( int k = s , m = 0 ; k <= e ; k++ , m++ )//计算段内非关键帧的运动补偿
        {
            tmpimg = frameVec[k].clone();
            Point2f shift;//平移补偿
            Mat affine;//旋转补偿
            if( affineMatrix.size() == 0 )//即在此段内横跨整段的轨迹条数小于3
            {
                shift = Point2f(0.0,0.0);
                affine = Mat::zeros(2,3,CV_64FC1);
                affine.at<double>(0,0) = 1;
                affine.at<double>(1,1) = 1;

                //计算平移旋转补偿
                shift = shiftVecForZeroTrj[m];//平移补偿量
                if( flag == 0 )//第1段
                {
                    if( affineMatrixForZeroTrj[m].empty() )
                    {
                    }
                    else
                    {
                        //通过奇异值分解得到旋转矩阵
                        affine = affineMatrixForZeroTrj[m].colRange(0,2);
                        Mat W, U, VT, R;
                        SVD thissvd;
                        thissvd.compute(affine,W,U,VT,SVD::FULL_UV);
                        R = U*VT;

                        //计算旋转补偿量
                        double theta , degree;
                        theta = asinf( R.at<double>(0,1) );
                        degree = theta * 180 / 3.1415926;//弧度转角度
                        affine = getRotationMatrix2D( avgFeatPosForZeroTrj[m]+shift , degree , 1 );
                    }
                }
                if( flag == 1 )//最后1段
                {
                    if( affineMatrixForZeroTrj[m].empty() )
                    {
                    }
                    else
                    {
                        //通过奇异值分解得到旋转矩阵
                        affine = affineMatrixForZeroTrj[m].colRange(0,2);
                        Mat W, U, VT, R;
                        SVD thissvd;
                        thissvd.compute(affine,W,U,VT,SVD::FULL_UV);
                        R = U*VT;

                        //计算旋转补偿量
                        double theta , degree;
                        theta = asinf( R.at<double>(0,1) );
                        degree = theta * 180 / 3.1415926;//弧度转角度
                        affine = getRotationMatrix2D( avgFeatPosForZeroTrj[m]+shift , degree , 1 );
                    }
                }
                if( flag == 2 )//中间段
                {
                    if( affineMatrixForZeroTrj[m].empty() || affineMatrixForZeroTrj[affineMatrixForZeroTrj.size()-1].empty() )
                    {
                    }
                    else
                    {
                        //通过奇异值分解得到前关键帧到后关键帧的旋转矩阵，并通过线性插值计算前关键帧到稳定位置的旋转量
                        Mat s2eAffine, W, U, VT, R, S, WW;
                        s2eAffine = affineMatrixForZeroTrj[affineMatrixForZeroTrj.size()-1].colRange(0,2);
                        SVD thissvd;
                        thissvd.compute(s2eAffine,W,U,VT,SVD::FULL_UV);
                        R = U * VT;
                        double theta , delta_theta;
                        theta = asinf( R.at<double>(0,1) );
                        delta_theta = ( m + 1 ) * theta / (double)( e - s + 2 );

                        //通过奇异值分解得到非关键帧到前关键帧的旋转矩阵
                        Mat i2sAffine;
                        i2sAffine = affineMatrixForZeroTrj[m].colRange(0,2);
                        thissvd.compute(i2sAffine,W,U,VT,SVD::FULL_UV);
                        i2sAffine = U * VT;
                        theta = asinf( i2sAffine.at<double>(0,1) );

                        //计算旋转补偿量
                        double degree;
                        degree = theta + delta_theta;
                        degree = degree * 180 / 3.1415926;
                        affine = getRotationMatrix2D( avgFeatPosForZeroTrj[m]+shift , degree , 1 );
                    }
                }
            }
            else
            {
                if( flag == 0 )//第1段
                {
                    shift = avgFeatPos[avgFeatPos.size()-1] - avgFeatPos[m];//平移补偿量
                    //通过奇异值分解得到旋转矩阵
                    affine = affineMatrix[m].colRange(0,2);
                    Mat W, U, VT, R;
                    SVD thissvd;
                    thissvd.compute(affine,W,U,VT,SVD::FULL_UV);
                    R = U*VT;

                    //计算旋转补偿量
                    double theta , degree;
                    theta = asinf( R.at<double>(0,1) );
                    degree = theta * 180 / 3.1415926;
                    affine = getRotationMatrix2D( avgFeatPos[m]+shift , degree , 1 );
                }
                if( flag == 1 )//最后1段
                {
                    shift = avgFeatPos[0] - avgFeatPos[m+1];//平移补偿量
                    //通过奇异值分解得到旋转矩阵
                    affine = affineMatrix[m].colRange(0,2);
                    Mat W, U, VT, R;
                    SVD thissvd;
                    thissvd.compute(affine,W,U,VT,SVD::FULL_UV);
                    R = U*VT;

                    //计算旋转补偿量
                    double theta , degree;
                    theta = asinf( R.at<double>(0,1) );
                    degree = theta * 180 / 3.1415926;
                    affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
                }
                if( flag == 2 )//中间段
                {
                    if( needShakeDetect )
                    {
                        //为抖动检测计算振幅
                        computeAmplitudeForShakeDetect( avgFeatPos ,affineMatrixForShakeDetect ,m , shift_SD ,rotate_SD , scale_SD );
                    }
                    //平移补偿计算
                    Point2f tmp;
                    tmp = avgFeatPos[avgFeatPos.size()-1] - avgFeatPos[0];
                    tmp.x = ( m + 1 ) * tmp.x / (float)( avgFeatPos.size() - 1 );
                    tmp.y = ( m + 1 ) * tmp.y / (float)( avgFeatPos.size() - 1 );
                    shift = avgFeatPos[0] - avgFeatPos[m+1] + tmp;

                    //旋转补偿计算
                    if( affineMatrix[affineMatrix.size()-1].empty() || affineMatrix[m].empty() )
                    {
                        affine = Mat::zeros(2,3,CV_64FC1);
                        affine.at<double>(0,0) = 1;
                        affine.at<double>(1,1) = 1;
                    }
                    else
                    {
                        //通过奇异值分解得到前关键帧到后关键帧的旋转矩阵，并通过线性插值计算前关键帧到稳定位置的旋转量
                        Mat s2eAffine, W, U, VT, R;
                        s2eAffine = affineMatrix[affineMatrix.size()-1].colRange(0,2);
                        SVD thissvd;
                        thissvd.compute(s2eAffine,W,U,VT,SVD::FULL_UV);
                        R = U*VT;
                        double theta , delta_theta;
                        theta = asinf( R.at<double>(0,1) );
                        delta_theta = ( m + 1 ) * theta / (double)( avgFeatPos.size() - 1 );

                        //通过奇异值分解得到非关键帧到前关键帧的旋转矩阵
                        Mat i2sAffine , W2 , U2 , VT2;
                        i2sAffine = affineMatrix[m].colRange(0,2);
                        thissvd.compute(i2sAffine,W2,U2,VT2,SVD::FULL_UV);//计算旋转矩阵
                        i2sAffine = U2*VT2;
                        theta = asinf( i2sAffine.at<double>(0,1) );

                        //计算旋转补偿量
                        double degree;
                        degree = theta + delta_theta;
                        degree = degree * 180 / 3.1415926;
                        affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
                    }
                }
            }

            //平移补偿转化为齐次矩阵形式
            Mat shiftAffine = Mat::zeros(3,3,CV_64FC1);
            shiftAffine.at<double>(0,0) = 1;
            shiftAffine.at<double>(1,1) = 1;
            shiftAffine.at<double>(0,2) = shift.x;
            shiftAffine.at<double>(1,2) = shift.y;
            shiftAffine.at<double>(2,2) = 1;

            //旋转补偿转化为齐次矩阵形式
            Mat tmpaff = Mat::zeros(3,3,CV_64FC1);
            tmpaff.at<double>(0,0) = affine.at<double>(0,0);
            tmpaff.at<double>(0,1) = affine.at<double>(0,1);
            tmpaff.at<double>(0,2) = affine.at<double>(0,2);
            tmpaff.at<double>(1,0) = affine.at<double>(1,0);
            tmpaff.at<double>(1,1) = affine.at<double>(1,1);
            tmpaff.at<double>(1,2) = affine.at<double>(1,2);
            tmpaff.at<double>(2,2) = 1;

            Mat resultAffine;//最终补偿变换
            resultAffine = tmpaff * shiftAffine ;
            resultAffine = resultAffine.rowRange(0,2);

            Mat out;//稳定帧
            warpAffine( tmpimg , out , resultAffine , videoSize ,1,0,Scalar(0,0,0));

            if( !needShakeDetect || flag != 2 )
            {
            }
            else
            {
                segImg.push_back( out );
                segImg_ori.push_back( tmpimg );
            }
        }

        if( needShakeDetect && flag == 2 )//只对中间段进行去抖判断
        {
            /*判断是否去抖*/
            //计算平均平移振幅
            float avg_shift_SD;
            double avg_rotate_SD , avg_scale_SD;
            float sum = 0;
            for( int SD = 0 ; SD < shift_SD.size() ; SD++ )
            {
                sum += shift_SD[SD];
            }
            avg_shift_SD = sum / (float)shift_SD.size();
            //计算平均旋转振幅
            double sum2 = 0;
            for( int SD = 0 ; SD < rotate_SD.size() ; SD++ )
            {
                sum2 += rotate_SD[SD];
            }
            avg_rotate_SD = sum2 / (float)rotate_SD.size();
            //计算平均尺度振幅
            sum2 = 0;
            for( int SD = 0 ; SD < scale_SD.size() ; SD++ )
            {
                sum2 += scale_SD[SD];
            }
            avg_scale_SD = sum2 / (float)scale_SD.size();

            if( avg_shift_SD > SHIFT_SD || avg_rotate_SD > ROTATE_SD || avg_scale_SD > SCALE_SD )//去抖
            {
                for( int SD = 0 ; SD < segAffine.size() ; SD++ )
                {
                    //imshow(stabWindow,segImg[SD]);
                    outImg.image = segImg[SD];
                    outImg.encoding = "bgr8";
                    pub.publish(outImg.toImageMsg());
                    waitKey(1);
                }
            }
            else
            {
                Mat tmpaff = Mat::zeros(2,3,CV_64FC1);
                tmpaff.at<double>(0,0) = 1;
                tmpaff.at<double>(1,1) = 1;
                for( int SD = 0 ; SD < segAffine.size(); SD++ )
                {
                    //imshow(stabWindow,segImg_ori[SD]);
                    outImg.image = segImg_ori[SD];
                    outImg.encoding = "bgr8";
                    pub.publish(outImg.toImageMsg());
                    waitKey(1);
                }
            }
            /**/
        }

        if( ( flag == 0 && processCount == 0 ) || flag == 2 )//关键帧不进行稳定处理
        {
            Mat result = frameVec[end].clone();
            //imshow(stabWindow,result);
            outImg.image = result;
            outImg.encoding = "bgr8";
            pub.publish(outImg.toImageMsg());
            waitKey(1);
        }
    }

    features.clear();
    featIndex.clear();
    costMap.clear();
    if( processCount != 2 )
    {
        frameVec.erase( frameVec.begin() , frameVec.begin() + path[0] );
    }
    else
    {
        frameVec.clear();
    }
}

// The end of stable video
int count22;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat cameraFeed = cv_ptr->image;

    Mat image;
    image = cameraFeed.clone();
    frameVec.push_back( image );

    videoSize.width = image.cols;
    videoSize.height = image.rows;

    if( frameVec.size() >= SEGSIZE )
    {
        if( count22 == 0)
        {
            processCount = 0;//第一次处理
        }
        else
        {
            processCount = 1;//中间次数处理
        }
        frameSmooth( frameVec , costMap , videoSize , processCount );

        count22++;
    }
    if( frameVec.size() >= GAPSIZE + 2 * WINDOWSIZE )
    {
        if( count22 == 0)//第一次处理
        {
            processCount = 0;
        }
        else
        {
            processCount = 2;//中间次数处理
        }
        frameSmooth( frameVec , costMap , videoSize , processCount );
    }
}

int main(int argc, char *argv[])
{

	image_transport::ImageTransport it(nh);

	pub = it.advertise("stable/image", 3);

    ros::init(argc, argv, "StableVideo");

    image_transport::Subscriber imgSub;

    imgSub = it.subscribe("/uav_cam/iamge", 5, imageCallback);

    ros::spin();

    return 0;
}
