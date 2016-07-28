#ifndef THREAD_H
#define THREAD_H

#include <QThread>
#include <QSemaphore>
#include <opencv.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

#define SEGSIZE 30        //每次处理帧数
#define MIN_DISTANCE 20
#define BUFFERSIZE SEGSIZE*4

extern Mat frameVec[BUFFERSIZE];//原始帧容器
extern vector<Mat> stableVec;//稳定帧容器
extern QSemaphore *semReadBuffer , *semMotionEst , *semMotionComp , *semShowandSave;
extern list<list<vector<Point2f> > > trj;//存储特征点轨迹
extern vector<Point2i> motionEstList;//存储每次运动估计时用到的帧在frameVec中的位置，x表示起点，y表示长度
extern vector<Point2i> motionCompList;//存储每次运动补偿时用到的帧在frameVec中的位置，x表示起点，y表示长度
extern vector<Point2i> showandSaveList;//存储每次显示和保存时用到的帧在frameVec中的位置，x表示起点，y表示长度

//extern ofstream file;

//读取视频帧线程
class ThreadRead : public QThread
{
public:
    VideoCapture videoCap;
    bool m_stop;
    int bufferIndex;//指向这一次读取的帧在frameVec中存放的位置

    void init()
    {
        m_stop = false;
        bufferIndex = 0;
    }

    void stop()
    {
        m_stop = true;
    }

    void run()
    {
        bool firstRead = true;//是否是读视频首个SEGSIZE帧
        bool readFinished = false;
        while(true)
        {
            semReadBuffer->acquire();

            Point2i tmp;
            int readNum;
            if( firstRead )
            {
                readNum = SEGSIZE;
                tmp.x = bufferIndex;
            }
            else
            {
                readNum = SEGSIZE - 1;
                if( bufferIndex > 0 )
                {
                    tmp.x = bufferIndex - 1;
                }
                else
                {
                    tmp.x = BUFFERSIZE - 1;
                }
            }
            int i;
            for( i = 0 ; i < readNum ; i++ )
            {
                Mat frame;
                if( videoCap.read(frame) && !m_stop )
                {
                    frameVec[bufferIndex++] = frame;
                    if( bufferIndex == BUFFERSIZE )
                    {
                        bufferIndex = 0;
                    }
                }
                else
                {
                    readFinished = true;
                    break;
                }
            }
            if( firstRead )
            {
                tmp.y = i;
            }
            else
            {
                tmp.y = i + 1;
            }
            firstRead = false;
            motionEstList.push_back( tmp );
            motionCompList.push_back( tmp );
            showandSaveList.push_back( tmp );

            semMotionEst->release();

            if( readFinished )
            {
                break;
            }
        }

        //file<<"read over"<<endl;
    }
};

//视频稳定运动估计线程
class ThreadMotionEstimation : public QThread
{
public:
    Size videoSize;

    void run()
    {
        while(true)
        {
            semMotionEst->acquire();

            if( motionEstList[0].y < SEGSIZE )
            {
                semMotionComp->release();
                break;
            }

            constructTrajectory();

            motionEstList.erase( motionEstList.begin() );

            semMotionComp->release();


        }
        //file<<"motionestimation over"<<endl;
    }

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
    void constructTrajectory()
    {
        int start = motionEstList[0].x;
        int length = motionEstList[0].y;

        list<vector<Point2f> > features;
        Mat preGray , curGray;
        vector<Point2f> preFeats;
        vector<Point2f> curFeats;
        vector<uchar> status;
        vector<float> err;
        cvtColor( frameVec[start] , preGray , CV_RGB2GRAY );//转化为灰度图
        goodFeaturesToTrack( preGray , preFeats , 200 , 0.1 , MIN_DISTANCE );//检测特征点
        int preFeatsNum = preFeats.size();
        for( int i = 0 ; i < preFeatsNum ; i++ )
        {
            vector<Point2f> tmp;
            tmp.push_back( preFeats[i] );
            features.push_back( tmp );
        }

        for( int i = 1 ; i < length ; i++ )
        {
            int index = ( start + i ) % ( BUFFERSIZE );

            curFeats.clear();
            status.clear();
            err.clear();
            cvtColor( frameVec[index] , curGray , CV_RGB2GRAY );
            calcOpticalFlowPyrLK( preGray , curGray , preFeats , curFeats , status , err );//根据已检测到的前一帧特征点在后一帧查找匹配的特征点

            /*消除特征点误匹配*/
            vector<Point2f> prefeatures = preFeats;
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

            list<vector<Point2f> >::iterator Iter;
            int j = 0;
            preFeats.clear();
            for( Iter = features.begin() ; Iter != features.end() ; j++ )
            {
                if(status[j] == 0 || err[j] > 20 || outOfImg(curFeats[j], videoSize))
                {
                    Iter = features.erase( Iter );
                }
                else
                {
                    preFeats.push_back( curFeats[j] );
                    (*Iter).push_back( curFeats[j] );
                    Iter++;
                }
            }

            if( features.size() == 0 )
            {
                break;
            }

            preGray = curGray.clone();

        }

        trj.push_back( features );

    }
};

//视频稳定运动补偿线程
class ThreadMotionCompensation : public QThread
{
public:
    Size videoSize;
    float clipRation = 0.8;
    bool clipControlFlag = false;

    void run()
    {
        while(true)
        {
            semMotionComp->acquire();

            if( motionCompList[0].y < SEGSIZE )
            {
                semShowandSave->release();
                break;
            }

            frameCompensate();

            motionCompList.erase( motionCompList.begin() );

            semShowandSave->release();

        }
        //file<<"motioncompensation over"<<endl;
    }

    void computeAffine( vector<Point2f> &avgFeatPos , vector<Mat> &affineMatrix )
    {
        list<list<vector<Point2f> > >::iterator Iter = trj.begin();
        if( (*Iter).size() < 3 )//计算仿射矩阵至少需要三个特征点对
            return;

        int trjNum = (*Iter).size();
        int trjLength = (*(*Iter).begin()).size();
        for( int i = 0 ; i < trjLength ; i++ )
        {
            Point2f sum(0,0);
            list<vector<Point2f> >::iterator Iter_2;
            for( Iter_2 = (*Iter).begin() ; Iter_2 != (*Iter).end() ; Iter_2++ )
            {
                sum += (*Iter_2)[i];
            }
            Point2f tmp;
            tmp.x = sum.x / trjNum;
            tmp.y = sum.y / trjNum;
            avgFeatPos.push_back(tmp);
        }

        //轨迹坐标中心化，转置
        vector<vector<Point2f> > normalTrj_T;
        for( int i = 0 ; i < trjLength ; i++ )
        {
            vector<Point2f> tmp;
            list<vector<Point2f> >::iterator Iter_2;
            for( Iter_2 = (*Iter).begin() ; Iter_2 != (*Iter).end() ; Iter_2++ )
            {
                (*Iter_2)[i] -= avgFeatPos[i];
                tmp.push_back( (*Iter_2)[i] );
            }
            normalTrj_T.push_back( tmp );
        }

        Mat affine;
        for( int i = 1 ; i < avgFeatPos.size() - 1 ; i++ )
        {
            affine = estimateRigidTransform( normalTrj_T[i] , normalTrj_T[0] , false );
            affineMatrix.push_back( affine );
        }
        affine = estimateRigidTransform( normalTrj_T[0] , normalTrj_T[avgFeatPos.size()-1] , false );
        affineMatrix.push_back( affine );
    }

    void frameCompensate()
    {
        int start = motionCompList[0].x;
        int length = motionCompList[0].y;

        /*稳定视频帧*/
        vector<Point2f> avgFeatPos;//存储段内每帧的几何中心坐标
        vector<Mat> affineMatrix;//存储帧与帧之间的仿射变换

        /*为旋转插值准备数据，即计算仿射矩阵，计算本段非关键帧到前关键帧的仿射矩阵与前关键帧到后关键帧的仿射矩阵*/
        computeAffine( avgFeatPos , affineMatrix );

        int x_tip = videoSize.width * ( 1 - clipRation ) / 2;
        int y_tip = videoSize.height * ( 1 - clipRation ) / 2;
        if( clipControlFlag )
        {
            //cout << "y  " << y_tip << "   x   " << x_tip << endl;
            //cout << y_tip << "   " << videoSize.height << "    " << x_tip << "    " << videoSize.width << endl;
            Mat out = frameVec[start](Range(y_tip , videoSize.height - y_tip), Range(x_tip , videoSize.width - x_tip));
            stableVec.push_back(out);
        }
        else
        {
            stableVec.push_back(frameVec[start]);
        }

        //通过奇异值分解得到前关键帧到后关键帧的旋转矩阵，并通过线性插值计算前关键帧到稳定位置的旋转量
        double s2e_theta;
        if( affineMatrix.size() != 0  )
        {
            if( !affineMatrix[affineMatrix.size()-1].empty() )
            {
                Mat s2eAffine, W, U, VT, R;
                s2eAffine = affineMatrix[affineMatrix.size()-1].colRange(0,2);
                SVD thissvd;
                thissvd.compute(s2eAffine,W,U,VT,SVD::FULL_UV);
                R = U*VT;
                s2e_theta = asinf( R.at<double>(0,1) );
            }
        }

        Mat tmpimg;
        for( int m = 0 ; m < length - 2 ; m++ )//计算段内非关键帧的运动补偿
        {
            int index = ( start + m + 1 ) % ( BUFFERSIZE );
            tmpimg = frameVec[index].clone();
            Point2f shift;//平移补偿
            Mat affine;//旋转补偿
            if( affineMatrix.size() == 0 )//即在此段内横跨整段的轨迹条数小于3
            {
                shift = Point2f(0.0,0.0);
                affine = Mat::zeros(2,3,CV_64FC1);
                affine.at<double>(0,0) = 1;
                affine.at<double>(1,1) = 1;
            }
            else
            {
                //平移补偿计算
                Point2f tmp;
                tmp = avgFeatPos[avgFeatPos.size()-1] - avgFeatPos[0];
                tmp.x = ( m + 1 ) * tmp.x / (float)( avgFeatPos.size() - 1 );
                tmp.y = ( m + 1 ) * tmp.y / (float)( avgFeatPos.size() - 1 );
                shift = avgFeatPos[0] - avgFeatPos[m+1] + tmp;

                //旋转补偿计算
                double degree;
                if( affineMatrix[affineMatrix.size()-1].empty() || affineMatrix[m].empty() )
                {
                    /*affine = Mat::zeros(2,3,CV_64FC1);
                    affine.at<double>(0,0) = 1;
                    affine.at<double>(1,1) = 1;*/
                    degree = 0;
                }
                else
                {
                    double delta_theta;
                    delta_theta = ( m + 1 ) * s2e_theta / (double)( avgFeatPos.size() - 1 );

                    //通过奇异值分解得到非关键帧到前关键帧的旋转矩阵
                    double theta;
                    SVD thissvd;
                    Mat i2sAffine , W2 , U2 , VT2;
                    i2sAffine = affineMatrix[m].colRange(0,2);
                    thissvd.compute(i2sAffine,W2,U2,VT2,SVD::FULL_UV);//计算旋转矩阵
                    i2sAffine = U2*VT2;
                    theta = asinf( i2sAffine.at<double>(0,1) );

                    //计算旋转补偿量
                    //double degree;//角度为正表示逆时针旋转
                    degree = theta + delta_theta;
                    //degree = degree * 180 / 3.1415926;
                    //affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
                }

                if( clipControlFlag )
                {
                    clipControl( clipRation , avgFeatPos[m+1] , shift , degree );
                    degree = degree * 180 / 3.1415926;
                    affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
                }
                else
                {
                    degree = degree * 180 / 3.1415926;
                    affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
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
            if( clipControlFlag )
            {
                Mat newOut = out(Range(y_tip , videoSize.height - y_tip), Range(x_tip , videoSize.width - x_tip));
                stableVec.push_back(newOut);
            }
            else
                stableVec.push_back(out);
        }

        trj.pop_front();
    }

    //经过平移旋转后仍包括裁剪窗口返回true，否则返回false，且修改shift和degree值
    bool clipControl( float clipRation , Point2f center , Point2f &shift , double &degree )
    {
        vector<Point2f> pt;//原始图像顶点坐标
        pt.push_back( Point2f(0,0) );
        pt.push_back( Point2f(0,videoSize.height-1) );
        pt.push_back( Point2f(videoSize.width-1,videoSize.height-1) );
        pt.push_back( Point2f(videoSize.width-1,0) );
        vector<Point2f> pt_clip;//裁剪窗口顶点坐标
        int x_tip = videoSize.width * ( 1 - clipRation ) / 2;
        int y_tip = videoSize.height * ( 1 - clipRation ) / 2;
        pt_clip.push_back( Point2f(x_tip,y_tip) );
        pt_clip.push_back( Point2f(x_tip,videoSize.height-y_tip) );
        pt_clip.push_back( Point2f(videoSize.width-x_tip,videoSize.height-y_tip) );
        pt_clip.push_back( Point2f(videoSize.width-x_tip,y_tip) );


        Mat shiftMat = Mat::eye(3,3,CV_64FC1);
        shiftMat.at<double>(0,2) = shift.x;
        shiftMat.at<double>(1,2) = shift.y;
        double angle = degree * 180 / 3.1415926;
        Mat tmpRotateMat = getRotationMatrix2D( center+shift , angle , 1 );
        Mat rotateMat = Mat::zeros(3,3,CV_64FC1);
        rotateMat.at<double>(0,0) = tmpRotateMat.at<double>(0,0);
        rotateMat.at<double>(0,1) = tmpRotateMat.at<double>(0,1);
        rotateMat.at<double>(0,2) = tmpRotateMat.at<double>(0,2);
        rotateMat.at<double>(1,0) = tmpRotateMat.at<double>(1,0);
        rotateMat.at<double>(1,1) = tmpRotateMat.at<double>(1,1);
        rotateMat.at<double>(1,2) = tmpRotateMat.at<double>(1,2);
        rotateMat.at<double>(2,2) = 1;
        Mat affine = rotateMat * shiftMat;
        affine = affine.rowRange(0,2);

        if( isInsideAfterTransform( affine , pt_clip , pt ) )//经过平移旋转后依然包括裁剪窗口
        {
            return true;
        }
        else
        {
            if( x_tip < abs(shift.x) || y_tip < abs(shift.y) )//经过平移后不包括裁剪窗口
            {
                float ratio1 = (float)x_tip / abs(shift.x);
                float ratio2 = (float)y_tip / abs(shift.y);
                if( ratio1 < ratio2 )
                {
                    if( shift.x > 0 )
                    {
                        shift.x = x_tip;
                    }
                    else
                    {
                        shift.x = -x_tip;
                    }
                    shift.y = ratio1 * shift.y;
                }
                else
                {
                    shift.x = ratio2 * shift.x;
                    if( shift.y > 0 )
                    {
                        shift.y = y_tip;
                    }
                    else
                    {
                        shift.y = -y_tip;
                    }
                }
                degree = 0;
            }
            else//经过平移后包括裁剪窗口
            {
                /*计算最大旋转角*/
                vector<Point2f> new_clip_pt;
                for( int i = 0 ; i < 4 ; i++ )
                {
                    new_clip_pt.push_back(pt_clip[i]-shift);
                }

                double maxDegree[4];
                vector<Point2f> img_line;
                vector<Point2f> clip_line;
                img_line.push_back( Point2f(0,0) );
                img_line.push_back( Point2f(videoSize.width-1,0) );
                clip_line.push_back( new_clip_pt[0] );
                clip_line.push_back( new_clip_pt[3] );
                maxDegree[0] = computeMaxDegree( img_line , clip_line , degree , center );

                img_line[1] = Point2f(videoSize.height-1,0);
                clip_line[0] = Point2f(videoSize.height-1-new_clip_pt[1].y , new_clip_pt[1].x);
                clip_line[1] = Point2f(videoSize.height-1-new_clip_pt[0].y , new_clip_pt[0].x);
                Point2f newCenter;
                newCenter.x = videoSize.height-1-center.y;
                newCenter.y = center.x;
                maxDegree[1] = computeMaxDegree( img_line , clip_line , degree , newCenter );

                img_line[1] = Point2f(videoSize.width-1,0);
                clip_line[0] = Point2f(videoSize.width-1-new_clip_pt[2].x , videoSize.height-1-new_clip_pt[2].y);
                clip_line[1] = Point2f(videoSize.width-1-new_clip_pt[1].x , videoSize.height-1-new_clip_pt[1].y);
                newCenter.x = videoSize.width-1-center.x;
                newCenter.y = videoSize.height-1-center.y;
                maxDegree[2] = computeMaxDegree( img_line , clip_line , degree , newCenter );

                img_line[1] = Point2f(videoSize.height-1,0);
                clip_line[0] = Point2f(new_clip_pt[3].y , videoSize.width-1-new_clip_pt[3].x);
                clip_line[1] = Point2f(new_clip_pt[2].y , videoSize.width-1-new_clip_pt[2].x);
                newCenter.x = center.y;
                newCenter.y = videoSize.width-1-center.x;
                maxDegree[3] = computeMaxDegree( img_line , clip_line , degree , newCenter );

                if( degree > 0 )
                {
                    double min = degree;
                    for( int i = 0 ; i < 4 ; i++ )
                    {
                        if( min > maxDegree[i] )
                        {
                            min = maxDegree[i];
                        }
                    }
                    degree = min;
                }
                else
                {
                    double max = degree;
                    for( int i = 0 ; i < 4 ; i++ )
                    {
                        if( max < maxDegree[i] )
                        {
                            max = maxDegree[i];
                        }
                    }
                    degree = max;
                }
                /**/
            }

            return false;
        }
    }

    //判断经过变换后是否依然包括裁剪窗口
    bool isInsideAfterTransform( Mat &affine , vector<Point2f> &pt_clip , vector<Point2f> &pt )
    {
        vector<Point2f> pt_transform;//变换后的坐标
        for( int i = 0 ; i < 4 ; i++ )
        {
            Mat tmp = Mat::ones(3,1,CV_64FC1);
            tmp.at<double>(0,0) = pt[i].x;
            tmp.at<double>(1,0) = pt[i].y;
            Mat pos = affine * tmp;
            Point2f p;
            p.x = pos.at<double>(0,0);
            p.y = pos.at<double>(1,0);
            pt_transform.push_back(p);
        }
        //判断裁剪窗口顶点坐标是否都在变换后的坐标组成的矩形内
        bool allInside = true;
        for( int i = 0 ; i < 4 ; i++ )
        {
            for( int j = 0 ; j < 4 ; j++ )
            {
                Point2f vec1 , vec2;
                vec1 = pt_transform[j] - pt_clip[i];
                vec2 = pt_transform[(j+1)%4] - pt_transform[j];
                float cross_product = vec1.x * vec2.y - vec2.x * vec1.y;
                if( cross_product > 0 )
                {
                    allInside = false;
                    break;
                }
            }
            if( !allInside )
            {
                break;
            }
        }

        return allInside;
    }

    //计算最大旋转角
    //img_line：起点为原点，与X轴重合，长度为图像长或宽
    //clip_line：裁剪线
    //degree：旋转角度，正值为逆时针旋转
    //center：旋转中心
    double computeMaxDegree( vector<Point2f> img_line , vector<Point2f> clip_line , double degree , Point2f center )
    {
        if( degree > 0 )
        {
            if( center.x <= clip_line[0].x )
            {
                return 3.1415926;
            }
            else
            {
                float dis = sqrt( pow(clip_line[0].x - center.x,2) + pow(clip_line[0].y - center.y,2) );
                if( dis <= center.y )
                {
                    return 3.1415926;
                }
                else
                {
                    /*计算切点*/
                    float a1 , a2 , a3;
                    a1 = center.x - clip_line[0].x;
                    a2 = center.y - clip_line[0].y;
                    a3 = center.x*clip_line[0].x - center.x*center.x + center.y*clip_line[0].y;
                    float k , n;
                    k = -a2 / a1;
                    n = -a3 / a1;
                    float a , b , c;
                    a = k*k + 1;
                    b = 2*k*n - 2*center.x*k - 2*center.y;
                    c = n*n - 2*center.x*n + center.x*center.x;
                    Point2f pointofContact;
                    float y1 , y2;
                    y1 = (-b + sqrt(b*b - 4*a*c)) / (2*a);
                    y2 = (-b - sqrt(b*b - 4*a*c)) / (2*a);
                    pointofContact.y = (y1<y2)?y1:y2;
                    pointofContact.x = k*pointofContact.y + n;
                    /**/

                    Point2f vec1 , vec2;
                    vec1 = pointofContact - clip_line[0];
                    vec2 = clip_line[1] - clip_line[0];
                    float cos_alpha = (vec1.x*vec2.x + vec1.y*vec2.y) / (sqrt(vec1.x*vec1.x+vec1.y*vec1.y) * sqrt(vec2.x*vec2.x+vec2.y*vec2.y));
                    double alpha = acos(cos_alpha);

                    return alpha;
                }
            }
        }
        else
        {
            if( center.x >= clip_line[1].x )
            {
                return -3.1415926;
            }
            else
            {
                float dis = sqrt( pow(clip_line[1].x - center.x,2) + pow(clip_line[1].y - center.y,2) );
                if( dis <= center.y )
                {
                    return -3.1415926;
                }
                else
                {
                    /*计算切点*/
                    float a1 , a2 , a3;
                    a1 = center.x - clip_line[1].x;
                    a2 = center.y - clip_line[1].y;
                    a3 = center.x*clip_line[1].x - center.x*center.x + center.y*clip_line[1].y;
                    float k , n;
                    k = -a2 / a1;
                    n = -a3 / a1;
                    float a , b , c;
                    a = k*k + 1;
                    b = 2*k*n - 2*center.x*k - 2*center.y;
                    c = n*n - 2*center.x*n + center.x*center.x;
                    Point2f pointofContact;
                    float y1 , y2;
                    y1 = (-b + sqrt(b*b - 4*a*c)) / (2*a);
                    y2 = (-b - sqrt(b*b - 4*a*c)) / (2*a);
                    pointofContact.y = (y1<y2)?y1:y2;
                    pointofContact.x = k*pointofContact.y + n;
                    /**/

                    Point2f vec1 , vec2;
                    vec1 = pointofContact - clip_line[1];
                    vec2 = clip_line[0] - clip_line[1];
                    float cos_alpha = (vec1.x*vec2.x + vec1.y*vec2.y) / (sqrt(vec1.x*vec1.x+vec1.y*vec1.y) * sqrt(vec2.x*vec2.x+vec2.y*vec2.y));
                    double alpha = acos(cos_alpha);

                    return -alpha;
                }
            }
        }
    }
};

#endif
