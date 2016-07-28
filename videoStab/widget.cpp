#include "widget.h"
#include "ui_widget.h"
#include <QTime>
#include <sys/time.h>
#include <unistd.h>

Mat frameVec[SEGSIZE*4];//原始帧容器
vector<Mat> stableVec;//稳定帧容器
QSemaphore *semReadBuffer , *semMotionEst , *semMotionComp , *semShowandSave;
list<list<vector<Point2f> > > trj;//存储特征点轨迹
vector<Point2i> motionEstList;//存储每次运动估计时用到的帧在frameVec中的位置，x表示起点，y表示长度
vector<Point2i> motionCompList;//存储每次运动补偿时用到的帧在frameVec中的位置，x表示起点，y表示长度
vector<Point2i> showandSaveList;//存储每次显示和保存时用到的帧在frameVec中的位置，x表示起点，y表示长度

Widget::Widget(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    this->resize(QSize(windowWidth, windowHeight));

    menuCreate();

    this->setWindowFlags(Qt::FramelessWindowHint);

    /// TODO: Init all the layouts
    /// Blind the param and the ui layouts
    btnOpenRaw = ui->btnOpenVideoRaw;
    btnCtrlStb = ui->btnCtrlStab;
    btnOpenGen = ui->btnOpenVideoStab;
    /// TODO: Set the layouts' attributes
    /// Set the init position
    initLayouts();
    /// Set the buttons' init state
    btnOpenGen->setEnabled(false);
    btnCtrlStb->setEnabled(false);
    btnOpenRaw->setEnabled(true);
    /// TODO: Realize the layouts' function
    /// Use connect to blind
    connect(btnOpenRaw, SIGNAL(clicked()), this, SLOT(openVideoRaw()));
    connect(btnOpenGen, SIGNAL(clicked()), this, SLOT(openVideoGenerate()));
    connect(btnCtrlStb, SIGNAL(clicked()), this, SLOT(controlStable()));
    connect(ui->btnExit,SIGNAL(clicked()), this, SLOT(exitProgram()));

    /// Set the stroage video path
    pathVideoGen = "/tmp/stabelVideo.avi";
    file.open("path.txt", ios::out);

    imgSizeMin.width = imgLayoutWidth;
    imgSizeMin.height = imgLayoutHeight;

    imgSizeMax.width = imgShowWidth;
    imgSizeMax.height = imgShowHeight;

    ui->checkCutEdge->setVisible(false);
    ui->sliderCutMannualy->setVisible(false);
    ui->labelCutEdgeShow->setVisible(false);
    ui->lineEditCammer->setVisible(false);
    ui->btnStop->setVisible(false);
    ui->btnStop->setEnabled(false);

    procImgShow = imread("../videoStab/imgs/processingEN.png");
}

Widget::~Widget()
{
    delete ui;
}

void Widget::openVideoRaw()
{
    ui->imgLabelShow->clear();
    ui->imgLabelRaw->clear();
    ui->imgLabelGen->clear();
    ui->txtlabelShow->clear();
    // get the raw video's path
    pathVideoRaw = QFileDialog::getOpenFileName(
                this,
                tr("Choose a video file"),
                tr("/home/odroid/ws/samples/movs/"),
                tr("Videos(*.mov *.avi *.mp4)")
                );

    if(pathVideoRaw.length() > 0)
    {
        btnOpenRaw->setText("Fast scanning...");

        QMessageBox::information(this, tr("Notify"),
                                 tr("Open video successfully!"),
                                 QMessageBox::Yes);

        // Set the notify label information
        ui->txtLabelRaw->setText("Original video");
        ui->txtLabelGen->setText("Processing video");
        ui->txtlabelShow->clear();
        // Reset the layouts show
        btnCtrlStb->setEnabled(true);
        isVideoGen = false;
        btnCtrlStb->setText("Start Process");

        btnOpenRaw->setText("Open Files");

        ui->checkCutEdge->setChecked(true);
        setCutEdgeInfo(true);
    }
    else
    {
        QMessageBox::warning(this, tr("Notify"),
                             tr("Open video failed!\n Just try again "),
                             QMessageBox::Yes);
    }
}

void Widget::controlStable()
{
    ui->btnStop->setEnabled(true);
    ui->btnStop->setVisible(true);
    btnCtrlStb->setEnabled(false);
    setCutEdgeInfo(false);

    int retStatus = videoStab();

    if(retStatus != -1)
    {
        QMessageBox::information(this, tr("Notify"),
                                 tr("The stable video has storaged in \n %1").arg(
                                     pathVideoGen.toStdString().c_str()), 1,
                                 QMessageBox::Yes);
        ui->btnExit->setEnabled(true);
        btnOpenGen->setEnabled(true);
    }
    else if(retStatus == -1)
    {
        btnCtrlStb->setEnabled(true);
    }
    ui->btnStop->setEnabled(false);
    ui->btnStop->setVisible(false);
}

void Widget::initLayouts()
{
    int midWidth = windowWidth / 2;
    int offset = windowWidth / 3.5;
    int offsetMin = 3;
    btnOpenRaw->setGeometry(midWidth - offset - iconWidth / 2,
                            windowHeight - iconHeight - offsetMin,
                            iconWidth, iconHeight);
    btnCtrlStb->setGeometry(midWidth - iconWidth / 2,
                            windowHeight - iconHeight - offsetMin,
                            iconWidth, iconHeight);
    btnOpenGen->setGeometry(midWidth + offset - iconWidth / 2,
                            windowHeight - iconHeight - offsetMin,
                            iconWidth, iconHeight);
    ui->btnExit->setGeometry(windowWidth - 80, 20, 55, 35);
    // Original video show window
    ui->imgLabelRaw->setGeometry(windowWidth / 2 - imgLayoutWidth - 5,
                                 imgLayoutY, imgLayoutWidth, imgLayoutHeight);
    ui->txtLabelRaw->setGeometry(windowWidth / 2 - imgLayoutWidth / 2 - 5 - txtLayoutY,
                                 imgLayoutY + imgLayoutHeight + 10, 200, 20);
    // Processing video show window
    ui->imgLabelGen->setGeometry(windowWidth / 2 + 5,
                                 imgLayoutY, imgLayoutWidth, imgLayoutHeight);
    ui->txtLabelGen->setGeometry(windowWidth / 2 + imgLayoutWidth / 2  + 5 - txtLayoutY,
                                 imgLayoutY + imgLayoutHeight + 10, 200, 20);
    // Ulitimate video show window
    ui->imgLabelShow->setGeometry(windowWidth / 2 - imgShowWidth / 2,
                                  50, imgShowWidth, imgShowHeight);
    ui->txtlabelShow->setGeometry(windowWidth / 2 - 5 - txtLayoutY,
                                  20, 200, 20);
}

void Widget::openVideoGenerate()
{
    ui->btnStop->setEnabled(true);
    ui->btnStop->setVisible(true);
    btnOpenGen->setText("on Showing");
    btnCtrlStb->setEnabled(false);

    ui->checkUseCam->setVisible(false);
    ui->imgLabelGen->clear();
    ui->imgLabelRaw->clear();
    ui->txtLabelGen->clear();
    ui->txtLabelRaw->clear();
    ui->txtlabelShow->setText("Generated video");

    showVideo(pathVideoGen.toStdString(), ui->imgLabelShow);

    btnOpenRaw->setEnabled(true);
    btnCtrlStb->setEnabled(true);
    //btnOpenGen->setEnabled(false);
    btnOpenGen->setText("Open Generated");
    isVideoGen = true;
    ui->checkUseCam->setVisible(true);
    ui->btnStop->setEnabled(false);
    ui->btnStop->setVisible(false);
    btnCtrlStb->setEnabled(true);
}

void Widget::displayMat(Mat image, QLabel* imageLabel)
{
    Mat rgb;
    QImage img;
    if(image.channels()== 3)
    {
        //cvt Mat BGR 2 QImage RGB
        cvtColor(image,rgb,CV_BGR2RGB);
        img =QImage((const unsigned char*)(rgb.data),
                    rgb.cols,rgb.rows,
                    rgb.cols*rgb.channels(),
                    QImage::Format_RGB888);
    }
    else
    {
        img =QImage((const unsigned char*)(image.data),
                    image.cols,image.rows,
                    image.cols*image.channels(),
                    QImage::Format_RGB888);
    }

    imageLabel->setPixmap(QPixmap::fromImage(img));
}

void Widget::exitProgram()
{
    this->close();
    exit(0);
}

void Widget::showVideo(string path, QLabel* imageLabel)
{
    VideoCapture cap(path);
    statGenRunning = true;
    double fps = cap.get(CV_CAP_PROP_FPS);
    fps = fps > 0 ? fps : 30.0;
    cap.set(CV_CAP_PROP_FPS, 30);
    while(cap.isOpened() && statGenRunning) {
        Mat frame;
        cap >> frame;
        if(!frame.data)
            return ;
        Mat showFrame;
        cv::resize(frame, showFrame, imgSizeMax);

        displayMat(showFrame, imageLabel);
        waitKey(25);
    }
    ui->imgLabelShow->clear();
    statGenRunning = false;
}

void Widget::menuCreateAction()
{

}

void Widget::menuCreate()
{
    menu = new QMenu(this);
    menu->addMenu(tr("File"));
    menu->addMenu(tr("Edit"));
    menu->addMenu(tr("About"));
}

void Widget::setCutEdgeInfo(bool status)
{
    ui->checkCutEdge->setEnabled(status);
    ui->checkCutEdge->setVisible(status);
    if(!status)
    {
        ui->sliderCutMannualy->setEnabled(status);
        ui->sliderCutMannualy->setVisible(status);
        ui->labelCutEdgeShow->setVisible(status);
    }
}

//===================================================================

int Widget::videoStab()
{
    /*初始化*/
    semReadBuffer = new QSemaphore(4);
    semMotionEst = new QSemaphore(0);
    semMotionComp = new QSemaphore(0);
    semShowandSave = new QSemaphore(0);
    trj.clear();
    motionEstList.clear();
    motionCompList.clear();
    showandSaveList.clear();
    stableVec.clear();
    threadRead.init();
    /**/

    VideoCapture videoCap;

    if(ui->checkUseCam->checkState())
    {
        int camNum = ui->lineEditCammer->text().toInt();
        videoCap.open(camNum);
        cout << camNum << endl;
        if(!videoCap.isOpened())
        {
            QMessageBox::warning(this, tr("Error"),
                                 tr("The camera port %1 is not valid").arg(camNum),
                                 QMessageBox::Yes);
            return -1;
        }
    }
    else
    {
        videoCap.open( pathVideoRaw.toStdString() );//使用opencv封装的类VideoCapture打开视频
    }

    /*获取视频属性*/
    Size videoSize;
    videoSize.height = videoCap.get(CV_CAP_PROP_FRAME_HEIGHT);//视频分辨率高度
    videoSize.width = videoCap.get(CV_CAP_PROP_FRAME_WIDTH);//视频分辨率宽度
    int fourcc = videoCap.get(CV_CAP_PROP_FOURCC);//视频编码方式
    double fps = videoCap.get(CV_CAP_PROP_FPS);//视频帧率

    fps = fps > 0 ? fps : 30.0;

    double gap = 1000.0 / fps;

    threadRead.videoCap = videoCap;
    threadMotionEstimation.videoSize = videoSize;
    threadMotionCompensation.videoSize = videoSize;
    threadMotionCompensation.clipControlFlag = true;

    float cutRate = 0.8;
    if(ui->checkCutEdge->checkState() == false)
    {
        cutRate = 1.0 - (float)ui->sliderCutMannualy->value() / 100.0;
    }
    threadMotionCompensation.clipRation = cutRate;

    Size vw_videoSize;
    if(threadMotionCompensation.clipControlFlag)
    {
        int x_tip = videoSize.width*(1-threadMotionCompensation.clipRation)/2;
        int y_tip =videoSize.height*(1-threadMotionCompensation.clipRation)/2;
        Size videoSize_clip;
        videoSize_clip.width = videoSize.width-2*x_tip;
        videoSize_clip.height = videoSize.height-2*y_tip;
        vw_videoSize = videoSize_clip;
    }
    else
    {
        vw_videoSize = videoSize;
    }

    VideoWriter vw = VideoWriter(pathVideoGen.toStdString(), CV_FOURCC('M', 'J', 'P', 'G'), fps, vw_videoSize);

    threadRead.start();
    threadMotionEstimation.start();
    threadMotionCompensation.start();

    QTime t;
    bool first = true;
    namedWindow(procImgTitle);
    while(true)
    {
        semShowandSave->acquire();

        if( first )
        {
            t.start();
            first = false;
        }

        int start = showandSaveList[0].x;
        int length = showandSaveList[0].y;

        if( length < SEGSIZE )
        {
            break;
        }

        for( int i = 0 ; i < length - 1 ; i++ )
        {
            //=========================================
            genFrame = stableVec[i].clone();
            //cv::resize(frameVec[i], genFrame, imgSizeMin);
            cv::resize(genFrame, genFrame, Size(imgLayoutWidth, imgLayoutHeight));
            displayMat(genFrame, ui->imgLabelGen);
            //=========================================
            //==================================================
            oriFrame = frameVec[start].clone();
            start ++;
            if(start >= BUFFERSIZE)
            {
                start = 0;
            }
            //oriFrame.resize();
            cv::resize(oriFrame, oriFrame, imgSizeMin);
            displayMat(oriFrame, ui->imgLabelRaw);

            imshow(procImgTitle, procImgShow);
            if(ui->checkUseCam->isChecked())
                waitKey(10);
            waitKey(5);
            //==================================================
            vw << stableVec[i];
            while( t.elapsed() < gap )
            {
                QCoreApplication::processEvents();
            }
            t.start();
        }

        showandSaveList.erase( showandSaveList.begin() );
        if( stableVec.size() >= SEGSIZE )
        {
            stableVec.erase( stableVec.begin() , stableVec.begin() + length - 1 );
        }

        semReadBuffer->release();

    }
    destroyWindow(procImgTitle);
    vw.release();

    threadRead.wait();
    threadMotionEstimation.wait();
    threadMotionCompensation.wait();
}

void Widget::on_checkCutEdge_clicked()
{
    if(checkCutState)
    {
        ui->sliderCutMannualy->setEnabled(false);
        ui->sliderCutMannualy->setVisible(false);
        ui->labelCutEdgeShow->setVisible(false);
    }
    else
    {
        ui->sliderCutMannualy->setEnabled(true);
        ui->sliderCutMannualy->setVisible(true);
        ui->labelCutEdgeShow->setVisible(true);
    }
    checkCutState = !checkCutState;
}

void Widget::on_checkUseCam_clicked()
{
    if(checkCammeraStatus)
    {
        if(!ui->checkCutEdge->isChecked())
        {
            ui->sliderCutMannualy->setEnabled(true);
            ui->sliderCutMannualy->setVisible(true);
            ui->labelCutEdgeShow->setVisible(true);
        }
        ui->lineEditCammer->setVisible(true);
        ui->checkCutEdge->setVisible(true);
        btnOpenRaw->setEnabled(false);
        setCutEdgeInfo(true);
        btnCtrlStb->setEnabled(true);
    }
    else
    {
        if(!ui->checkCutEdge->isChecked())
        {
            ui->sliderCutMannualy->setEnabled(false);
            ui->sliderCutMannualy->setVisible(false);
            ui->labelCutEdgeShow->setVisible(false);
        }
        ui->lineEditCammer->setVisible(false);
        ui->checkCutEdge->setVisible(false);
        btnOpenRaw->setEnabled(true);
    }
    checkCammeraStatus = !checkCammeraStatus;
}

void Widget::on_btnStop_clicked()
{
    if(statGenRunning)
    {
        statGenRunning = false;
    }
    else
    {
        threadRead.stop();
    }
}
