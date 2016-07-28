#ifndef WIDGET_H
#define WIDGET_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QString>
#include "cv4stab.h"
#include <QFileDialog>
#include <QEvent>
#include <QHelpEvent>
#include <QToolTip>
#include <QPoint>
#include <QRect>
#include <QMenu>
#include <QMessageBox>
#include <QThread>
#include "myThread.h"


namespace Ui {
class Widget;
}

class Widget : public QMainWindow
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    void showVideo(string path, QLabel *imageLabel);
    void displayMat(Mat image, QLabel *imageLabel);

    /// \attention The code is based on Yang's
    //->
    int videoStab();
    /// \author thrid cooperation

    // Let's rock here!
    void menuCreateAction();
    void menuCreate();
    void setCutEdgeInfo(bool status);
    ~Widget();

private slots:
    void openVideoRaw();
    void openVideoGenerate();
    void controlStable();
    void initLayouts();
    void exitProgram();

    void on_checkCutEdge_clicked();

    void on_checkUseCam_clicked();

    void on_btnStop_clicked();

public:

private:
    QPushButton* btnOpenRaw;
    QPushButton* btnOpenGen;
    QPushButton* btnCtrlStb;
    Mat procImgShow;
    string procImgTitle = ".";

    ThreadRead threadRead;
    ThreadMotionEstimation threadMotionEstimation;
    ThreadMotionCompensation threadMotionCompensation;

    Ui::Widget *ui;
    QMenu* menu;

    const static int windowWidth = 800;
    const static int windowHeight = 480;
    const static int iconWidth = 120;
    const static int iconHeight = 40;
    const static int imgLayoutWidth = 370;
    const static int imgLayoutHeight = 250;
    const static int imgShowWidth = 710;
    const static int imgShowHeight = 380;
    const static int imgLayoutY = 80;
    const static int txtLayoutY = 45;

    Size imgSizeMin;
    Size imgSizeMax;

    QString pathVideoRaw;
    QString pathVideoGen;

    bool checkCutState = false;
    bool checkCammeraStatus = true;
    bool statGenRunning = false;

    string videoName;

    bool isVideoGen;

    Size videoSize;
    bool needShakeDetect = false;

    string stabWindow;
    Mat oriFrame, genFrame;

    ofstream file;
    VideoWriter vw;
};


#endif // WIDGET_H
