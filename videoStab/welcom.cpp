#include "welcom.h"
#include "ui_welcom.h"

Welcom::Welcom(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Welcom)
{
    ui->setupUi(this);
    this->resize(QSize(800, 480));

    this->setWindowFlags(Qt::X11BypassWindowManagerHint);

    connect(ui->btnWelcom, SIGNAL(clicked()), this, SLOT(enterMain()));
    connect(ui->btnAbout,SIGNAL(clicked()),this, SLOT(showAbout()));
    connect(ui->btnExit, SIGNAL(clicked()),this, SLOT(exitProgram()));
}

Welcom::~Welcom()
{
    delete ui;
}

void Welcom::enterMain()
{
    Widget* widget = new Widget();
    this->close();
    widget->show();
}

void Welcom::showAbout()
{
    About* about = new About();
    this->hide();
    about->show();
}

void Welcom::exitProgram()
{
    this->close();
}
