#include "about.h"
#include "ui_about.h"

About::About(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::About)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::X11BypassWindowManagerHint);
    this->resize(QSize(800, 480));
    connect(ui->btnAboutBack,SIGNAL(clicked()),this, SLOT(backToMain()));
}

About::~About()
{
    delete ui;
}

void About::backToMain()
{
    Welcom* wcm = new Welcom();
    this->close();
    wcm->show();
}
