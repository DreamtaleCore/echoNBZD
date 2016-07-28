#ifndef ABOUT_H
#define ABOUT_H

#include <QWidget>
#include "welcom.h"

namespace Ui {
class About;
}

class About : public QWidget
{
    Q_OBJECT

public:
    explicit About(QWidget *parent = 0);
    ~About();

private:
    Ui::About *ui;

private slots:
    void backToMain();
};

#endif // ABOUT_H
