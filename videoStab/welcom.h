#ifndef WELCOM_H
#define WELCOM_H

#include <QWidget>
#include "widget.h"
#include "about.h"

namespace Ui {
class Welcom;
}

class Welcom : public QWidget
{
    Q_OBJECT

public:
    explicit Welcom(QWidget *parent = 0);
    ~Welcom();

private:
    Ui::Welcom *ui;

private slots:
    void enterMain();
    void showAbout();
    void exitProgram();

};

#endif // WELCOM_H
