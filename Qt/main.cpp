#include <QApplication>
#include "AHRS.h"
#include "ahrs_window.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    AHRSWindow* w = new AHRSWindow();
    w->show();

    return a.exec();
}
