#include <QApplication>
#include "emulator_window.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("AHRS INS Emulator");

    EmulatorWindow window;
    window.show();

    return app.exec();
}
