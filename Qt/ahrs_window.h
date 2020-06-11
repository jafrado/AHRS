#ifndef _AHRS_WINDOW_H
#define _AHRS_WINDOW_H
#include "AHRS.h"
#include "ui_ahrs_window.h"

class AHRSWindow : public QMainWindow
{
    Q_OBJECT
        friend class AHRSReceiver;
public:
    explicit AHRSWindow(QWidget* parent = 0);
    ~AHRSWindow();
    void keyPressEvent(QKeyEvent* event) { return hud->keyPressEvent(event); }
    void closeEvent(QCloseEvent* event);

public slots:
    void connectINS();
    void disconnectINS();

private:
    Ui::AHRSWindow* ui;
    AHRSWidget* hud;
    AHRSReceiver* receiver;
};



#endif /* !_AHRS_WINDOW_H */
