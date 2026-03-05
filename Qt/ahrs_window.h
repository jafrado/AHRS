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
    void keyPressEvent(QKeyEvent* event) {
        if (event->key() == Qt::Key_F11) {
            if (isFullScreen()) {
                showNormal();
                menuBar()->show();
                ui->mainToolBar->show();
                ui->statusBar->show();
                hud->deviceLabel->show();
            } else {
                menuBar()->hide();
                ui->mainToolBar->hide();
                ui->statusBar->hide();
                hud->deviceLabel->hide();
                showFullScreen();
            }
        } else {
            hud->keyPressEvent(event);
        }
    }
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
