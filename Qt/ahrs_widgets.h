#ifndef __AHRS_WIDGETS_H
#define __AHRS_WIDGETS_H

/***
 * Widgets for AHRS Display
 * AHRSAttitudeIndicator derived from qAttitudeIndicator by cmex81/qAttitudeIndicator
 * cmex81/qAttitudeIndicator
 * https://github.com/cmex81/qAttitudeIndicator
 * 
 * AHRSAttitude Indicator - enhanced by bushuhui
 * AHRSCompass - derived from bushui/qFlightInstruments QCompass
 * bushuhui/qFlightInstruments
 * https://github.com/bushuhui/qFlightInstruments
 * 
 * AHRSInfo RTK GPS Indicator - by jafrado
 * 
 */

#include <QWidget>
#include <QLabel>
#include <QMainWindow>
#include <QBoxLayout>
#include <QVBoxLayout>
#include <QtCore>
#include <QtGui>
#include <QString>
#include <QThread>
#include <QByteArray>
#include <QLabel>
#include <QInputDialog>
#include <QMessageBox>

class AHRSAttitudeIndicator : public QWidget
{
    Q_OBJECT

public:
    AHRSAttitudeIndicator(QWidget* parent = 0);
    ~AHRSAttitudeIndicator();

    void setRoll(double val)
    {
        roll = val;
        if (roll < -180)
            roll = -180;
        if (roll > 180)
            roll = 180;
        emit widgetReplot();
    }

    void setPitch(double val)
    {
        pitch = val;
        if (pitch < -90)
            pitch = -90;
        if (pitch > 90)
            pitch = 90;
        emit widgetReplot();
    }

    double getRoll() { return roll; }
    double getPitch() { return pitch; }


signals:
    void widgetReplot(void);

protected slots:
    void widgetReplot_slot(void);

protected:
    void paintEvent(QPaintEvent* event);
    void resizeEvent(QResizeEvent* event);
    void keyPressEvent(QKeyEvent* event);

private:
    int size;
    int offset;
    double roll;
    double pitch;
};


class AHRSCompass : public QWidget
{
    Q_OBJECT

public:
    AHRSCompass(QWidget* parent = 0);
    ~AHRSCompass();

    void setYaw(double val)
    {
        yaw = val;
        if (yaw < 0)
            yaw = 360 + yaw;
        if (yaw > 360)
            yaw = yaw - 360;
        emit canvasReplot();
    }
    void setAlt(double val)
    {
        altitude = val;
        emit canvasReplot();
    }
    void setH(double val)
    {
        msl = val;
        emit canvasReplot();
    }
    double getYaw() { return yaw; }
    double getAlt() { return altitude; }
    double getH() { return msl; }

signals:
    void canvasReplot(void);

protected slots:
    void canvasReplot_slot(void);

protected:
    void paintEvent(QPaintEvent* event);
    void resizeEvent(QResizeEvent* event);
    void keyPressEvent(QKeyEvent* event);

protected:
    int sizeMin, sizeMax;
    int size, offset;
    double  yaw;
    double  altitude;
    double  msl;
};



class AHRSInfo : public QWidget
{
    Q_OBJECT

public:
    QLabel* latitude;
    QLabel* longitude;
    QLabel* pdop;
    QLabel* siv;
    QLabel* rtk;
    QLabel* fix;
    QLabel* hdg;
    QLabel* gspeed;

    QLabel* precision;
    QLabel* ecefX;
    QLabel* ecefY;
    QLabel* ecefZ;

    QLabel* ecefVX;
    QLabel* ecefVY;
    QLabel* ecefVZ;

    AHRSInfo(QWidget* parent = 0);
    ~AHRSInfo();
    void updatePosition(double lat, double lon);
    void updatePDOP(double pdop);
    void updateStatus(int siv, int fixType, int rtkType);
    
signals:
protected slots:


};

#endif /* !__AHRS_WIDGETS */
