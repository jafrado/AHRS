#include "ahrs_window.h"

AHRSWindow::AHRSWindow(QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui::AHRSWindow),
    hud(nullptr),
    receiver(nullptr)
{
    ui->setupUi(this);
    //Setup AHRS HUD
    hud = new AHRSWidget(this);
    connect(ui->actionConnect, SIGNAL(triggered()), this, SLOT(connectINS()));
    connect(ui->actionClose, SIGNAL(triggered()), this, SLOT(disconnectINS()));
    setCentralWidget(hud);
}



//sensor device page
void AHRSWindow::connectINS()
{
    qDebug() << "Connect to device ...";
    QString deviceName;
    QStringList items;

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Name : " << info.portName();
        qDebug() << "Description : " << info.description();
        qDebug() << "Manufacturer: " << info.manufacturer();

        items << QString(info.portName() + ":" + info.description() + ":" + info.manufacturer());
    }

    if (items.length() <= 0) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::information(this, tr("GPS Device not Found!"), QDialog::tr("<p>Serial port not detected!<p>Make sure the GPS Device is connected and plugged into this system."));
        if (reply == QMessageBox::Ok)
            qDebug() << "Ok pressed";
        else
            qDebug() << "Escape pressed";
        deviceName = "none";
    }
    else {
        bool ok;
        QString item = QInputDialog::getItem(this, tr("QInputDialog::getItem()"), tr("Device:"), items, 0, false, &ok);
        if (ok && !item.isEmpty()) {
            qDebug() << "Selected:" << item;
        }
        deviceName = item;
        hud->deviceLabel->setText(deviceName);
    }
    receiver = new AHRSReceiver(deviceName.split(':')[0]);
    receiver->begin();
    hud->setReceiver(receiver);

}
void AHRSWindow::disconnectINS()
{
    if (receiver != nullptr)
        receiver->end();
}

void AHRSWindow::closeEvent(QCloseEvent* event)
{
    disconnectINS();
}

AHRSWindow::~AHRSWindow()
{
    disconnectINS();
    delete ui;
    delete hud;
    delete receiver;
}
