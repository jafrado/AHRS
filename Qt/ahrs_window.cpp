#include "ahrs_window.h"
#include "appearance_dialog.h"
#include <QLineEdit>
#include <QActionGroup>

AHRSWindow::AHRSWindow(QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui::AHRSWindow),
    hud(nullptr),
    receiver(nullptr)
{
    ui->setupUi(this);

    // Setup AHRS HUD (single central widget)
    hud = new AHRSWidget(this);
    connect(ui->actionConnect, SIGNAL(triggered()), this, SLOT(connectINS()));
    connect(ui->actionClose,   SIGNAL(triggered()), this, SLOT(disconnectINS()));
    setCentralWidget(hud);

    // Options menu
    QMenu        *optMenu    = menuBar()->addMenu("Options");
    QMenu        *unitsMenu  = optMenu->addMenu("Units");
    QActionGroup *unitsGroup = new QActionGroup(this);

    QAction *actImperial = unitsMenu->addAction("Imperial");
    QAction *actMetric   = unitsMenu->addAction("Metric");
    actImperial->setCheckable(true);
    actMetric->setCheckable(true);
    unitsGroup->addAction(actImperial);
    unitsGroup->addAction(actMetric);
    unitsGroup->setExclusive(true);

    // Reflect the units already loaded from settings by AHRSWidget constructor
    actImperial->setChecked(hud->units() == AHRSWidget::Imperial);
    actMetric->setChecked(hud->units() == AHRSWidget::Metric);

    connect(actImperial, &QAction::triggered, [this]{ hud->setUnits(AHRSWidget::Imperial); });
    connect(actMetric,   &QAction::triggered, [this]{ hud->setUnits(AHRSWidget::Metric); });

    QAction *actAppearance = optMenu->addAction("Appearance...");
    connect(actAppearance, &QAction::triggered, [this]{
        AppearanceDialog dlg(hud, this);
        dlg.exec();
    });

    // Open 100 px wider than the layout's natural recommendation
    QSize hint = sizeHint();
    resize(hint.width() + 250, hint.height());
}



//sensor device page
void AHRSWindow::connectINS()
{
    qDebug() << "Connect to device ...";

    QMessageBox::StandardButton choice = QMessageBox::question(
        this, "Connection Type",
        "Connect via TCP?\n\nYes = TCP (emulator)\nNo = serial port (hardware)",
        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

    if (choice == QMessageBox::Cancel)
        return;

    if (choice == QMessageBox::Yes) {
        bool ok;
        QString hostPort = QInputDialog::getText(this, "TCP Connection",
            "Host:Port:", QLineEdit::Normal, "localhost:5555", &ok);
        if (!ok || hostPort.trimmed().isEmpty())
            return;
        QStringList parts = hostPort.trimmed().split(':');
        QString host = parts[0].trimmed();
        quint16 port = (parts.size() > 1) ? parts[1].trimmed().toUShort() : 5555;
        hud->deviceLabel->setText("TCP: " + hostPort.trimmed());
        receiver = new AHRSReceiver(host, port);
    } else {
        QStringList items;
        foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
            QString name = info.portName().isEmpty() ? info.systemLocation() : info.portName();
            qDebug() << "Name : " << name;
            qDebug() << "Description : " << info.description();
            qDebug() << "Manufacturer: " << info.manufacturer();
            items << QString(name + ":" + info.description() + ":" + info.manufacturer());
        }
        if (items.isEmpty()) {
            QMessageBox::information(this, tr("GPS Device not Found!"),
                tr("<p>Serial port not detected.<p>Make sure the GPS device is connected."));
            return;
        }
        bool ok;
        QString item = QInputDialog::getItem(this, tr("Serial Port"), tr("Device:"), items, 0, false, &ok);
        if (!ok || item.isEmpty())
            return;
        QString deviceName = item.split(':')[0];
        hud->deviceLabel->setText(deviceName);
        receiver = new AHRSReceiver(deviceName);
    }

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
