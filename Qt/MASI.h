#ifndef MASI_H
#define MASI_H

/*
 * MASI -- Mach-enabled Airspeed Indicator
 * Black face, white scale (0-1000 in 10-unit ticks, labeled every 100),
 * 270 deg sweep (7:30 to 4:30 o'clock), orange Mach readout in centre.
 * Bezel matches AHRSAttitudeIndicator / AHRSCompass style.
 */

#include <QOpenGLWidget>
#include <QString>
#include <QColor>
#include <QFont>

class MASI : public QOpenGLWidget
{
    Q_OBJECT

public:
    enum Units { Metric, Imperial };

    explicit MASI(QWidget *parent = nullptr);
    ~MASI();

    // speed in current units (m/s if Metric, ft/s if Imperial)
    void setSpeed(double speed);
    void setMach(double mach);
    void setUnits(Units u);
    void setDisplayScale(double scale);
    void setInstrFont(const QFont &font);
    void setLabelColor(const QColor &color);

    double getSpeed() const { return m_speed; }
    double getMach()  const { return m_mach;  }
    Units  getUnits() const { return m_units; }

signals:
    void widgetReplot();

protected slots:
    void widgetReplot_slot();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    int     m_size;
    int     m_offset;
    double  m_speed;        // current speed in current units, clamped [0, 1000]
    double  m_mach;         // current Mach number
    Units   m_units;
    QString m_fontFamily;
    int     m_labelSize;
    QColor  m_labelColor;
    double  m_displayScale;
};

#endif // MASI_H
