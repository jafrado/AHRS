#include "appearance_dialog.h"
#include "AHRS.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QPushButton>
#include <QFrame>
#include <QColorDialog>
#include <QSettings>
#include <QCoreApplication>

AppearanceDialog::AppearanceDialog(AHRSWidget *hud, QWidget *parent)
    : QDialog(parent), m_hud(hud)
{
    setWindowTitle("Appearance");
    setMinimumWidth(580);

    // Capture current HUD state (restored on Cancel)
    m_savedAHRSFont    = m_ahrsFont    = hud->ahrsFont();
    m_savedAHRSColor   = m_ahrsColor   = hud->ahrsColor();
    m_savedStatusFont  = m_statusFont  = hud->statusFont();
    m_savedStatusColor = m_statusColor = hud->statusColor();

    // Group selector
    m_groupCombo = new QComboBox(this);
    m_groupCombo->addItem("AHRS Instruments");
    m_groupCombo->addItem("Status Panel");

    QHBoxLayout *groupRow = new QHBoxLayout;
    groupRow->addWidget(new QLabel("Group:", this));
    groupRow->addWidget(m_groupCombo);
    groupRow->addStretch();

    // Font group box (left column)
    QGroupBox   *fontBox  = new QGroupBox("Font", this);
    QFormLayout *fontForm = new QFormLayout(fontBox);

    m_fontCombo   = new QFontComboBox(this);
    m_sizeSpinBox = new QSpinBox(this);
    m_sizeSpinBox->setRange(6, 72);
    m_sizeSpinBox->setSuffix(" pt");
    m_boldCheck   = new QCheckBox("Bold",   this);
    m_italicCheck = new QCheckBox("Italic", this);

    QHBoxLayout *styleRow = new QHBoxLayout;
    styleRow->addWidget(m_boldCheck);
    styleRow->addWidget(m_italicCheck);
    styleRow->addStretch();

    fontForm->addRow("Family:", m_fontCombo);
    fontForm->addRow("Size:",   m_sizeSpinBox);
    fontForm->addRow("Style:",  styleRow);

    // Preview (below font controls)
    QGroupBox   *prevBox    = new QGroupBox("Preview", this);
    QVBoxLayout *prevLayout = new QVBoxLayout(prevBox);
    m_previewLabel = new QLabel("Abc 123  N 45 deg  +2.5 ft/s", this);
    m_previewLabel->setAlignment(Qt::AlignCenter);
    m_previewLabel->setMinimumHeight(52);
    prevLayout->addWidget(m_previewLabel);

    QVBoxLayout *leftCol = new QVBoxLayout;
    leftCol->addWidget(fontBox);
    leftCol->addWidget(prevBox);
    leftCol->addStretch();

    // Color group box (right column)
    QGroupBox   *colorBox    = new QGroupBox("Color", this);
    QVBoxLayout *colorLayout = new QVBoxLayout(colorBox);

    m_colorSwatch = new QLabel(this);
    m_colorSwatch->setFixedSize(120, 80);
    m_colorSwatch->setFrameStyle(QFrame::Box | QFrame::Plain);
    m_colorSwatch->setLineWidth(2);

    m_hexEdit = new QLineEdit(this);
    m_hexEdit->setMaxLength(7);
    m_hexEdit->setFixedWidth(90);
    m_hexEdit->setPlaceholderText("#rrggbb");

    QPushButton *chooseBtn = new QPushButton("Choose...", this);

    QHBoxLayout *hexRow = new QHBoxLayout;
    hexRow->addWidget(new QLabel("Hex:", this));
    hexRow->addWidget(m_hexEdit);
    hexRow->addStretch();

    colorLayout->addWidget(m_colorSwatch, 0, Qt::AlignCenter);
    colorLayout->addSpacing(8);
    colorLayout->addLayout(hexRow);
    colorLayout->addSpacing(4);
    colorLayout->addWidget(chooseBtn, 0, Qt::AlignLeft);
    colorLayout->addStretch();

    // Middle row: font/preview left, color right
    QHBoxLayout *midRow = new QHBoxLayout;
    midRow->addLayout(leftCol, 3);
    midRow->addWidget(colorBox, 2);

    // Button row: Defaults on the left, Apply/Cancel/OK on the right
    QPushButton *defaultsBtn = new QPushButton("Defaults", this);
    QPushButton *applyBtn    = new QPushButton("Apply",    this);
    QPushButton *cancelBtn   = new QPushButton("Cancel",   this);
    QPushButton *okBtn       = new QPushButton("OK",       this);
    okBtn->setDefault(true);

    QHBoxLayout *btnRow = new QHBoxLayout;
    btnRow->addWidget(defaultsBtn);
    btnRow->addStretch();
    btnRow->addWidget(applyBtn);
    btnRow->addWidget(cancelBtn);
    btnRow->addWidget(okBtn);

    // Main layout
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(groupRow);
    mainLayout->addSpacing(4);
    mainLayout->addLayout(midRow, 1);
    mainLayout->addSpacing(4);
    mainLayout->addLayout(btnRow);

    // Populate controls from initial group state
    loadGroupSettings();

    // Connections
    connect(m_groupCombo,  QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &AppearanceDialog::onGroupChanged);
    connect(m_fontCombo,   &QFontComboBox::currentFontChanged,
            this, &AppearanceDialog::onControlChanged);
    connect(m_sizeSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &AppearanceDialog::onControlChanged);
    connect(m_boldCheck,   &QCheckBox::toggled, this, &AppearanceDialog::onControlChanged);
    connect(m_italicCheck, &QCheckBox::toggled, this, &AppearanceDialog::onControlChanged);
    connect(m_hexEdit,     &QLineEdit::textEdited, this, &AppearanceDialog::onHexEdited);
    connect(chooseBtn,     &QPushButton::clicked,  this, &AppearanceDialog::onChooseColor);
    connect(defaultsBtn,   &QPushButton::clicked,  this, &AppearanceDialog::onDefaults);
    connect(applyBtn,      &QPushButton::clicked,  this, &AppearanceDialog::onApply);
    connect(cancelBtn,     &QPushButton::clicked,  this, &QDialog::reject);
    connect(okBtn,         &QPushButton::clicked,  [this]{ onApply(); accept(); });
}

// Load the current group's stored state into controls

void AppearanceDialog::loadGroupSettings()
{
    int    idx = m_groupCombo->currentIndex();
    QFont  f   = (idx == 0) ? m_ahrsFont   : m_statusFont;
    QColor c   = (idx == 0) ? m_ahrsColor  : m_statusColor;

    m_fontCombo->blockSignals(true);
    m_sizeSpinBox->blockSignals(true);
    m_boldCheck->blockSignals(true);
    m_italicCheck->blockSignals(true);

    m_fontCombo->setCurrentFont(f);
    m_sizeSpinBox->setValue(f.pointSize() > 0 ? f.pointSize() : 10);
    m_boldCheck->setChecked(f.bold());
    m_italicCheck->setChecked(f.italic());

    m_fontCombo->blockSignals(false);
    m_sizeSpinBox->blockSignals(false);
    m_boldCheck->blockSignals(false);
    m_italicCheck->blockSignals(false);

    updateSwatchColor(c);
    updatePreview();
}

// Read controls and store into the specified group's state

void AppearanceDialog::saveGroupSettings(int groupIdx)
{
    QFont f = m_fontCombo->currentFont();
    f.setPointSize(m_sizeSpinBox->value());
    f.setBold(m_boldCheck->isChecked());
    f.setItalic(m_italicCheck->isChecked());

    if (groupIdx == 0) {
        m_ahrsFont  = f;
        m_ahrsColor = m_currentColor;
    } else {
        m_statusFont  = f;
        m_statusColor = m_currentColor;
    }
}

// Group switch: save previous group, load new group

void AppearanceDialog::onGroupChanged(int newIdx)
{
    saveGroupSettings(1 - newIdx);
    loadGroupSettings();
}

// Any font control changed

void AppearanceDialog::onControlChanged()
{
    saveGroupSettings(m_groupCombo->currentIndex());
    updatePreview();
}

// Color: open QColorDialog

void AppearanceDialog::onChooseColor()
{
    QColor c = QColorDialog::getColor(m_currentColor, this, "Choose Color");
    if (c.isValid())
        updateSwatchColor(c);
}

// Color: user typed hex value

void AppearanceDialog::onHexEdited(const QString &text)
{
    QString s = text.startsWith('#') ? text : '#' + text;
    QColor c(s);
    if (!c.isValid())
        return;

    m_currentColor = c;
    m_colorSwatch->setStyleSheet(QString("background-color: %1;").arg(c.name()));

    int idx = m_groupCombo->currentIndex();
    if (idx == 0) m_ahrsColor  = c;
    else          m_statusColor = c;

    updatePreview();
}

// Update swatch and hex edit together

void AppearanceDialog::updateSwatchColor(const QColor &c)
{
    m_currentColor = c;
    m_colorSwatch->setStyleSheet(QString("background-color: %1;").arg(c.name()));

    m_hexEdit->blockSignals(true);
    m_hexEdit->setText(c.name());
    m_hexEdit->blockSignals(false);

    int idx = m_groupCombo->currentIndex();
    if (idx == 0) m_ahrsColor  = c;
    else          m_statusColor = c;
}

// Refresh the preview label with current font + color

void AppearanceDialog::updatePreview()
{
    QFont f = m_fontCombo->currentFont();
    f.setPointSize(m_sizeSpinBox->value());
    f.setBold(m_boldCheck->isChecked());
    f.setItalic(m_italicCheck->isChecked());

    m_previewLabel->setFont(f);
    m_previewLabel->setStyleSheet(
        QString("color: %1; background-color: #111111; padding: 6px;")
        .arg(m_currentColor.name()));
}

// Push all group states to the live HUD (no disk write)

void AppearanceDialog::pushToHUD()
{
    m_hud->setAHRSFont(m_ahrsFont);
    m_hud->setAHRSColor(m_ahrsColor);
    m_hud->setStatusFont(m_statusFont);
    m_hud->setStatusColor(m_statusColor);
}

// Persist all settings to AHRS.ini

void AppearanceDialog::saveSettings()
{
    QString iniPath = QCoreApplication::applicationDirPath() + "/AHRS.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup("AHRS");
    settings.setValue("font_family", m_ahrsFont.family());
    settings.setValue("font_size",   m_ahrsFont.pointSize());
    settings.setValue("font_bold",   m_ahrsFont.bold());
    settings.setValue("font_italic", m_ahrsFont.italic());
    settings.setValue("color",       m_ahrsColor.name());
    settings.endGroup();

    settings.beginGroup("Status");
    settings.setValue("font_family", m_statusFont.family());
    settings.setValue("font_size",   m_statusFont.pointSize());
    settings.setValue("font_bold",   m_statusFont.bold());
    settings.setValue("font_italic", m_statusFont.italic());
    settings.setValue("color",       m_statusColor.name());
    settings.endGroup();
}

// Apply button: commit current controls, push to HUD, save to disk

void AppearanceDialog::onApply()
{
    saveGroupSettings(m_groupCombo->currentIndex());
    pushToHUD();
    saveSettings();
}

// Defaults button: reset all groups to factory defaults, push to HUD (no disk write)

void AppearanceDialog::onDefaults()
{
    m_ahrsFont    = QFont("Arial", 8);
    m_ahrsColor   = QColor(Qt::white);
    m_statusFont  = QFont("Arial", 12);
    m_statusColor = QColor(0, 255, 0);
    loadGroupSettings();
    pushToHUD();
}

// Cancel / window-X: revert HUD to pre-dialog state (no disk write)

void AppearanceDialog::reject()
{
    m_hud->setAHRSFont(m_savedAHRSFont);
    m_hud->setAHRSColor(m_savedAHRSColor);
    m_hud->setStatusFont(m_savedStatusFont);
    m_hud->setStatusColor(m_savedStatusColor);
    QDialog::reject();
}
