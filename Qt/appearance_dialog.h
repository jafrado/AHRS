#ifndef APPEARANCE_DIALOG_H
#define APPEARANCE_DIALOG_H

#include <QDialog>
#include <QComboBox>
#include <QFontComboBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QColor>
#include <QFont>

class AHRSWidget;

class AppearanceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AppearanceDialog(AHRSWidget *hud, QWidget *parent = nullptr);

protected:
    void reject() override;

private slots:
    void onGroupChanged(int newIdx);
    void onControlChanged();
    void onChooseColor();
    void onHexEdited(const QString &text);
    void onApply();
    void onDefaults();

private:
    AHRSWidget    *m_hud;

    // Group selector
    QComboBox     *m_groupCombo;

    // Font controls
    QFontComboBox *m_fontCombo;
    QSpinBox      *m_sizeSpinBox;
    QCheckBox     *m_boldCheck;
    QCheckBox     *m_italicCheck;

    // Color controls
    QLabel        *m_colorSwatch;
    QLineEdit     *m_hexEdit;
    QColor         m_currentColor;

    // Preview
    QLabel        *m_previewLabel;

    // Per-group live state (edited in dialog, pushed on Apply/OK)
    QFont  m_ahrsFont,    m_statusFont;
    QColor m_ahrsColor,   m_statusColor;

    // State captured on open -- restored on Cancel
    QFont  m_savedAHRSFont,   m_savedStatusFont;
    QColor m_savedAHRSColor,  m_savedStatusColor;

    void loadGroupSettings();
    void saveGroupSettings(int groupIdx);
    void pushToHUD();
    void saveSettings();
    void updateSwatchColor(const QColor &c);
    void updatePreview();
};

#endif // APPEARANCE_DIALOG_H
