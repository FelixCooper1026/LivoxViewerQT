#ifndef LIVOXVIEWER_SLAMCONTROLDIALOG_H
#define LIVOXVIEWER_SLAMCONTROLDIALOG_H

#include <QDialog>
#include <QMap>

class QLabel;
class QCheckBox;
class QFormLayout;
class LivoxViewerWindow;
class SlamUiBridge;

class SlamControlDialog : public QDialog
{
    Q_OBJECT

public:
    SlamControlDialog(LivoxViewerWindow* window, SlamUiBridge* bridge, QWidget* parent = nullptr);

private:
    void refreshFields();
    QLabel* addField(QFormLayout* form, const QString& name);

    LivoxViewerWindow* m_window = nullptr;
    SlamUiBridge* m_bridge = nullptr;
    QMap<QString, QLabel*> m_fields;
    QCheckBox* m_mapPreviewCheck = nullptr;
};

#endif // LIVOXVIEWER_SLAMCONTROLDIALOG_H
