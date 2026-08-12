#ifndef LIVOXVIEWER_PARAMETERUISTATE_H
#define LIVOXVIEWER_PARAMETERUISTATE_H

#include <QFile>
#include <QMap>
#include <QSet>
#include <QString>
#include <QVector>
#include <cstdint>

class QLabel;
class QWidget;

struct ParameterUiState
{
    QMap<uint16_t, QString> values;
    QMap<uint16_t, QString> csvValues;
    QMap<uint16_t, QLabel*> labels;
    QMap<uint16_t, QWidget*> controls;

    QSet<uint16_t> configurableKeys;
    QSet<uint16_t> statusKeys;
    QSet<uint16_t> updatedConfigKeys;

    QFile recordFile;
    bool isRecording = false;
    QString recordFilePath;
    QMap<uint16_t, QString> recordedKeys;
    QVector<uint16_t> recordedOrder;
};

#endif // LIVOXVIEWER_PARAMETERUISTATE_H
