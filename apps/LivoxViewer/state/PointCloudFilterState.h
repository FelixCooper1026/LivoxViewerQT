#ifndef LIVOXVIEWER_POINTCLOUDFILTERSTATE_H
#define LIVOXVIEWER_POINTCLOUDFILTERSTATE_H

#include <QVector>
#include <cstdint>

class QCheckBox;
class QDialog;
class QLabel;
class QListWidget;
class QPushButton;
class QSpinBox;

struct PointCloudFilterState
{
    bool showNoisePoints = false;
    bool removeNoisePoints = false;

    int tagVal76 = 0;
    int tagVal54 = 0;
    int tagVal32 = 0;
    int tagVal10 = 0;
    QVector<uint8_t> noiseFilterTags;

    QDialog* dialog = nullptr;
    QSpinBox* spin76 = nullptr;
    QSpinBox* spin54 = nullptr;
    QSpinBox* spin32 = nullptr;
    QSpinBox* spin10 = nullptr;
    QLabel* tagLabel = nullptr;
    QCheckBox* showNoiseCheck = nullptr;
    QCheckBox* removeNoiseCheck = nullptr;
    QListWidget* noiseFilterList = nullptr;
    QPushButton* addNoiseFilterButton = nullptr;
    QPushButton* removeNoiseFilterButton = nullptr;
};

#endif // LIVOXVIEWER_POINTCLOUDFILTERSTATE_H
