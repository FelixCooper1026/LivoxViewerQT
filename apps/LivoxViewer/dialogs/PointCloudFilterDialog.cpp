#include "LivoxViewerWindow.h"
#include <QListWidget>

void LivoxViewerWindow::showPointCloudFilterDialog()
{
        if (!filterState.dialog) {
            filterState.dialog = new QDialog(this);
            filterState.dialog->setWindowTitle("点云滤波");
            filterState.dialog->setMinimumWidth(500);
            QVBoxLayout* layout = new QVBoxLayout(filterState.dialog);

            // Tag值滤波设置
            QGroupBox* tagGroup = new QGroupBox("Tag值滤波", filterState.dialog);
            QVBoxLayout* tagLayout = new QVBoxLayout(tagGroup);



            auto makeTagRow = [&](const QString& label, int& value, QSpinBox*& spin, QLabel*& desc, const QString& meaning) {
                QWidget* row = new QWidget(filterState.dialog);
                QHBoxLayout* h = new QHBoxLayout(row);
                h->setContentsMargins(0,0,0,0);
                QLabel* lbl = new QLabel(label + ":", row);
                spin = new QSpinBox(row);
                spin->setRange(0, 3);
                spin->setValue(value);
                spin->setToolTip("0: 置信度优; 1: 置信度中; 2: 置信度差; 3: 保留");
                desc = new QLabel(meaning, row);
                desc->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
                h->addWidget(lbl);
                h->addSpacing(8);
                h->addWidget(spin);
                h->addSpacing(12);
                h->addWidget(desc, 1);
                tagLayout->addWidget(row);
            };

            QLabel* desc76, *desc54, *desc32, *desc10;
            QString meaning76 = "保留位";
            QString meaning54 = "近处回吸噪点";
            QString meaning32 = "雨雾、灰尘等微小颗粒";
            QString meaning10 = "相近物体间的粘连点云";

            makeTagRow("Bit[7-6]", filterState.tagVal76, filterState.spin76, desc76, meaning76);
            makeTagRow("Bit[5-4]", filterState.tagVal54, filterState.spin54, desc54, meaning54);
            makeTagRow("Bit[3-2]", filterState.tagVal32, filterState.spin32, desc32, meaning32);
            makeTagRow("Bit[1-0]", filterState.tagVal10, filterState.spin10, desc10, meaning10);

            // 设置初始值
            if (filterState.spin76) filterState.spin76->setValue(filterState.tagVal76);
            if (filterState.spin54) filterState.spin54->setValue(filterState.tagVal54);
            if (filterState.spin32) filterState.spin32->setValue(filterState.tagVal32);
            if (filterState.spin10) filterState.spin10->setValue(filterState.tagVal10);

            layout->addWidget(tagGroup);



            // 滤噪列表
            QGroupBox* filterListGroup = new QGroupBox("滤噪列表", filterState.dialog);
            QVBoxLayout* filterListLayout = new QVBoxLayout(filterListGroup);

            // 添加Tag值到滤噪列表
            QWidget* addFilterRow = new QWidget(filterState.dialog);
            QHBoxLayout* addFilterLayout = new QHBoxLayout(addFilterRow);
            addFilterLayout->setContentsMargins(0,0,0,0);

            QLabel* addFilterLabel = new QLabel("当前Tag值:", addFilterRow);
            QLabel* currentTagLabel = new QLabel("0", addFilterRow);
            currentTagLabel->setStyleSheet("font-weight: bold; color: green;");
            filterState.addNoiseFilterButton = new QPushButton("添加到滤噪列表", addFilterRow);
            filterState.addNoiseFilterButton->setEnabled(false);

            addFilterLayout->addWidget(addFilterLabel);
            addFilterLayout->addWidget(currentTagLabel);
            addFilterLayout->addSpacing(12);
            addFilterLayout->addWidget(filterState.addNoiseFilterButton);
            addFilterLayout->addStretch();
            filterListLayout->addWidget(addFilterRow);

            // 滤噪列表显示
            filterState.noiseFilterList = new QListWidget(filterState.dialog);
            filterState.noiseFilterList->setMaximumHeight(120);
            filterListLayout->addWidget(filterState.noiseFilterList);

            // 移除按钮
            QHBoxLayout* removeFilterLayout = new QHBoxLayout();
            filterState.removeNoiseFilterButton = new QPushButton("移除选中项", filterState.dialog);
            filterState.removeNoiseFilterButton->setEnabled(false);
            removeFilterLayout->addWidget(filterState.removeNoiseFilterButton);
            removeFilterLayout->addStretch();
            filterListLayout->addLayout(removeFilterLayout);

            layout->addWidget(filterListGroup);

            // 噪点处理选项（全局设置）
            QGroupBox* noiseGroup = new QGroupBox("噪点处理", filterState.dialog);
            QVBoxLayout* noiseLayout = new QVBoxLayout(noiseGroup);

            filterState.showNoiseCheck = new QCheckBox("高亮显示噪点", noiseGroup);
            filterState.removeNoiseCheck = new QCheckBox("移除噪点（仅移除显示，并非真正不输出）", noiseGroup);

            noiseLayout->addWidget(filterState.showNoiseCheck);
            noiseLayout->addWidget(filterState.removeNoiseCheck);
            layout->addWidget(noiseGroup);

            // 控制按钮
            QWidget* ctrlRow = new QWidget(filterState.dialog);
            QHBoxLayout* ctrlLayout = new QHBoxLayout(ctrlRow);
            ctrlLayout->setContentsMargins(0,0,0,0);
            QPushButton* closeBtn = new QPushButton("关闭", ctrlRow);
            ctrlLayout->addStretch();
            ctrlLayout->addWidget(closeBtn);
            layout->addWidget(ctrlRow);

            // 连接信号
            auto updateTagLabel = [this]() {
                if (filterState.tagLabel && filterState.tagLabel->isVisible()) {
                    uint8_t tag = makeFilterTag();
                    filterState.tagLabel->setText(QString::number(tag));
                }
            };

            // 动态更新含义说明
            auto updateMeanings = [this, desc76, desc54, desc32, desc10, meaning76, meaning54, meaning32, meaning10]() {
                auto confToText = [](int v) {
                    switch(v & 3) {
                        case 0: return QString("置信度优");
                        case 1: return QString("置信度中");
                        case 2: return QString("置信度差");
                        default: return QString("保留");
                    }
                };

                if (desc76) desc76->setText(QString("%1（%2）").arg(meaning76, confToText(filterState.tagVal76)));
                if (desc54) desc54->setText(QString("%1（%2）").arg(meaning54, confToText(filterState.tagVal54)));
                if (desc32) desc32->setText(QString("%1（%2）").arg(meaning32, confToText(filterState.tagVal32)));
                if (desc10) desc10->setText(QString("%1（%2）").arg(meaning10, confToText(filterState.tagVal10)));
            };

            auto refreshPointCloud = [this]() {
                if (playbackState.active && playbackState.frame >= 0) {
                    playbackState.resetSlidingWindow();
                    showLvx2PlaybackFrame(playbackState.frame);
                } else if (pointCloudView) {
                    pointCloudView->update();
                }
            };

            auto connectFilterSpin = [this, updateMeanings, refreshPointCloud](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterState.dialog, [this, spin, desc, updateMeanings, refreshPointCloud]() {
                    if (desc == "Bit[7-6]") filterState.tagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterState.tagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterState.tagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterState.tagVal10 = spin->value();

                    // 更新含义说明和标签
                    updateMeanings();

                    refreshPointCloud();
                });
            };

            connectFilterSpin(filterState.spin76, "Bit[7-6]");
            connectFilterSpin(filterState.spin54, "Bit[5-4]");
            connectFilterSpin(filterState.spin32, "Bit[3-2]");
            connectFilterSpin(filterState.spin10, "Bit[1-0]");





            connect(filterState.showNoiseCheck, &QCheckBox::toggled, filterState.dialog, [this, refreshPointCloud](bool en) {
                filterState.showNoisePoints = en;
                refreshPointCloud();
            });
            connect(filterState.removeNoiseCheck, &QCheckBox::toggled, filterState.dialog, [this, refreshPointCloud](bool en) {
                filterState.removeNoisePoints = en;
                refreshPointCloud();
            });



            // 更新当前Tag值显示
            auto updateCurrentTagDisplay = [this, currentTagLabel]() {
                uint8_t tag = makeFilterTag();
                currentTagLabel->setText(QString::number(tag));

                // 检查是否已在列表中
                bool alreadyInList = filterState.noiseFilterTags.contains(tag);
                filterState.addNoiseFilterButton->setEnabled(!alreadyInList);
                filterState.addNoiseFilterButton->setText(alreadyInList ? "已在列表中" : "添加到滤噪列表");
            };

            // 连接滤噪列表相关信号
            connect(filterState.addNoiseFilterButton, &QPushButton::clicked, filterState.dialog, [this, currentTagLabel, updateCurrentTagDisplay, refreshPointCloud]() {
                uint8_t tag = makeFilterTag();
                if (!filterState.noiseFilterTags.contains(tag)) {
                    filterState.noiseFilterTags.append(tag);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                    refreshPointCloud();
                }
            });

            connect(filterState.removeNoiseFilterButton, &QPushButton::clicked, filterState.dialog, [this, updateCurrentTagDisplay, refreshPointCloud]() {
                int currentRow = filterState.noiseFilterList->currentRow();
                if (currentRow >= 0 && currentRow < filterState.noiseFilterTags.size()) {
                    filterState.noiseFilterTags.removeAt(currentRow);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                    refreshPointCloud();
                }
            });

            connect(filterState.noiseFilterList, &QListWidget::itemSelectionChanged, filterState.dialog, [this]() {
                filterState.removeNoiseFilterButton->setEnabled(filterState.noiseFilterList->currentRow() >= 0);
            });



            // 重新连接spinbox信号，只更新含义说明和当前Tag值显示，不触发点云更新
            auto connectFilterSpinWithTag = [this, updateMeanings, updateCurrentTagDisplay](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterState.dialog, [this, spin, desc, updateMeanings, updateCurrentTagDisplay]() {
                    if (desc == "Bit[7-6]") filterState.tagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterState.tagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterState.tagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterState.tagVal10 = spin->value();

                    // 只更新含义说明和当前Tag值显示，不触发点云更新
                    updateMeanings();
                    updateCurrentTagDisplay();
                });
            };

            // 重新连接所有spinbox
            connectFilterSpinWithTag(filterState.spin76, "Bit[7-6]");
            connectFilterSpinWithTag(filterState.spin54, "Bit[5-4]");
            connectFilterSpinWithTag(filterState.spin32, "Bit[3-2]");
            connectFilterSpinWithTag(filterState.spin10, "Bit[1-0]");

            connect(closeBtn, &QPushButton::clicked, filterState.dialog, &QDialog::accept);

            // 设置初始状态
            if (filterState.showNoiseCheck) filterState.showNoiseCheck->setChecked(filterState.showNoisePoints);
            if (filterState.removeNoiseCheck) filterState.removeNoiseCheck->setChecked(filterState.removeNoisePoints);

            // 初始化含义说明和当前Tag值显示
            updateMeanings();
            updateCurrentTagDisplay();

            // 初始化滤噪列表
            updateNoiseFilterList();
        }

        filterState.dialog->show();
        filterState.dialog->raise();
        filterState.dialog->activateWindow();
}
