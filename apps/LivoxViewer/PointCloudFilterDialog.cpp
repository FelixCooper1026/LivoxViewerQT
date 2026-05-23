#include "LivoxViewerWindow.h"
void LivoxViewerWindow::showPointCloudFilterDialog()
{
        if (!filterDialog) {
            filterDialog = new QDialog(this);
            filterDialog->setWindowTitle("点云滤波");
            filterDialog->setMinimumWidth(500);
            QVBoxLayout* layout = new QVBoxLayout(filterDialog);

            // Tag值滤波设置
            QGroupBox* tagGroup = new QGroupBox("Tag值滤波", filterDialog);
            QVBoxLayout* tagLayout = new QVBoxLayout(tagGroup);



            auto makeTagRow = [&](const QString& label, int& value, QSpinBox*& spin, QLabel*& desc, const QString& meaning) {
                QWidget* row = new QWidget(filterDialog);
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

            makeTagRow("Bit[7-6]", filterTagVal76, filterSpin76, desc76, meaning76);
            makeTagRow("Bit[5-4]", filterTagVal54, filterSpin54, desc54, meaning54);
            makeTagRow("Bit[3-2]", filterTagVal32, filterSpin32, desc32, meaning32);
            makeTagRow("Bit[1-0]", filterTagVal10, filterSpin10, desc10, meaning10);

            // 设置初始值
            if (filterSpin76) filterSpin76->setValue(filterTagVal76);
            if (filterSpin54) filterSpin54->setValue(filterTagVal54);
            if (filterSpin32) filterSpin32->setValue(filterTagVal32);
            if (filterSpin10) filterSpin10->setValue(filterTagVal10);

            layout->addWidget(tagGroup);



            // 滤噪列表
            QGroupBox* filterListGroup = new QGroupBox("滤噪列表", filterDialog);
            QVBoxLayout* filterListLayout = new QVBoxLayout(filterListGroup);

            // 添加Tag值到滤噪列表
            QWidget* addFilterRow = new QWidget(filterDialog);
            QHBoxLayout* addFilterLayout = new QHBoxLayout(addFilterRow);
            addFilterLayout->setContentsMargins(0,0,0,0);

            QLabel* addFilterLabel = new QLabel("当前Tag值:", addFilterRow);
            QLabel* currentTagLabel = new QLabel("0", addFilterRow);
            currentTagLabel->setStyleSheet("font-weight: bold; color: green;");
            addNoiseFilterButton = new QPushButton("添加到滤噪列表", addFilterRow);
            addNoiseFilterButton->setEnabled(false);

            addFilterLayout->addWidget(addFilterLabel);
            addFilterLayout->addWidget(currentTagLabel);
            addFilterLayout->addSpacing(12);
            addFilterLayout->addWidget(addNoiseFilterButton);
            addFilterLayout->addStretch();
            filterListLayout->addWidget(addFilterRow);

            // 滤噪列表显示
            noiseFilterList = new QListWidget(filterDialog);
            noiseFilterList->setMaximumHeight(120);
            filterListLayout->addWidget(noiseFilterList);

            // 移除按钮
            QHBoxLayout* removeFilterLayout = new QHBoxLayout();
            removeNoiseFilterButton = new QPushButton("移除选中项", filterDialog);
            removeNoiseFilterButton->setEnabled(false);
            removeFilterLayout->addWidget(removeNoiseFilterButton);
            removeFilterLayout->addStretch();
            filterListLayout->addLayout(removeFilterLayout);

            layout->addWidget(filterListGroup);

            // 噪点处理选项（全局设置）
            QGroupBox* noiseGroup = new QGroupBox("噪点处理", filterDialog);
            QVBoxLayout* noiseLayout = new QVBoxLayout(noiseGroup);

            showNoiseCheck = new QCheckBox("高亮显示噪点", noiseGroup);
            removeNoiseCheck = new QCheckBox("移除噪点（仅移除显示，并非真正不输出）", noiseGroup);

            noiseLayout->addWidget(showNoiseCheck);
            noiseLayout->addWidget(removeNoiseCheck);
            layout->addWidget(noiseGroup);

            // 控制按钮
            QWidget* ctrlRow = new QWidget(filterDialog);
            QHBoxLayout* ctrlLayout = new QHBoxLayout(ctrlRow);
            ctrlLayout->setContentsMargins(0,0,0,0);
            QPushButton* closeBtn = new QPushButton("关闭", ctrlRow);
            ctrlLayout->addStretch();
            ctrlLayout->addWidget(closeBtn);
            layout->addWidget(ctrlRow);

            // 连接信号
            auto updateTagLabel = [this]() {
                if (filterTagLabel && filterTagLabel->isVisible()) {
                    uint8_t tag = makeFilterTag();
                    filterTagLabel->setText(QString::number(tag));
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

                if (desc76) desc76->setText(QString("%1（%2）").arg(meaning76, confToText(filterTagVal76)));
                if (desc54) desc54->setText(QString("%1（%2）").arg(meaning54, confToText(filterTagVal54)));
                if (desc32) desc32->setText(QString("%1（%2）").arg(meaning32, confToText(filterTagVal32)));
                if (desc10) desc10->setText(QString("%1（%2）").arg(meaning10, confToText(filterTagVal10)));
            };

            auto connectFilterSpin = [this, updateMeanings](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterDialog, [this, spin, desc, updateMeanings]() {
                    if (desc == "Bit[7-6]") filterTagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterTagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterTagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterTagVal10 = spin->value();

                    // 更新含义说明和标签
                    updateMeanings();

                    if (pointCloudView) pointCloudView->update();
                });
            };

            connectFilterSpin(filterSpin76, "Bit[7-6]");
            connectFilterSpin(filterSpin54, "Bit[5-4]");
            connectFilterSpin(filterSpin32, "Bit[3-2]");
            connectFilterSpin(filterSpin10, "Bit[1-0]");





            connect(showNoiseCheck, &QCheckBox::toggled, filterDialog, [this](bool en) {
                showNoisePoints = en;
                if (pointCloudView) pointCloudView->update();
            });
            connect(removeNoiseCheck, &QCheckBox::toggled, filterDialog, [this](bool en) {
                removeNoisePoints = en;
                if (pointCloudView) pointCloudView->update();
            });



            // 更新当前Tag值显示
            auto updateCurrentTagDisplay = [this, currentTagLabel]() {
                uint8_t tag = makeFilterTag();
                currentTagLabel->setText(QString::number(tag));

                // 检查是否已在列表中
                bool alreadyInList = noiseFilterTags.contains(tag);
                addNoiseFilterButton->setEnabled(!alreadyInList);
                addNoiseFilterButton->setText(alreadyInList ? "已在列表中" : "添加到滤噪列表");
            };

            // 连接滤噪列表相关信号
            connect(addNoiseFilterButton, &QPushButton::clicked, filterDialog, [this, currentTagLabel, updateCurrentTagDisplay]() {
                uint8_t tag = makeFilterTag();
                if (!noiseFilterTags.contains(tag)) {
                    noiseFilterTags.append(tag);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                }
            });

            connect(removeNoiseFilterButton, &QPushButton::clicked, filterDialog, [this, updateCurrentTagDisplay]() {
                int currentRow = noiseFilterList->currentRow();
                if (currentRow >= 0 && currentRow < noiseFilterTags.size()) {
                    noiseFilterTags.removeAt(currentRow);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                    if (pointCloudView) pointCloudView->update();
                }
            });

            connect(noiseFilterList, &QListWidget::itemSelectionChanged, filterDialog, [this]() {
                removeNoiseFilterButton->setEnabled(noiseFilterList->currentRow() >= 0);
            });



            // 重新连接spinbox信号，只更新含义说明和当前Tag值显示，不触发点云更新
            auto connectFilterSpinWithTag = [this, updateMeanings, updateCurrentTagDisplay](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterDialog, [this, spin, desc, updateMeanings, updateCurrentTagDisplay]() {
                    if (desc == "Bit[7-6]") filterTagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterTagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterTagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterTagVal10 = spin->value();

                    // 只更新含义说明和当前Tag值显示，不触发点云更新
                    updateMeanings();
                    updateCurrentTagDisplay();
                });
            };

            // 重新连接所有spinbox
            connectFilterSpinWithTag(filterSpin76, "Bit[7-6]");
            connectFilterSpinWithTag(filterSpin54, "Bit[5-4]");
            connectFilterSpinWithTag(filterSpin32, "Bit[3-2]");
            connectFilterSpinWithTag(filterSpin10, "Bit[1-0]");

            connect(closeBtn, &QPushButton::clicked, filterDialog, &QDialog::accept);

            // 设置初始状态
            if (showNoiseCheck) showNoiseCheck->setChecked(showNoisePoints);
            if (removeNoiseCheck) removeNoiseCheck->setChecked(removeNoisePoints);

            // 初始化含义说明和当前Tag值显示
            updateMeanings();
            updateCurrentTagDisplay();

            // 初始化滤噪列表
            updateNoiseFilterList();
        }

        filterDialog->show();
        filterDialog->raise();
        filterDialog->activateWindow();
}
