#include "LivoxViewerWindow.h"
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QRadioButton>
#include <QStandardPaths>
void LivoxViewerWindow::showFormatConvertDialog()
{
        QSettings settings("Livox", "LivoxViewerQT");
        QString lastSrc = settings.value("convert/lastSource", "").toString();

        QDialog dlg(this);
        dlg.setWindowTitle("LVX2格式转换");
        dlg.resize(620, 340);
        QVBoxLayout* root = new QVBoxLayout(&dlg);

        QGroupBox* srcBox = new QGroupBox("源文件", &dlg);
        QHBoxLayout* srcLayout = new QHBoxLayout(srcBox);
        QLineEdit* srcEdit = new QLineEdit(srcBox);
        srcEdit->setText(lastSrc);
        QPushButton* srcBtn = new QPushButton("选择LVX2", srcBox);
        srcLayout->addWidget(srcEdit, 1);
        srcLayout->addWidget(srcBtn);
        root->addWidget(srcBox);

        QGroupBox* modeBox = new QGroupBox("转换模式", &dlg);
        QVBoxLayout* modeLayout = new QVBoxLayout(modeBox);
        QRadioButton* modeMerge = new QRadioButton("所有帧转换为一个文件", modeBox);
        QRadioButton* modeSplit = new QRadioButton("100ms单帧转换为多个文件", modeBox);
        modeMerge->setChecked(true);
        modeLayout->addWidget(modeMerge);
        modeLayout->addWidget(modeSplit);
        root->addWidget(modeBox);

        QGroupBox* formatBox = new QGroupBox("转换类型", &dlg);
        QHBoxLayout* formatLayout = new QHBoxLayout(formatBox);
        QRadioButton* fmtPcd = new QRadioButton("PCD", formatBox);
        QRadioButton* fmtLas = new QRadioButton("LAS", formatBox);
        QRadioButton* fmtCsv = new QRadioButton("CSV", formatBox);
        QRadioButton* fmtTxt = new QRadioButton("TXT", formatBox);
        fmtPcd->setChecked(true);
        formatLayout->addWidget(fmtPcd);
        formatLayout->addWidget(fmtLas);
        formatLayout->addWidget(fmtCsv);
        formatLayout->addWidget(fmtTxt);
        formatLayout->addStretch();
        root->addWidget(formatBox);

        QGroupBox* outBox = new QGroupBox("输出文件", &dlg);
        QGridLayout* outLayout = new QGridLayout(outBox);
        QLabel* outDirLabel = new QLabel("保存路径:", outBox);
        QLineEdit* outDirEdit = new QLineEdit(outBox);
        QPushButton* outDirBtn = new QPushButton("选择路径", outBox);
        QLabel* outNameLabel = new QLabel("文件名:", outBox);
        QLineEdit* outNameEdit = new QLineEdit(outBox);
        outLayout->addWidget(outDirLabel, 0, 0);
        outLayout->addWidget(outDirEdit, 0, 1);
        outLayout->addWidget(outDirBtn, 0, 2);
        outLayout->addWidget(outNameLabel, 1, 0);
        outLayout->addWidget(outNameEdit, 1, 1, 1, 2);
        root->addWidget(outBox);

        QProgressBar* progress = new QProgressBar(&dlg);
        progress->setRange(0, 100);
        progress->setValue(0);
        root->addWidget(progress);

        QLabel* resultLabel = new QLabel(&dlg);
        resultLabel->setText("就绪");
        root->addWidget(resultLabel);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
        QPushButton* startBtn = new QPushButton("开始转换", &dlg);
        box->addButton(startBtn, QDialogButtonBox::ActionRole);
        root->addWidget(box);

        auto syncDefaultOutput = [srcEdit, outDirEdit, outNameEdit]() {
            const QFileInfo fi(srcEdit->text());
            if (fi.exists()) {
                outDirEdit->setText(fi.absolutePath());
                outNameEdit->setText(fi.completeBaseName());
            }
        };
        if (!srcEdit->text().isEmpty()) {
            syncDefaultOutput();
        }

        connect(srcBtn, &QPushButton::clicked, &dlg, [this, srcEdit, &settings, syncDefaultOutput]() {
            QString startDir = QFileInfo(srcEdit->text()).absolutePath();
            if (startDir.isEmpty()) {
                startDir = settings.value("convert/lastSourceDir", QDir::homePath()).toString();
            }
            const QString p = QFileDialog::getOpenFileName(this, "选择LVX2源文件", startDir, "LVX2点云 (*.lvx2)");
            if (p.isEmpty()) {
                return;
            }
            srcEdit->setText(p);
            settings.setValue("convert/lastSourceDir", QFileInfo(p).absolutePath());
            syncDefaultOutput();
        });
        connect(outDirBtn, &QPushButton::clicked, &dlg, [this, outDirEdit]() {
            const QString dir = QFileDialog::getExistingDirectory(this, "选择保存路径",
                outDirEdit->text().isEmpty() ? QDir::homePath() : outDirEdit->text());
            if (!dir.isEmpty()) {
                outDirEdit->setText(dir);
            }
        });
        connect(startBtn, &QPushButton::clicked, &dlg, [this, &settings, srcEdit, outDirEdit, outNameEdit,
                                                         modeSplit, fmtLas, fmtCsv, fmtTxt, progress, resultLabel, startBtn]() {
            const QString srcPath = srcEdit->text().trimmed();
            const QString outDir = outDirEdit->text().trimmed();
            const QString outName = outNameEdit->text().trimmed();
            if (srcPath.isEmpty() || outDir.isEmpty() || outName.isEmpty()) {
                resultLabel->setText("错误：请完整选择源文件、保存路径和文件名");
                return;
            }
            if (!QFileInfo::exists(srcPath)) {
                resultLabel->setText("错误：源文件不存在");
                return;
            }
            if (!QDir(outDir).exists()) {
                resultLabel->setText("错误：保存路径不存在");
                return;
            }

            settings.setValue("convert/lastSource", srcPath);
            const QString outputNoExt = QDir(outDir).filePath(outName);
            const Lvx2ConvertMode mode = modeSplit->isChecked()
                ? Lvx2ConvertMode::SplitBy100ms
                : Lvx2ConvertMode::MergeAllToOne;
            Lvx2ConvertFormat format = Lvx2ConvertFormat::PCD;
            if (fmtLas->isChecked()) {
                format = Lvx2ConvertFormat::LAS;
            } else if (fmtCsv->isChecked()) {
                format = Lvx2ConvertFormat::CSV;
            } else if (fmtTxt->isChecked()) {
                format = Lvx2ConvertFormat::TXT;
            }

            startBtn->setEnabled(false);
            progress->setValue(0);
            resultLabel->setText("正在转换...");
            bool ok = convertLvx2File(srcPath, outputNoExt, mode, format, [&](int done, int total) {
                const int value = (total > 0) ? (done * 100 / total) : 0;
                progress->setValue(value);
            });
            startBtn->setEnabled(true);
            progress->setValue(ok ? 100 : progress->value());
            resultLabel->setText(ok ? "转换完成，可继续选择其他源文件进行转换" :
                                      "转换失败，请检查LVX2文件格式或输出路径权限");
        });
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        dlg.exec();
}
