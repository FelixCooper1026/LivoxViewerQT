#include "LivoxViewerWindow.h"

class NumberItem : public QTableWidgetItem {
public:
    explicit NumberItem(double v, int decimals = 3)
        : QTableWidgetItem(QString::number(v, 'f', decimals)), value(v) {}
    explicit NumberItem(int v)
        : QTableWidgetItem(QString::number(v)), value(static_cast<double>(v)) {}
    explicit NumberItem(qlonglong v)
        : QTableWidgetItem(QString::number(v)), value(static_cast<double>(v)) {}
    bool operator<(const QTableWidgetItem& other) const override {
        const NumberItem* o = dynamic_cast<const NumberItem*>(&other);
        if (o) return value < o->value;
        return QTableWidgetItem::operator<(other);
    }
private:
    double value;
};

void LivoxViewerWindow::onMeasurementUpdated()
{
    if (!pointCloudView) return;
    if (pointCloudView->hasMeasureP1() && !pointCloudView->hasMeasureP2()) {
        statusLabelBar->setText("测距：已选择第一点，按住Ctrl+左键选择第二点");
    } else if (pointCloudView->hasMeasureP1() && pointCloudView->hasMeasureP2()) {
        double d = pointCloudView->getMeasureDistance();
        statusLabelBar->setText(QString("测距结果：%1 m").arg(d, 0, 'f', 3));
        logMessage(QString("测距完成：%1 m").arg(d, 0, 'f', 3));
    } else {
        statusLabelBar->setText("测距模式：按住Ctrl+左键选择第一点");
    }
}

void LivoxViewerWindow::updateSelectionTableAndLog()
{
    QVector<PointCloudPoint> pts;
    if (pointCloudView->hasSelectionAabb()) {
        pts = pointCloudView->pointsInPersistSelection(20000000);
    } else {
        QRect sel = pointCloudView->currentSelectionRect();
        if (!sel.isEmpty()) pts = pointCloudView->pointsInRect(sel, 20000000);
    }

    QTableWidget* table = attrTable ? attrTable : selectionTable;
    if (!table) return;

    if (!pts.isEmpty()) {
        int totalCount = pts.size();
        int zeroCount = 0;
        for (const PointCloudPoint& p : pts) {
            if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) {
                ++zeroCount;
            }
        }
        int validCount = totalCount - zeroCount;

        if (totalCount != lastSelectionCount) {
            lastSelectionCount = totalCount;
            logMessage(QString("框选点云个数: %1, 零点个数: %2, 总数: %3")
                       .arg(validCount).arg(zeroCount).arg(totalCount));
        }

        bool sorting = table->isSortingEnabled();
        table->setSortingEnabled(false);
        table->clearContents();
        table->setRowCount(0);
        const int maxRows = 500;
        int rows = 0;
        for (const PointCloudPoint& p : pts) {
            if (rows >= maxRows) break;
            int row = table->rowCount();
            table->insertRow(row);
            auto* i0 = new NumberItem(row + 1);
            auto* i1 = new NumberItem(p.x, 3);
            auto* i2 = new NumberItem(p.y, 3);
            auto* i3 = new NumberItem(p.z, 3);
            auto* i4 = new NumberItem(static_cast<int>(p.reflectivity));
            auto* i5 = new NumberItem(static_cast<int>(p.tag));
            i0->setTextAlignment(Qt::AlignCenter);
            i1->setTextAlignment(Qt::AlignCenter);
            i2->setTextAlignment(Qt::AlignCenter);
            i3->setTextAlignment(Qt::AlignCenter);
            i4->setTextAlignment(Qt::AlignCenter);
            i5->setTextAlignment(Qt::AlignCenter);
            table->setItem(row, 0, i0);
            table->setItem(row, 1, i1);
            table->setItem(row, 2, i2);
            table->setItem(row, 3, i3);
            table->setItem(row, 4, i4);
            table->setItem(row, 5, i5);
            rows++;
        }
        table->setSortingEnabled(sorting);
    } else {
        if (lastSelectionCount != -1) {
            lastSelectionCount = -1;
            logMessage("已清除框选");
        }
        if (table) {
            table->setSortingEnabled(false);
            table->clearContents();
            table->setRowCount(0);
            table->setSortingEnabled(true);
        }
    }
}

void LivoxViewerWindow::onSelectionFinished()
{
    if (!pointCloudView || !(attrTable || selectionTable)) return;
    updateSelectionTableAndLog();
}
