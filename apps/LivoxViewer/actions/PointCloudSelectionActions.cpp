#include "LivoxViewerWindow.h"

#include <QAbstractTableModel>
#include <QHeaderView>
#include <QTableView>

#include <algorithm>

namespace {

constexpr int kSelectionRoiRows = 500;

struct SelectionPointRow
{
    PointCloudPoint point;
    int originalIndex = 0;
};

class SelectionPointTableModel : public QAbstractTableModel
{
public:
    explicit SelectionPointTableModel(QObject* parent = nullptr)
        : QAbstractTableModel(parent)
    {
    }

    int rowCount(const QModelIndex& parent = QModelIndex()) const override
    {
        return parent.isValid() ? 0 : int(rows_.size());
    }

    int columnCount(const QModelIndex& parent = QModelIndex()) const override
    {
        return parent.isValid() ? 0 : 6;
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override
    {
        if (role != Qt::DisplayRole) {
            return {};
        }
        if (orientation == Qt::Horizontal) {
            static const QStringList headers = {"Index", "X(m)", "Y(m)", "Z(m)", "Refl", "Tag"};
            return headers.value(section);
        }
        return {};
    }

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override
    {
        if (!index.isValid() || index.row() < 0 || index.row() >= rows_.size()) {
            return {};
        }
        if (role == Qt::TextAlignmentRole) {
            return Qt::AlignCenter;
        }
        if (role != Qt::DisplayRole) {
            return {};
        }

        if (index.row() < roiStart_ || index.row() >= roiEnd()) {
            return QStringLiteral("...");
        }

        const SelectionPointRow& row = rows_.at(index.row());
        const PointCloudPoint& p = row.point;
        switch (index.column()) {
        case 0: return row.originalIndex + 1;
        case 1: return QString::number(p.x, 'f', 3);
        case 2: return QString::number(p.y, 'f', 3);
        case 3: return QString::number(p.z, 'f', 3);
        case 4: return int(p.reflectivity);
        case 5: return int(p.tag);
        default: return {};
        }
    }

    void sort(int column, Qt::SortOrder order = Qt::AscendingOrder) override
    {
        if (rows_.isEmpty()) {
            return;
        }

        beginResetModel();
        sortColumn_ = column;
        sortOrder_ = order;
        sortRows();
        roiStart_ = 0;
        endResetModel();
    }

    void setPoints(QVector<PointCloudPoint>&& points)
    {
        beginResetModel();
        const int previousRoiStart = roiStart_;
        rows_.clear();
        rows_.reserve(points.size());
        for (int i = 0; i < points.size(); ++i) {
            rows_.push_back(SelectionPointRow{points.at(i), i});
        }
        if (sortColumn_ >= 0) {
            sortRows();
        }
        roiStart_ = rows_.isEmpty() ? 0 : std::clamp(previousRoiStart, 0, int(rows_.size()) - 1);
        endResetModel();
    }

    void clear()
    {
        beginResetModel();
        rows_.clear();
        roiStart_ = 0;
        endResetModel();
    }

    void setRoiStart(int row)
    {
        if (rows_.isEmpty()) {
            return;
        }
        const int clamped = std::clamp(row, 0, int(rows_.size()) - 1);
        if (clamped == roiStart_) {
            return;
        }
        const int oldStart = roiStart_;
        const int oldEnd = roiEnd();
        roiStart_ = clamped;
        emitRangeChanged(oldStart, oldEnd);
        emitRangeChanged(roiStart_, roiEnd());
    }

    int roiStart() const { return roiStart_; }
    int roiEnd() const { return std::min(int(rows_.size()), roiStart_ + kSelectionRoiRows); }
    int totalCount() const { return int(rows_.size()); }

private:
    void sortRows()
    {
        const int column = sortColumn_;
        const Qt::SortOrder order = sortOrder_;
        std::stable_sort(rows_.begin(), rows_.end(), [column, order](const SelectionPointRow& a, const SelectionPointRow& b) {
            int comparison = 0;
            switch (column) {
            case 0: comparison = compare(a.originalIndex, b.originalIndex); break;
            case 1: comparison = compare(a.point.x, b.point.x); break;
            case 2: comparison = compare(a.point.y, b.point.y); break;
            case 3: comparison = compare(a.point.z, b.point.z); break;
            case 4: comparison = compare(a.point.reflectivity, b.point.reflectivity); break;
            case 5: comparison = compare(a.point.tag, b.point.tag); break;
            default: comparison = compare(a.originalIndex, b.originalIndex); break;
            }
            return order == Qt::AscendingOrder ? comparison < 0 : comparison > 0;
        });
    }

    template <typename T>
    static int compare(const T& a, const T& b)
    {
        if (a < b) return -1;
        if (b < a) return 1;
        return 0;
    }

    void emitRangeChanged(int start, int end)
    {
        if (start >= end || rows_.isEmpty()) {
            return;
        }
        const QModelIndex topLeft = index(start, 0);
        const QModelIndex bottomRight = index(end - 1, columnCount() - 1);
        emit dataChanged(topLeft, bottomRight, {Qt::DisplayRole});
    }

    QVector<SelectionPointRow> rows_;
    int roiStart_ = 0;
    int sortColumn_ = -1;
    Qt::SortOrder sortOrder_ = Qt::AscendingOrder;
};

SelectionPointTableModel* ensureSelectionModel(QTableView* table)
{
    if (!table) {
        return nullptr;
    }

    if (auto* model = dynamic_cast<SelectionPointTableModel*>(table->model())) {
        return model;
    }

    auto* model = new SelectionPointTableModel(table);
    table->setModel(model);
    table->setSortingEnabled(true);
    table->horizontalHeader()->setSortIndicatorShown(true);
    table->horizontalHeader()->setSectionsClickable(true);
    QObject::connect(table, &QTableView::pressed, table, [model](const QModelIndex& index) {
        if (index.isValid()) {
            model->setRoiStart(index.row());
        }
    });
    return model;
}

} // namespace

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
    QTableView* table = attrTable ? attrTable : selectionTable;
    SelectionPointTableModel* model = ensureSelectionModel(table);
    if (!model || !pointCloudView) {
        return;
    }

    QVector<PointCloudPoint> pts;
    if (pointCloudView->hasSelectionAabb()) {
        pts = pointCloudView->pointsInPersistSelection(20000000);
    } else {
        QRect sel = pointCloudView->currentSelectionRect();
        if (!sel.isEmpty()) {
            pts = pointCloudView->pointsInRect(sel, 20000000);
        }
    }

    if (!pts.isEmpty()) {
        int zeroCount = 0;
        for (const PointCloudPoint& p : pts) {
            if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) {
                ++zeroCount;
            }
        }
        const int totalCount = pts.size();
        const int validCount = totalCount - zeroCount;

        if (totalCount != lastSelectionCount) {
            lastSelectionCount = totalCount;
            logMessage(QString("框选点云个数: %1, 零点个数: %2, 总数: %3")
                       .arg(validCount).arg(zeroCount).arg(totalCount));
        }

        model->setPoints(std::move(pts));
    } else {
        if (lastSelectionCount != -1) {
            lastSelectionCount = -1;
            logMessage("已清除框选");
        }
        model->clear();
    }
}

void LivoxViewerWindow::onSelectionFinished()
{
    if (!pointCloudView || !(attrTable || selectionTable)) return;
    updateSelectionTableAndLog();
}
