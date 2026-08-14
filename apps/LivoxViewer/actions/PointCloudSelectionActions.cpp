#include "LivoxViewerWindow.h"

#include <QAbstractTableModel>
#include <QHeaderView>
#include <QPointer>
#include <QTableView>
#include <QtConcurrent/QtConcurrentRun>

#include <algorithm>

namespace {

constexpr int kSelectionRoiRows = 500;

enum SelectionPointColumn {
    ColumnId = 0,
    ColumnCoordA,
    ColumnCoordB,
    ColumnCoordC,
    ColumnReflectivity,
    ColumnTag,
    ColumnLine,
    ColumnCount
};

struct SelectionPointRow
{
    PointCloudPoint point;
    int originalIndex = 0;
};

struct SelectionTableBuildResult
{
    QVector<SelectionPointRow> rows;
    bool useSphericalColumns = false;
    int zeroPointCount = 0;
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
        return parent.isValid() ? 0 : ColumnCount;
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override
    {
        if (role != Qt::DisplayRole) {
            return {};
        }
        if (orientation == Qt::Horizontal) {
            if (useSphericalColumns_) {
                static const QStringList headers = {"ID", "Theta", "Phi", "Depth", "Refl", "Tag", "Line"};
                return headers.value(section);
            }
            static const QStringList headers = {"ID", "X(m)", "Y(m)", "Z(m)", "Refl", "Tag", "Line"};
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

        if (roiEnabled() && (index.row() < roiStart_ || index.row() >= roiEnd())) {
            return QStringLiteral("...");
        }

        const SelectionPointRow& row = rows_.at(index.row());
        const PointCloudPoint& p = row.point;
        switch (index.column()) {
        case ColumnId: return row.originalIndex + 1;
        case ColumnCoordA: return QString::number(useSphericalColumns_ ? p.theta : p.x, 'f', 3);
        case ColumnCoordB: return QString::number(useSphericalColumns_ ? p.phi : p.y, 'f', 3);
        case ColumnCoordC: return QString::number(useSphericalColumns_ ? p.depth : p.z, 'f', 3);
        case ColumnReflectivity: return int(p.reflectivity);
        case ColumnTag: return int(p.tag);
        case ColumnLine: return int(p.line);
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
        QVector<SelectionPointRow> rows;
        rows.reserve(points.size());
        for (int i = 0; i < points.size(); ++i) {
            rows.push_back(SelectionPointRow{points.at(i), i});
        }
        setRows(std::move(rows), !points.isEmpty() && points.first().spherical);
    }

    void setRows(QVector<SelectionPointRow>&& rows, bool useSphericalColumns)
    {
        beginResetModel();
        const int previousRoiStart = roiStart_;
        useSphericalColumns_ = useSphericalColumns;
        rows_ = std::move(rows);
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
        useSphericalColumns_ = false;
        roiStart_ = 0;
        endResetModel();
    }

    void setRoiStart(int row)
    {
        if (!roiEnabled()) {
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
    bool roiEnabled() const { return rows_.size() > kSelectionRoiRows; }

private:
    void sortRows()
    {
        const int column = sortColumn_;
        const Qt::SortOrder order = sortOrder_;
        const bool useSphericalColumns = useSphericalColumns_;
        std::stable_sort(rows_.begin(), rows_.end(), [column, order, useSphericalColumns](const SelectionPointRow& a, const SelectionPointRow& b) {
            int comparison = 0;
            switch (column) {
            case ColumnId: comparison = compare(a.originalIndex, b.originalIndex); break;
            case ColumnCoordA: comparison = useSphericalColumns ? compare(a.point.theta, b.point.theta) : compare(a.point.x, b.point.x); break;
            case ColumnCoordB: comparison = useSphericalColumns ? compare(a.point.phi, b.point.phi) : compare(a.point.y, b.point.y); break;
            case ColumnCoordC: comparison = useSphericalColumns ? compare(a.point.depth, b.point.depth) : compare(a.point.z, b.point.z); break;
            case ColumnReflectivity: comparison = compare(a.point.reflectivity, b.point.reflectivity); break;
            case ColumnTag: comparison = compare(a.point.tag, b.point.tag); break;
            case ColumnLine: comparison = compare(a.point.line, b.point.line); break;
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
    bool useSphericalColumns_ = false;
    Qt::SortOrder sortOrder_ = Qt::AscendingOrder;
};

void configureSelectionTable(QTableView* table)
{
    QHeaderView* header = table->horizontalHeader();
    header->setSortIndicatorShown(true);
    header->setSectionsClickable(true);
    header->setStretchLastSection(false);
    header->setMinimumSectionSize(52);
    header->setSectionResizeMode(ColumnId, QHeaderView::ResizeToContents);
    header->setSectionResizeMode(ColumnCoordA, QHeaderView::Stretch);
    header->setSectionResizeMode(ColumnCoordB, QHeaderView::Stretch);
    header->setSectionResizeMode(ColumnCoordC, QHeaderView::Stretch);
    header->setSectionResizeMode(ColumnReflectivity, QHeaderView::Stretch);
    header->setSectionResizeMode(ColumnTag, QHeaderView::Stretch);
    header->setSectionResizeMode(ColumnLine, QHeaderView::Stretch);
}

SelectionPointTableModel* ensureSelectionModel(QTableView* table)
{
    if (!table) {
        return nullptr;
    }

    if (auto* model = dynamic_cast<SelectionPointTableModel*>(table->model())) {
        configureSelectionTable(table);
        return model;
    }

    auto* model = new SelectionPointTableModel(table);
    table->setModel(model);
    table->setSortingEnabled(true);
    configureSelectionTable(table);
    QObject::connect(table, &QTableView::pressed, table, [model](const QModelIndex& index) {
        if (index.isValid()) {
            model->setRoiStart(index.row());
        }
    });
    return model;
}

SelectionTableBuildResult buildSelectionTableRows(QVector<PointCloudPoint>&& points, int zeroPointCount)
{
    SelectionTableBuildResult result;
    result.zeroPointCount = zeroPointCount;
    result.useSphericalColumns = !points.isEmpty() && points.first().spherical;
    result.rows.reserve(points.size());
    for (int i = 0; i < points.size(); ++i) {
        result.rows.push_back(SelectionPointRow{points.at(i), i});
    }
    return result;
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

void LivoxViewerWindow::clearSelectionTableAndSummary()
{
    QTableView* table = attrTable ? attrTable : selectionTable;
    if (SelectionPointTableModel* model = ensureSelectionModel(table)) {
        model->clear();
    }
    lastSelectionCount = -1;
    if (selectionSummaryLabel) {
        selectionSummaryLabel->setText(QStringLiteral("未框选点云"));
    }
}

void LivoxViewerWindow::setPointCloudSelectionEnabled(bool enabled)
{
    enabled = enabled && pointCloudView;
    const bool wasEnabled = pointCloudToolMode == PointCloudToolMode::Selection;

    if (enabled && !wasEnabled) {
        selectionRestoreRightPanelVisible = (paramsDock && paramsDock->isVisible()) ||
                                            (attrDock && attrDock->isVisible());
        selectionRestoreParamsDockVisible = selectionRestoreRightPanelVisible
            ? paramsDock->isVisible()
            : restoreRightParamsDock;
        selectionRestoreAttrDockVisible = selectionRestoreRightPanelVisible
            ? attrDock->isVisible()
            : restoreRightAttrDock;
        selectionRestoreActiveRightDock = activeRightDock;
        selectionRightDockStateSaved = true;
    }

    ++selectionTableGeneration;
    pointCloudToolMode = enabled ? PointCloudToolMode::Selection : PointCloudToolMode::None;
    forEachPointCloudView([enabled](PointCloudView* view) {
        view->setSelectionModeEnabled(enabled);
    });

    if (enabled) {
        if (attrDock) {
            attrDock->show();
            attrDock->raise();
            activeRightDock = attrDock;
        }
        updateSelectionTableAndLog();
        return;
    }

    clearSelectionTableAndSummary();
    if (!wasEnabled || !selectionRightDockStateSaved) {
        return;
    }

    const bool rightPanelStillVisible = paramsDock->isVisible() || attrDock->isVisible();
    const bool restorePanelNow = selectionRestoreRightPanelVisible && rightPanelStillVisible;
    restoreRightParamsDock = selectionRestoreParamsDockVisible;
    restoreRightAttrDock = selectionRestoreAttrDockVisible;
    paramsDock->setVisible(restorePanelNow && selectionRestoreParamsDockVisible);
    attrDock->setVisible(restorePanelNow && selectionRestoreAttrDockVisible);
    if (!restorePanelNow) {
        activeRightDock = selectionRestoreActiveRightDock;
    } else if (selectionRestoreActiveRightDock == attrDock && selectionRestoreAttrDockVisible) {
        attrDock->raise();
        activeRightDock = attrDock;
    } else if (selectionRestoreActiveRightDock == paramsDock && selectionRestoreParamsDockVisible) {
        paramsDock->raise();
        activeRightDock = paramsDock;
    } else if (selectionRestoreParamsDockVisible) {
        paramsDock->raise();
        activeRightDock = paramsDock;
    } else if (selectionRestoreAttrDockVisible) {
        attrDock->raise();
        activeRightDock = attrDock;
    } else {
        activeRightDock = nullptr;
    }
    selectionRightDockStateSaved = false;
    selectionRestoreRightPanelVisible = false;
    selectionRestoreActiveRightDock.clear();
}

void LivoxViewerWindow::updateSelectionTableAndLog()
{
    QTableView* table = attrTable ? attrTable : selectionTable;
    SelectionPointTableModel* model = ensureSelectionModel(table);
    if (!model || !pointCloudView) {
        return;
    }

    if (!pointCloudView->hasSelectionAabb()) {
        lastSelectionCount = -1;
        if (selectionSummaryLabel) {
            selectionSummaryLabel->setText("未框选点云");
        }
        model->clear();
        return;
    }

    if (selectionSummaryLabel) {
        selectionSummaryLabel->setText("框选点云计算中...");
    }
    pointCloudView->requestSelectionUpdate();
}

void LivoxViewerWindow::onSelectionPointsReady(QVector<PointCloudPoint> points, int zeroPointCount)
{
    PointCloudView* sourceView = qobject_cast<PointCloudView*>(sender());
    if (sourceView && sourceView != pointCloudView) {
        return;
    }
    if (pointCloudToolMode != PointCloudToolMode::Selection) {
        return;
    }

    QTableView* table = attrTable ? attrTable : selectionTable;
    SelectionPointTableModel* model = ensureSelectionModel(table);
    if (!model || !pointCloudView) {
        return;
    }

    const QPointer<PointCloudView> selectionSource(sourceView ? sourceView : pointCloudView);
    const quint64 generation = ++selectionTableGeneration;
    if (points.isEmpty()) {
        lastSelectionCount = -1;
        if (selectionSummaryLabel) {
            selectionSummaryLabel->setText("未框选点云");
        }
        model->clear();
        pointCloudView->update();
        return;
    }

    const int totalCount = points.size();
    const int validCount = totalCount - zeroPointCount;
    lastSelectionCount = totalCount;
    if (selectionSummaryLabel) {
        selectionSummaryLabel->setText(QString("框选点云个数: %1\n零点个数: %2\n总数: %3")
                                       .arg(validCount)
                                       .arg(zeroPointCount)
                                       .arg(totalCount));
    }

    const QPointer<LivoxViewerWindow> guard(this);
    QtConcurrent::run([guard, selectionSource, generation, points = std::move(points), zeroPointCount]() mutable {
        SelectionTableBuildResult result = buildSelectionTableRows(std::move(points), zeroPointCount);
        if (!guard) {
            return;
        }
        QMetaObject::invokeMethod(guard.data(), [guard, selectionSource, generation, result = std::move(result)]() mutable {
            if (!guard || !selectionSource ||
                selectionSource != guard->pointCloudView ||
                guard->pointCloudToolMode != PointCloudToolMode::Selection ||
                generation != guard->selectionTableGeneration) {
                return;
            }
            LivoxViewerWindow* window = guard.data();
            QTableView* table = window->attrTable ? window->attrTable : window->selectionTable;
            SelectionPointTableModel* model = ensureSelectionModel(table);
            if (!model || !window->pointCloudView) {
                return;
            }

            model->setRows(std::move(result.rows), result.useSphericalColumns);
            window->pointCloudView->update();
        }, Qt::QueuedConnection);
    });
}

void LivoxViewerWindow::onSelectionFinished()
{
    if (!pointCloudView || !(attrTable || selectionTable)) return;
    updateSelectionTableAndLog();
}
