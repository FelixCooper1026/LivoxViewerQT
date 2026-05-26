#include "LivoxViewerWindow.h"
#include <QApplication>
#include <QHeaderView>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QStyledItemDelegate>
#include <QStyleOptionButton>

#include <algorithm>

namespace {

class CenteredCheckBoxDelegate : public QStyledItemDelegate
{
public:
    using QStyledItemDelegate::QStyledItemDelegate;

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override
    {
        if (index.data(Qt::CheckStateRole).isValid()) {
            QStyleOptionButton checkOption;
            checkOption.state = QStyle::State_Enabled |
                (index.data(Qt::CheckStateRole).toInt() == Qt::Checked ? QStyle::State_On : QStyle::State_Off);
            const QRect checkRect = QApplication::style()->subElementRect(QStyle::SE_CheckBoxIndicator, &checkOption);
            checkOption.rect = QRect(option.rect.center().x() - checkRect.width() / 2,
                                     option.rect.center().y() - checkRect.height() / 2,
                                     checkRect.width(),
                                     checkRect.height());
            QApplication::style()->drawControl(QStyle::CE_CheckBox, &checkOption, painter);
            return;
        }
        QStyledItemDelegate::paint(painter, option, index);
    }

    bool editorEvent(QEvent* event,
                     QAbstractItemModel* model,
                     const QStyleOptionViewItem& option,
                     const QModelIndex& index) override
    {
        if (!index.data(Qt::CheckStateRole).isValid()) {
            return QStyledItemDelegate::editorEvent(event, model, option, index);
        }

        bool toggle = false;
        if (event->type() == QEvent::MouseButtonRelease) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            toggle = mouseEvent->button() == Qt::LeftButton && option.rect.contains(mouseEvent->pos());
        } else if (event->type() == QEvent::KeyPress) {
            auto* keyEvent = static_cast<QKeyEvent*>(event);
            toggle = keyEvent->key() == Qt::Key_Space || keyEvent->key() == Qt::Key_Select;
        }

        if (!toggle) {
            return false;
        }

        const Qt::CheckState state =
            index.data(Qt::CheckStateRole).toInt() == Qt::Checked ? Qt::Unchecked : Qt::Checked;
        return model->setData(index, state, Qt::CheckStateRole);
    }
};

class PlaybackDeviceTable : public QTableWidget
{
public:
    explicit PlaybackDeviceTable(QWidget* parent = nullptr)
        : QTableWidget(parent)
    {
        setWordWrap(false);
        setTextElideMode(Qt::ElideRight);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    }

    void updateColumnLayout()
    {
        const int width = viewport()->width();
        setColumnHidden(3, width < 320);
        setColumnHidden(2, width < 240);
        setColumnWidth(0, fontMetrics().horizontalAdvance("显示") + 12);
        setColumnWidth(1, std::max(fontMetrics().horizontalAdvance("Mid360S"),
                                   fontMetrics().horizontalAdvance("型号")) + 12);
        horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
        horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
        horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
        horizontalHeader()->setSectionResizeMode(3, QHeaderView::Interactive);
    }

protected:
    void resizeEvent(QResizeEvent* event) override
    {
        QTableWidget::resizeEvent(event);
        updateColumnLayout();
    }
};

} // namespace

void LivoxViewerWindow::createFileInfoPanel()
{
    // 左侧：文件信息 Dock（播放 LVX2 / Pcap 时显示）
    lvx2FileDock = new QDockWidget(QStringLiteral("文件信息"), this);
    lvx2FileDock->setObjectName("Lvx2FileDock");
    lvx2FileDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* lvx2FileContent = new QWidget(lvx2FileDock);
    QVBoxLayout* lvx2FileLayout = new QVBoxLayout(lvx2FileContent);
    lvx2FileLayout->setContentsMargins(8, 8, 8, 8);
    lvx2FileLayout->setSpacing(6);
    QLabel* lvx2Hint = new QLabel(QStringLiteral("文件设备（勾选可见）"), lvx2FileContent);
    lvx2Hint->setToolTip("离线点云文件中的设备列表；取消勾选可隐藏对应设备点云");
    lvx2FileLayout->addWidget(lvx2Hint);
    lvx2DeviceTable = new PlaybackDeviceTable(lvx2FileContent);
    lvx2DeviceTable->setColumnCount(4);
    lvx2DeviceTable->setHorizontalHeaderLabels({"显示", "型号", "SN", "IP"});
    lvx2DeviceTable->verticalHeader()->setVisible(false);
    lvx2DeviceTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    lvx2DeviceTable->setSelectionMode(QAbstractItemView::NoSelection);
    lvx2DeviceTable->verticalHeader()->setDefaultSectionSize(fontMetrics().height() + 10);
    lvx2DeviceTable->setItemDelegateForColumn(0, new CenteredCheckBoxDelegate(lvx2DeviceTable));
    static_cast<PlaybackDeviceTable*>(lvx2DeviceTable)->updateColumnLayout();
    lvx2FileLayout->addWidget(lvx2DeviceTable);
    lvx2FileContent->setLayout(lvx2FileLayout);
    lvx2FileDock->setWidget(lvx2FileContent);
    lvx2FileDock->setMinimumWidth(0);
    addDockWidget(Qt::LeftDockWidgetArea, lvx2FileDock);
    tabifyDockWidget(lidarDevicesDock, imuDock);
    tabifyDockWidget(lidarDevicesDock, lvx2FileDock);
    lidarDevicesDock->raise();
    lvx2FileDock->hide();
}
