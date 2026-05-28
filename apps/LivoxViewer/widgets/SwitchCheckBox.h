#ifndef LIVOXVIEWER_WIDGETS_SWITCHCHECKBOX_H
#define LIVOXVIEWER_WIDGETS_SWITCHCHECKBOX_H

#include <QCheckBox>
#include <QPainter>

class SwitchCheckBox : public QCheckBox
{
public:
    explicit SwitchCheckBox(QWidget* parent = nullptr)
        : QCheckBox(parent)
    {
        setCursor(Qt::PointingHandCursor);
        setFixedSize(sizeHint());
    }

    QSize sizeHint() const override
    {
        return QSize(44, 24);
    }

protected:
    bool hitButton(const QPoint& pos) const override
    {
        return rect().contains(pos);
    }

    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        const QRectF trackRect(1.0, 3.0, 42.0, 18.0);
        const QColor trackColor = isChecked()
            ? palette().color(QPalette::Highlight)
            : palette().color(QPalette::Mid);
        painter.setPen(Qt::NoPen);
        painter.setBrush(trackColor);
        painter.drawRoundedRect(trackRect, 9.0, 9.0);

        const qreal knobDiameter = 16.0;
        const qreal knobX = isChecked()
            ? trackRect.right() - knobDiameter - 1.0
            : trackRect.left() + 1.0;
        QRectF knobRect(knobX, trackRect.top() + 1.0, knobDiameter, knobDiameter);
        painter.setBrush(palette().color(QPalette::Base));
        painter.drawEllipse(knobRect);
    }
};

#endif // LIVOXVIEWER_WIDGETS_SWITCHCHECKBOX_H
