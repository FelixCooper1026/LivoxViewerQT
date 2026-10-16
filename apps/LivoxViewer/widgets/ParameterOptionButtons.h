#ifndef LIVOXVIEWER_WIDGETS_PARAMETEROPTIONBUTTONS_H
#define LIVOXVIEWER_WIDGETS_PARAMETEROPTIONBUTTONS_H

#include <QAbstractButton>
#include <QButtonGroup>
#include <QGridLayout>
#include <QString>
#include <QWidget>
#include <algorithm>

namespace ParameterOptionButtons {

inline constexpr const char* kControlProperty = "parameterOptionButtons";

inline QString groupObjectName()
{
    return QStringLiteral("parameterOptionButtonGroup");
}

inline bool isOptionControl(const QWidget* control)
{
    return control && control->property(kControlProperty).toBool();
}

inline QButtonGroup* buttonGroup(QWidget* control)
{
    return control->findChild<QButtonGroup*>(groupObjectName());
}

inline int currentIndex(QWidget* control)
{
    return buttonGroup(control)->checkedId();
}

inline QString currentText(QWidget* control)
{
    QButtonGroup* group = buttonGroup(control);
    return group->button(group->checkedId())->text();
}

inline void setCurrentIndex(QWidget* control, int index)
{
    QButtonGroup* group = buttonGroup(control);
    if (group->checkedId() == index) return;
    group->setExclusive(false);
    for (QAbstractButton* button : group->buttons()) {
        button->setChecked(false);
    }
    group->setExclusive(true);
    if (QAbstractButton* button = group->button(index)) {
        button->setChecked(true);
    }
}

inline void setSignalsBlocked(QWidget* control, bool blocked)
{
    buttonGroup(control)->blockSignals(blocked);
}

inline void relayoutVisibleButtons(QWidget* control)
{
    auto* layout = static_cast<QGridLayout*>(control->layout());
    auto buttons = buttonGroup(control)->buttons();
    std::sort(buttons.begin(), buttons.end(), [](QAbstractButton* left, QAbstractButton* right) {
        return left->property("parameterOptionOrder").toInt() < right->property("parameterOptionOrder").toInt();
    });
    while (QLayoutItem* item = layout->takeAt(0)) delete item;
    int index = 0;
    for (QAbstractButton* button : buttons) {
        if (button->isHidden()) continue;
        layout->addWidget(button, index / 2, index % 2);
        ++index;
    }
    layout->invalidate();
    control->updateGeometry();
}

} // namespace ParameterOptionButtons

#endif // LIVOXVIEWER_WIDGETS_PARAMETEROPTIONBUTTONS_H
