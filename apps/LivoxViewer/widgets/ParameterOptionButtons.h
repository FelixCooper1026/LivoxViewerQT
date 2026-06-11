#ifndef LIVOXVIEWER_WIDGETS_PARAMETEROPTIONBUTTONS_H
#define LIVOXVIEWER_WIDGETS_PARAMETEROPTIONBUTTONS_H

#include <QAbstractButton>
#include <QButtonGroup>
#include <QString>
#include <QWidget>

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
    if (QAbstractButton* button = buttonGroup(control)->button(index)) {
        button->setChecked(true);
    }
}

inline void setSignalsBlocked(QWidget* control, bool blocked)
{
    buttonGroup(control)->blockSignals(blocked);
}

} // namespace ParameterOptionButtons

#endif // LIVOXVIEWER_WIDGETS_PARAMETEROPTIONBUTTONS_H
