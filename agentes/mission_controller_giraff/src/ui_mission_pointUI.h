/********************************************************************************
** Form generated from reading UI file 'mission_pointUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MISSION_POINTUI_H
#define UI_MISSION_POINTUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_point_guiDlg
{
public:
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QLabel *labelX;
    QSpinBox *coordY;
    QLabel *labelY;
    QSpinBox *coordX;
    QPushButton *pushButton_save_coords;

    void setupUi(QWidget *point_guiDlg)
    {
        if (point_guiDlg->objectName().isEmpty())
            point_guiDlg->setObjectName(QString::fromUtf8("point_guiDlg"));
        point_guiDlg->resize(846, 192);
        widget = new QWidget(point_guiDlg);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(170, 86, 535, 28));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        labelX = new QLabel(widget);
        labelX->setObjectName(QString::fromUtf8("labelX"));

        horizontalLayout->addWidget(labelX);

        coordY = new QSpinBox(widget);
        coordY->setObjectName(QString::fromUtf8("coordY"));
        coordY->setMinimum(-1750);
        coordY->setMaximum(1750);

        horizontalLayout->addWidget(coordY);

        labelY = new QLabel(widget);
        labelY->setObjectName(QString::fromUtf8("labelY"));

        horizontalLayout->addWidget(labelY);

        coordX = new QSpinBox(widget);
        coordX->setObjectName(QString::fromUtf8("coordX"));
        coordX->setMinimum(-1900);
        coordX->setMaximum(1900);

        horizontalLayout->addWidget(coordX);

        pushButton_save_coords = new QPushButton(widget);
        pushButton_save_coords->setObjectName(QString::fromUtf8("pushButton_save_coords"));

        horizontalLayout->addWidget(pushButton_save_coords);


        retranslateUi(point_guiDlg);

        QMetaObject::connectSlotsByName(point_guiDlg);
    } // setupUi

    void retranslateUi(QWidget *point_guiDlg)
    {
        point_guiDlg->setWindowTitle(QApplication::translate("point_guiDlg", "Form", nullptr));
        labelX->setText(QApplication::translate("point_guiDlg", "Coord X:", nullptr));
        labelY->setText(QApplication::translate("point_guiDlg", "Coord Y:", nullptr));
        pushButton_save_coords->setText(QApplication::translate("point_guiDlg", "Save coords", nullptr));
    } // retranslateUi

};

namespace Ui {
    class point_guiDlg: public Ui_point_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MISSION_POINTUI_H
