/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout;
    QSplitter *splitter;
    QFrame *beta_frame;
    QFrame *control_frame;
    QLabel *top_camera_label;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(769, 443);
        verticalLayout = new QVBoxLayout(guiDlg);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        splitter = new QSplitter(guiDlg);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        beta_frame = new QFrame(splitter);
        beta_frame->setObjectName(QString::fromUtf8("beta_frame"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(beta_frame->sizePolicy().hasHeightForWidth());
        beta_frame->setSizePolicy(sizePolicy);
        beta_frame->setMinimumSize(QSize(400, 0));
        beta_frame->setFrameShape(QFrame::StyledPanel);
        beta_frame->setFrameShadow(QFrame::Raised);
        splitter->addWidget(beta_frame);
        control_frame = new QFrame(splitter);
        control_frame->setObjectName(QString::fromUtf8("control_frame"));
        sizePolicy.setHeightForWidth(control_frame->sizePolicy().hasHeightForWidth());
        control_frame->setSizePolicy(sizePolicy);
        control_frame->setMinimumSize(QSize(200, 0));
        control_frame->setFrameShape(QFrame::StyledPanel);
        control_frame->setFrameShadow(QFrame::Raised);
        top_camera_label = new QLabel(control_frame);
        top_camera_label->setObjectName(QString::fromUtf8("top_camera_label"));
        top_camera_label->setGeometry(QRect(11, 11, 231, 200));
        top_camera_label->setMinimumSize(QSize(0, 200));
        splitter->addWidget(control_frame);

        verticalLayout->addWidget(splitter);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "giraff_viewer", nullptr));
        top_camera_label->setText(QApplication::translate("guiDlg", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
