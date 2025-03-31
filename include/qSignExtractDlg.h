#pragma once
#pragma execution_character_set("utf-8")


#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolButton>
#include <QGroupBox>
#include <QListWidget>
#include <QFrame>
#include <QSplitter>
#include <QLabel>
#include <QSettings>
#include <QMainWindow>

#include "ccGLWindow.h"
#include "ccGLWindowSignalEmitter.h"
#include "ccMainAppInterface.h"
#include "ccBox.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"

class qSignExtractDlg : public QDialog
{
    Q_OBJECT

public:
    qSignExtractDlg(ccMainAppInterface* app)
        : QDialog(app ? app->getMainWindow() : nullptr, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
        , m_app(app)
        , m_glWindow(nullptr)
    {
        setWindowTitle("·�����ʶ��");
        resize(1200, 800);

        QHBoxLayout* mainLayout = new QHBoxLayout(this);

        // ����������
        QVBoxLayout* leftLayout = new QVBoxLayout();

        // Ŀ¼�����Ӷ���
        QGroupBox* objectGroup = new QGroupBox("����Ŀ¼");
        QVBoxLayout* objectLayout = new QVBoxLayout();
        m_objectList = new QListWidget();
        objectLayout->addWidget(m_objectList);
        objectGroup->setLayout(objectLayout);
        leftLayout->addWidget(objectGroup);

        // ������ͼ��ť��
        QGroupBox* viewGroup = new QGroupBox("��ͼ�л�");
        QHBoxLayout* viewLayout = new QHBoxLayout();
        addViewButton(viewLayout, "ǰ��ͼ", CC_FRONT_VIEW);
        addViewButton(viewLayout, "����ͼ", CC_LEFT_VIEW);
        addViewButton(viewLayout, "����ͼ", CC_RIGHT_VIEW);
        addViewButton(viewLayout, "����ͼ", CC_TOP_VIEW);
        addViewButton(viewLayout, "����ͼ", CC_BACK_VIEW);
        addViewButton(viewLayout, "�ײ���ͼ", CC_BOTTOM_VIEW);
       
        viewGroup->setLayout(viewLayout);
        leftLayout->addWidget(viewGroup);

        // ���ܰ�ť��
        QGroupBox* functionGroup = new QGroupBox("����ѡ��");
        QVBoxLayout* functionLayout = new QVBoxLayout();
        addFunctionButton(functionLayout, "ȫ�Զ���ȡ", SLOT(onAutoExtract()));
        addFunctionButton(functionLayout, "��ѡ��ȡ", SLOT(onBoxSelectExtract()));
        addFunctionButton(functionLayout, "��ѡ������ȡ", SLOT(onPointGrowExtract()));
        addFunctionButton(functionLayout, "��ѡ��ȡ����", SLOT(onBoxClip()));
        functionGroup->setLayout(functionLayout);
        leftLayout->addWidget(functionGroup);

        leftLayout->addStretch();
        mainLayout->addLayout(leftLayout, 2);

        // �Ҳ�GL��������
        QFrame* glFrame = new QFrame(this);
        glFrame->setFrameStyle(QFrame::Box);
        QVBoxLayout* glLayout = new QVBoxLayout(glFrame);
        QWidget* glWidget = nullptr;
        m_app->createGLWindow(m_glWindow, glWidget);
        glLayout->addWidget(glWidget);
        glFrame->setLayout(glLayout);
        mainLayout->addWidget(glFrame, 8);

        setLayout(mainLayout);

        // ��ʼ����������
        if (m_glWindow)
        {
            m_glWindow->setPerspectiveState(false, true);
            m_glWindow->displayOverlayEntities(true, true);
            m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA);
            m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);
        }
    }

	bool setCloud(ccPointCloud* cloud)
	{
		if (!cloud || !m_glWindow || !m_app)
			return false;

		if (cloud->size() < 10)
		{
			ccLog::Error("��������̫�٣�");
			return false;
		}

		// ���û����ɫ��������Ӱ�ɫ��� scalar field ת��
		if (!cloud->hasColors())
		{
			bool success = false;
			if (cloud->hasDisplayedScalarField())
				success = cloud->convertCurrentScalarFieldToColors();
			else
				success = cloud->setColor(ccColor::white);

			if (!success)
			{
				ccLog::Error("��ɫ����ʧ�ܣ�");
				return false;
			}
		}

		// תΪ�Ҷ���ʾ��ͳһ��ʾ���
		cloud->convertRGBToGreyScale();

		// ��ʾ����
		cloud->setEnabled(true);
		cloud->setVisible(true);
		cloud->setSelected(false);
		cloud->showColors(true);
		cloud->showSF(false);

		// ���뵽 GL ����
		m_glWindow->addToOwnDB(cloud);

		// �Զ��ӽ����ŵ�����
		ccBBox bbox = cloud->getOwnBB();
		m_glWindow->updateConstellationCenterAndZoom(&bbox);

		m_glWindow->redraw();

		return true;
	}


private:
    ccMainAppInterface* m_app;
    ccGLWindowInterface* m_glWindow;
    QListWidget* m_objectList;

    void addViewButton(QHBoxLayout* layout, const QString& name, CC_VIEW_ORIENTATION view)
    {
        QToolButton* btn = new QToolButton(this);
        btn->setText(name);
        layout->addWidget(btn);
        connect(btn, &QToolButton::clicked, [=]() {
            if (m_glWindow)
                m_glWindow->setView(view);
            });
    }

    void addFunctionButton(QVBoxLayout* layout, const QString& name, const char* slot)
    {
        QPushButton* btn = new QPushButton(name, this);
        layout->addWidget(btn);
        connect(btn, SIGNAL(clicked()), this, slot);
    }

private slots:
    void onAutoExtract() { /* ȫ�Զ���ȡ�߼� */ }
    void onBoxSelectExtract() { /* ��ѡ��ȡ�߼� */ }
    void onPointGrowExtract() { /* ��������ȡ�߼� */ }
    void onBoxClip() { /* ��ѡ��ȡ�����߼� */ }
};
