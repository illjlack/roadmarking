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
        setWindowTitle("路标点云识别");
        resize(1200, 800);

        QHBoxLayout* mainLayout = new QHBoxLayout(this);

        // 左侧控制区域
        QVBoxLayout* leftLayout = new QVBoxLayout();

        // 目录管理子对象
        QGroupBox* objectGroup = new QGroupBox("对象目录");
        QVBoxLayout* objectLayout = new QVBoxLayout();
        m_objectList = new QListWidget();
        objectLayout->addWidget(m_objectList);
        objectGroup->setLayout(objectLayout);
        leftLayout->addWidget(objectGroup);

        // 正交视图按钮组
        QGroupBox* viewGroup = new QGroupBox("视图切换");
        QHBoxLayout* viewLayout = new QHBoxLayout();
        addViewButton(viewLayout, "前视图", CC_FRONT_VIEW);
        addViewButton(viewLayout, "左视图", CC_LEFT_VIEW);
        addViewButton(viewLayout, "右视图", CC_RIGHT_VIEW);
        addViewButton(viewLayout, "顶视图", CC_TOP_VIEW);
        addViewButton(viewLayout, "后视图", CC_BACK_VIEW);
        addViewButton(viewLayout, "底部视图", CC_BOTTOM_VIEW);
       
        viewGroup->setLayout(viewLayout);
        leftLayout->addWidget(viewGroup);

        // 功能按钮组
        QGroupBox* functionGroup = new QGroupBox("功能选择");
        QVBoxLayout* functionLayout = new QVBoxLayout();
        addFunctionButton(functionLayout, "全自动提取", SLOT(onAutoExtract()));
        addFunctionButton(functionLayout, "框选提取", SLOT(onBoxSelectExtract()));
        addFunctionButton(functionLayout, "点选生长提取", SLOT(onPointGrowExtract()));
        addFunctionButton(functionLayout, "框选截取点云", SLOT(onBoxClip()));
        functionGroup->setLayout(functionLayout);
        leftLayout->addWidget(functionGroup);

        leftLayout->addStretch();
        mainLayout->addLayout(leftLayout, 2);

        // 右侧GL窗口区域
        QFrame* glFrame = new QFrame(this);
        glFrame->setFrameStyle(QFrame::Box);
        QVBoxLayout* glLayout = new QVBoxLayout(glFrame);
        QWidget* glWidget = nullptr;
        m_app->createGLWindow(m_glWindow, glWidget);
        glLayout->addWidget(glWidget);
        glFrame->setLayout(glLayout);
        mainLayout->addWidget(glFrame, 8);

        setLayout(mainLayout);

        // 初始化窗口属性
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
			ccLog::Error("点云数据太少！");
			return false;
		}

		// 如果没有颜色，尝试添加白色或从 scalar field 转换
		if (!cloud->hasColors())
		{
			bool success = false;
			if (cloud->hasDisplayedScalarField())
				success = cloud->convertCurrentScalarFieldToColors();
			else
				success = cloud->setColor(ccColor::white);

			if (!success)
			{
				ccLog::Error("颜色处理失败！");
				return false;
			}
		}

		// 转为灰度显示（统一显示风格）
		cloud->convertRGBToGreyScale();

		// 显示设置
		cloud->setEnabled(true);
		cloud->setVisible(true);
		cloud->setSelected(false);
		cloud->showColors(true);
		cloud->showSF(false);

		// 加入到 GL 窗口
		m_glWindow->addToOwnDB(cloud);

		// 自动视角缩放到点云
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
    void onAutoExtract() { /* 全自动提取逻辑 */ }
    void onBoxSelectExtract() { /* 框选提取逻辑 */ }
    void onPointGrowExtract() { /* 点生长提取逻辑 */ }
    void onBoxClip() { /* 框选截取点云逻辑 */ }
};
