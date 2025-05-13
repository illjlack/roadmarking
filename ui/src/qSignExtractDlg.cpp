#include "qSignExtractDlg.h"

#include <QTreeWidget>
#include <QVBoxLayout>
#include <QContextMenuEvent>
#include <QMenu>
#include <QAction>
#include <QHeaderView>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <ccHObject.h>
#include <ccLog.h>
#include <ccPolyline.h>
#include <QThread>
#include <QApplication>

#include "CloudProcess.h"
#include "RoadMarkingExtract.h"


using namespace roadmarking;

// =========================================================================================================================== qSignExtractDlg
qSignExtractDlg::qSignExtractDlg(ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, m_app(app)
{
	setWindowTitle("路标点云识别");
	resize(1200, 800);
	setFocusPolicy(Qt::StrongFocus);

	QHBoxLayout* mainLayout = new QHBoxLayout(this);
	QVBoxLayout* leftLayout = new QVBoxLayout();

	m_glWidget = nullptr;
	if (m_app)
		m_app->createGLWindow(m_glWindow, m_glWidget);

	// ==================================== 前景绘制器
	m_foregroundPolylineEditor = new ForegroundPolylineEditor(m_glWindow);
	m_foregroundPolylineEditor->setSelectCloudPtr(&p_select_cloud);


	// ==================================== 阈值筛选
	histogramWidget = new ThresholdHistogramWidget(this);
	histogramWidget->setWindowFlag(Qt::Window);
	histogramWidget -> hide();
	connect(histogramWidget, &ThresholdHistogramWidget::addCloudToDB, this, &qSignExtractDlg::addCloudToDB);
	// ==================================== 对象目录
	{
		QGroupBox* objectGroup = new QGroupBox("对象目录", this);
		m_objectTree = new CloudObjectTreeWidget(objectGroup);
		m_objectTree->initialize(m_glWindow, m_app, &p_select_cloud, {});

		QVBoxLayout* objectLayout = new QVBoxLayout(objectGroup);
		objectLayout->addWidget(m_objectTree);
		objectGroup->setLayout(objectLayout);
		leftLayout->addWidget(objectGroup);
	}

	// ==================================== 视图切换
	{
		QGroupBox* viewGroup = new QGroupBox("视图切换", this);
		QHBoxLayout* viewLayout = new QHBoxLayout(viewGroup);  // 设置 parent 为 viewGroup

		auto addViewButton = [&](QHBoxLayout* layout, const QString& name, CC_VIEW_ORIENTATION view)
		{
			QToolButton* btn = new QToolButton(viewGroup);
			btn->setText(name);
			layout->addWidget(btn);
			connect(btn, &QToolButton::clicked, [=]() {
				if (m_glWindow) m_glWindow->setView(view);
				});
		};

		addViewButton(viewLayout, "前视图", CC_FRONT_VIEW);
		addViewButton(viewLayout, "左视图", CC_LEFT_VIEW);
		addViewButton(viewLayout, "右视图", CC_RIGHT_VIEW);
		addViewButton(viewLayout, "顶视图", CC_TOP_VIEW);
		addViewButton(viewLayout, "后视图", CC_BACK_VIEW);
		addViewButton(viewLayout, "底部视图", CC_BOTTOM_VIEW);

		viewGroup->setLayout(viewLayout);
		leftLayout->addWidget(viewGroup);

		QToolButton* btn = new QToolButton(viewGroup);  // 设置 parent 为 viewGroup
		btn->setText("1:1");
		viewLayout->addWidget(btn);
		connect(btn, &QToolButton::clicked, [=]()
			{
				if (p_select_cloud)
				{
					ccBBox bbox = p_select_cloud->getOwnBB();
					m_glWindow->updateConstellationCenterAndZoom(&bbox);
				}
			});

		connect(m_foregroundPolylineEditor, &ForegroundPolylineEditor::draw_finish, [viewGroup]()
			{
				for (int i = 0; i < viewGroup->layout()->count(); ++i) {
					QToolButton* btn = qobject_cast<QToolButton*>(viewGroup->layout()->itemAt(i)->widget());
					if (btn) {
						btn->setEnabled(true);
					}
				}
			});

		connect(m_foregroundPolylineEditor, &ForegroundPolylineEditor::draw_start, [viewGroup]()
			{
				for (int i = 0; i < viewGroup->layout()->count(); ++i) {
					QToolButton* btn = qobject_cast<QToolButton*>(viewGroup->layout()->itemAt(i)->widget());
					if (btn) {
						if (btn->text() == "1:1") {
							continue;
						}
						btn->setEnabled(false);
					}
				}
			});
	}



	// ==================================== 功能按钮组
	{
		QGroupBox* functionGroup = new QGroupBox("功能选择", this);
		QVBoxLayout* functionLayout = new QVBoxLayout(functionGroup);

		QButtonGroup* buttonGroup = new QButtonGroup(this);
		buttonGroup->setExclusive(true);

		auto addCheckableButton = [&](const QString& text, std::function<void()> callback)
		{
			QPushButton* btn = new QPushButton(text, functionGroup);
			btn->setCheckable(true);
			functionLayout->addWidget(btn);
			connect(btn, &QPushButton::clicked, this, std::move(callback));
			buttonGroup->addButton(btn);
		};

		addCheckableButton("选择强度阈值过滤点云", [this]() { onFilteCloudByIntensity(); });
		addCheckableButton("选择高程阈值过滤点云", [this]() { onFilteCloudByZ(); });
		addCheckableButton("全自动提取", [this]() { onAutoExtract(); });
		addCheckableButton("框选提取", [this]() { onBoxSelectExtract(); });
		addCheckableButton("点选生长提取", [this]() { onPointGrowExtract(); });
		addCheckableButton("框选截取点云", [this]() { onBoxClip(); });

		functionGroup->setLayout(functionLayout);
		leftLayout->addWidget(functionGroup);


		connect(m_foregroundPolylineEditor, &ForegroundPolylineEditor::draw_finish, [buttonGroup]()
			{
				for (auto* btn : buttonGroup->buttons())
				{
					btn->setEnabled(true);
				}
			});

		connect(m_foregroundPolylineEditor, &ForegroundPolylineEditor::draw_start, [buttonGroup]()
			{
				for (auto* btn : buttonGroup->buttons())
				{
					btn->setEnabled(false);
				}
			});
	}


	leftLayout->addStretch();
	mainLayout->addLayout(leftLayout, 2);

	// ==================================== GL窗口区域
	{
		QFrame* glFrame = new QFrame(this);
		glFrame->setFrameStyle(QFrame::Box);
		QVBoxLayout* glLayout = new QVBoxLayout(glFrame);

		glLayout->addWidget(m_glWidget);
		glFrame->setLayout(glLayout);
		mainLayout->addWidget(glFrame, 8);
		setLayout(mainLayout);


		/*
		enum PICKING_MODE {
			NO_PICKING,                           // 禁用拾取功能，用户无法选择任何物体
			ENTITY_PICKING,                       // 选择整个实体，允许用户选择如点云、网格等实体
			ENTITY_RECT_PICKING,                  // 矩形区域选择，允许用户通过矩形框选择多个实体
			FAST_PICKING,                         // 快速选择，优化了性能，但精度较低，适用于大数据集
			POINT_PICKING,                        // 选择单个点，允许用户选择点云中的点
			TRIANGLE_PICKING,                     // 选择三角形，允许用户选择网格中的三角形面片
			POINT_OR_TRIANGLE_PICKING,            // 选择点或三角形，允许用户选择点云中的点或网格中的三角形
			POINT_OR_TRIANGLE_OR_LABEL_PICKING,   // 选择点、三角形或标签，允许用户选择点云中的点、网格中的三角形或标签（如点云标注）
			LABEL_PICKING,                        // 选择标签，允许用户选择点云中的标签或其他注释信息
			DEFAULT_PICKING,                      // 默认拾取模式，通常与 `ENTITY_PICKING` 相同，用于选择实体
		};
		*/

		if (m_glWindow)
		{
			m_glWindow->setPerspectiveState(false, true);
			m_glWindow->displayOverlayEntities(true, true);
			m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA);
			m_glWindow->setPickingMode(ccGLWindowInterface::ENTITY_PICKING);
		}
	}

	// ==================================== 一些信号

	connect(m_foregroundPolylineEditor, &ForegroundPolylineEditor::update_tree, m_objectTree, [=]() { m_objectTree->refresh(); });
	connect(m_foregroundPolylineEditor, &ForegroundPolylineEditor::draw_start, [&]() {m_selectionMode = DRAW_SELECTION; });
	connect(m_foregroundPolylineEditor, &ForegroundPolylineEditor::draw_finish, [&](){m_selectionMode = ENTITY_SELECTION;});

	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::entitySelectionChanged, this, &qSignExtractDlg::onEntitySelectionChanged);
	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::itemPicked, this, &qSignExtractDlg::onItemPicked);
	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::itemPickedFast, this, &qSignExtractDlg::onItemPickedFast);

	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::leftButtonClicked, this, &qSignExtractDlg::onLeftButtonClicked);
	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::mouseMoved, this, &qSignExtractDlg::onMouseMoved);
	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::buttonReleased, this, &qSignExtractDlg::onButtonReleased);
	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::mouseWheelRotated, this, &qSignExtractDlg::onMouseWheelRotated);
	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::leftButtonDoubleClicked, this, &qSignExtractDlg::onLeftButtonDoubleClicked);
	connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::rightButtonDoubleClicked, this, &qSignExtractDlg::onRightButtonDoubleClicked);

}
qSignExtractDlg::~qSignExtractDlg()
{
	if (m_glWindow)
	{
		m_objectTree->relase();
		m_glWindow->getOwnDB()->removeAllChildren();

		if (m_app)
		{
			m_app->destroyGLWindow(m_glWindow);
			m_glWindow = nullptr;
		}
	}

	if (m_foregroundPolylineEditor)
	{
		delete m_foregroundPolylineEditor;
	}
}

bool qSignExtractDlg::setCloud(std::vector<ccHObject*>cloud)
{
	if (!cloud.size() || !m_glWindow || !m_app)
		return false;

	m_objectTree->initialize(m_glWindow, m_app, &p_select_cloud, cloud);
	ccBBox bbox = cloud[0]->getOwnBB();
	m_glWindow->updateConstellationCenterAndZoom(&bbox);
	m_glWindow->redraw();

	m_objectTree->refresh();
	return true;
}

void qSignExtractDlg::onAutoExtract()
{
	RoadMarkingExtract::automaticExtraction(PointCloudIO::convert_to_ccCloudPtr(p_select_cloud), m_app);
	m_objectTree->refresh();
}

void qSignExtractDlg::onBoxSelectExtract()
{
	m_foregroundPolylineEditor->startDraw();
	m_foregroundPolylineEditor->setCallbackfunc([=]
		{
			std::vector<CCVector3d> polyline;
			m_foregroundPolylineEditor->getPoints(polyline);
			std::vector <ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			ccPointCloud* cloud = new ccPointCloud;
			CloudProcess::crop_cloud_with_polygon(clouds, polyline, cloud);

			cloud->setName("cloud_cropped");
			m_glWindow->addToOwnDB(cloud);
			m_objectTree->refresh();

			if (p_select_cloud)p_select_cloud->setSelected(false);
			p_select_cloud = cloud;

			RoadMarkingExtract::automaticExtraction(PointCloudIO::convert_to_ccCloudPtr(p_select_cloud), m_app);
			m_objectTree->refresh();
		});
}

void qSignExtractDlg::onPointGrowExtract()
{
	m_foregroundPolylineEditor->startDraw();
	m_foregroundPolylineEditor->setDraw(2, false);
	m_foregroundPolylineEditor->setCallbackfunc([&]
		{
			SettingsDialog settingsDialog;
			settingsDialog.setDescription(
				"参数：\n"
				"调试模式：是否开启调试显示（将线显示在窗口上）\n"
				"是否进行额外的 csf 提取地面：\n"
				"是否拟合直线：\n"
				"是否使用动态矩形：\n"
				"矩形宽度(m)：线生长的宽度\n"
				"生长步长(m)：每次扩展的长度\n"
				"最小点数：每段线段需要包含的最少点数\n"
				"最大弯折角度(度)：线段间允许的最大弯折角度\n"
				"最大跳跃次数：尝试跳跃的最大次数（用于处理断开的线）"
			);

			// 布尔值
			settingsDialog.registerComponent<bool>("开启调试显示", "debugEnabled", false);
			settingsDialog.registerComponent<bool>("开启提取地面", "isGetGround", false);
			settingsDialog.registerComponent<bool>("拟合直线", "doFitLine", false);
			settingsDialog.registerComponent<bool>("使用动态矩形", "useDynamicRect", true);

			// 浮点值
			settingsDialog.registerComponent<float>("矩形宽度", "W", 1.0f);
			settingsDialog.registerComponent<float>("生长步长", "L", 2.0f);
			settingsDialog.registerComponent<float>("最大弯折角度(度)", "theta_max_degrees", 45.0f);

			// 整数值
			settingsDialog.registerComponent<int>("最小点数", "Nmin", 20);
			settingsDialog.registerComponent<int>("最大跳跃次数", "Kmax", 10);

			if (settingsDialog.exec() != QDialog::Accepted)
				return;

			QMap<QString, QVariant> parameters = settingsDialog.getParameters();

			// 参数提取
			bool debugEnabled = parameters["debugEnabled"].toBool();
			bool isGetGround = parameters["isGetGround"].toBool();
			bool doFitLine = parameters["doFitLine"].toBool();
			bool useDynamicRect = parameters["useDynamicRect"].toBool();
			double W = parameters["W"].toFloat();
			double L = parameters["L"].toFloat();
			int    Nmin = parameters["Nmin"].toUInt();
			double theta_max = parameters["theta_max_degrees"]
				.toFloat() * M_PI / 180.0; // 角度转弧度
			int    Kmax = parameters["Kmax"].toUInt();

			std::vector<CCVector3d> polyline;
			m_foregroundPolylineEditor->getPoints(polyline);

			if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))return;

			CCVector3 p0, v0;
			p0.x = polyline[0].x;
			p0.y = polyline[0].y;
			//p0.z = polyline[0].z;

			v0.x = polyline[1].x - p0.x;
			v0.y = polyline[1].y - p0.y;
			//v0.z = polyline[1].z - p0.z;
			ccPointCloud* cloud= new ccPointCloud;
			std::vector<CCVector3>line;

			CloudProcess::grow_line_from_seed(
				static_cast<ccPointCloud*>(p_select_cloud),
				p0,
				v0,
				cloud,
				line,
				debugEnabled ? m_glWindow : nullptr,
				isGetGround,
				doFitLine,        
				useDynamicRect,
				W,
				L,
				Nmin,
				theta_max,
				Kmax
			);


			ccPointCloud* ccCloud = new ccPointCloud;
			// 创建ccPolyline对象
			ccPolyline* polylineObj = new ccPolyline(ccCloud);

			// 将line中的点添加到polylineObj中
			for (const auto& point : line)
			{
				ccCloud->addPoint(CCVector3(point.x, point.y, point.z)); // 添加点
				polylineObj->addPointIndex(ccCloud->size() - 1);
			}

			polylineObj->setName("Extracted Polyline");
			polylineObj->setColor(ccColor::yellowRGB);
			polylineObj->showColors(true);
			ccHObject* selectedCloud = static_cast<ccHObject*>(p_select_cloud);
			selectedCloud->addChild(polylineObj);

			cloud->setName("Extracted Cloud");
			cloud->setColor(ccColor::red);
			cloud->showColors(true);
			selectedCloud->addChild(cloud);

			m_objectTree->refresh();
		});
}

void qSignExtractDlg::onFilteCloudByIntensity()
{
	if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))
	{
		ccLog::Error("未选择点云");
		return;
	}

	ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(p_select_cloud);

	showThresholdHistogram(cloud, true);
	m_objectTree->refresh();
}

void qSignExtractDlg::onFilteCloudByZ()
{
	if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))
	{
		ccLog::Error("未选择点云");
		return;
	}

	ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(p_select_cloud);

	showThresholdHistogram(cloud, false, true , 0, 200);
	m_objectTree->refresh();
}

void qSignExtractDlg::onBoxClip()
{
	m_selectionMode = DRAW_SELECTION;
	m_foregroundPolylineEditor->startDraw();
	m_foregroundPolylineEditor->setCallbackfunc([&]
		{
			std::vector<CCVector3d> polyline;
			m_foregroundPolylineEditor->getPoints(polyline);
			std::vector <ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			ccPointCloud* cloud = new ccPointCloud;
			CloudProcess::crop_cloud_with_polygon(clouds, polyline, cloud);

			cloud->setName("cloud_cropped");
			addCloudToDB(cloud);
			m_objectTree->async_refresh();
		});
}

void qSignExtractDlg::addCloudToDB(ccPointCloud* cloud)
{
	m_glWindow->addToOwnDB(cloud);
	if (p_select_cloud)p_select_cloud->setVisible(false);
	onEntitySelectionChanged(cloud);
}


void qSignExtractDlg::onItemPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3&, const CCVector3d&)
{

}

void qSignExtractDlg::onItemPickedFast(ccHObject* entity, int subEntityID, int x, int y)
{
	
	
}

void qSignExtractDlg::onLeftButtonClicked(int x, int y)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_foregroundPolylineEditor->onLeftButtonClicked(x, y);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onLeftButtonDoubleClicked(int x, int y)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_foregroundPolylineEditor->onDoubleLeftButtonClicked(x, y);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onRightButtonDoubleClicked(int x, int y)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_foregroundPolylineEditor->onDoubleRightButtonClicked(x, y);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_foregroundPolylineEditor->onMouseMoved(x, y, button);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onButtonReleased()
{
	m_glWindow->redraw();
}

void qSignExtractDlg::onMouseWheelRotated(int delta)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_foregroundPolylineEditor->onMouseWheelRotated(delta);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onEntitySelectionChanged(ccHObject* entity)
{
	if (!entity)return;
	if (p_select_cloud)
	{
		p_select_cloud->setSelected(false);
	}
	entity->setSelected(true);
	p_select_cloud = entity;
	m_glWindow->redraw();
	if (m_objectTree)m_objectTree->async_refresh();
}

// 应该是很多键被主程序当作快捷键拦截了，只有部分键有效
void qSignExtractDlg::keyPressEvent(QKeyEvent* event)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_foregroundPolylineEditor->onKeyPressEvent(event);
	}
}

void qSignExtractDlg::showThresholdHistogram(ccPointCloud* pointCloud, bool isfilterIntensity,bool is_has_threshold, float lowerThreshold, float upperThreshold)
{
	histogramWidget->setPointCloud(pointCloud, isfilterIntensity); // 设置点云数据

	histogramWidget->setUpperAndLowerThreshold(is_has_threshold, lowerThreshold, upperThreshold); // 设置阈值

	histogramWidget->show();
}


// ============================================================================ ForegroundPolylineEditor

ForegroundPolylineEditor::ForegroundPolylineEditor(ccGLWindowInterface* glWindow)
	: m_glWindow(glWindow), m_pointCloud(new ccPointCloud()), m_foregroundPolyline(new ccPolyline(m_pointCloud))
{
	m_foregroundPolyline->addChild(m_pointCloud);
	m_foregroundPolyline->setColor(ccColor::green);
	m_foregroundPolyline->showColors(true);
	m_foregroundPolyline->set2DMode(true);
	m_foregroundPolyline->setForeground(true);
	m_foregroundPolyline->showVertices(true);
	m_foregroundPolyline->setVertexMarkerWidth(2);
}

ForegroundPolylineEditor::~ForegroundPolylineEditor()
{
	m_glWindow->removeFromOwnDB(m_foregroundPolyline);

	if (m_pointCloud)
	{
		delete m_pointCloud;  // 删除内部创建的点云
		m_pointCloud = nullptr;
	}
	// 释放资源
	if (m_foregroundPolyline)
	{
		delete m_foregroundPolyline;
		m_foregroundPolyline = nullptr;
	}

}

void ForegroundPolylineEditor::setSelectCloudPtr(ccHObject** select_cloud)
{
	pp_select_cloud = select_cloud;
}

void ForegroundPolylineEditor::startDraw()
{
	// 窗口的准备
	{
		// 备份交互方式，结束时恢复，禁用其他按钮
		interaction_flags_backup = m_glWindow->getInteractionMode();
		picking_mode_backup = m_glWindow->getPickingMode();
		emit draw_start();

		// 固定为从上往下的正交视图，禁止相机旋转
		m_glWindow->setView(CC_TOP_VIEW);
		//if (pp_select_cloud && *pp_select_cloud)
		//{
		//	ccBBox bbox = (*pp_select_cloud)->getOwnBB();
		//	m_glWindow->updateConstellationCenterAndZoom(&bbox);
		//}
		m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_PAN_ONLY | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
		m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);
	}

	// 数据的准备
	{
		// 如果已有折线，则删除它
		if (m_foregroundPolyline->size())
		{
			m_foregroundPolyline->clear();
			m_pointCloud->clear();
			m_3DPoints.clear();
		}
		m_glWindow->addToOwnDB(m_foregroundPolyline);
	}
}

void ForegroundPolylineEditor::finishDraw(bool doAction)
{
	isFreezeUI = true;

	if (doAction && m_callback)
	{
		if (isClosed)m_3DPoints.push_back(m_3DPoints[0]);
		m_callback();
	}
	m_glWindow->setInteractionMode(interaction_flags_backup);
	m_glWindow->setPickingMode(picking_mode_backup);
	emit draw_finish();
	m_glWindow->removeFromOwnDB(m_foregroundPolyline);
	emit update_tree();
	if (m_foregroundPolyline->size())
	{
		m_foregroundPolyline->clear();
		m_pointCloud->clear();
		m_3DPoints.clear();
	}
	m_glWindow->redraw(true, false);
	isFreezeUI = false;
	resetDraw();
}

void ForegroundPolylineEditor:: setCallbackfunc(std::function<void()> callback)
{
	// 设置回调函数
	m_callback = callback;
}

void ForegroundPolylineEditor::onLeftButtonClicked(int x, int y)
{
	if (isFreezeUI)return;
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	if (!m_pointCloud->size())
	{
		m_pointCloud->addPoint(newPoint);
		m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1)); // 临时
		m_foregroundPolyline->addPointIndex(0); // 闭合
	}

	// 上一个点固定
	{
		CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 2));
		*lastPoint = newPoint;
		ccGLCameraParameters camera;
		m_glWindow->getGLCameraParameters(camera);

		// 修正，同 ForegroundPolylineEditor::updatePoly()
		camera.viewport[0] = -camera.viewport[2] / 2;
		camera.viewport[1] = -camera.viewport[3] / 2;

		CCVector3d _3DPoint;
		camera.unproject(*lastPoint, _3DPoint);
		m_3DPoints.push_back(_3DPoint);

		if (m_3DPoints.size() == max_points_num)
		{
			finishDraw(true);
			return;
		}
	}

	m_pointCloud->addPoint(newPoint);
	m_foregroundPolyline->removePointGlobalIndex(m_foregroundPolyline->size() - 1);
	m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1));
	if(isClosed)m_foregroundPolyline->addPointIndex(0);
	else m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1));
	m_glWindow->redraw(true, false);
}

void ForegroundPolylineEditor::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (isFreezeUI)return;

	if (!m_pointCloud->size())
		return;

	// 拖动更新2D屏幕上的点
	if (button == Qt::RightButton)
	{
		updatePoly();
	}

	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 2));
	*lastPoint = newPoint;
}

void ForegroundPolylineEditor::onDoubleLeftButtonClicked(int x, int y)
{
	if (isFreezeUI) return;
	finishDraw(true);
}

void ForegroundPolylineEditor::onDoubleRightButtonClicked(int x, int y)
{
	if (isFreezeUI) return;
	finishDraw(false);
}

void ForegroundPolylineEditor::onMouseWheelRotated(int delta)
{
	if (isFreezeUI)return;

	updatePoly();
}

void ForegroundPolylineEditor::onKeyPressEvent(QKeyEvent* event)
{
	if (isFreezeUI)return;

	if (event->key() == Qt::Key_F)
	{
		finishDraw(true);
	}

	if (event->key() == Qt::Key_G)
	{
		finishDraw(false);
	}
}

void ForegroundPolylineEditor::updatePoly()
{
	ccGLCameraParameters camera;

	// 缩放中心(viewport[0] + viewport[2]/2, viewport[1] + viewport[3]/2)在gl屏幕上是右上角， 需要移到中心
	// 不知道内部的过程为什么会产生这样的结果
	// 手动修正
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	for (size_t i = 0; i < m_3DPoints.size(); ++i)
	{
		CCVector3d projectedPoint;
		camera.project(m_3DPoints[i], projectedPoint);
		CCVector3* point = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(i));

		*point = CCVector3(projectedPoint.x
			, projectedPoint.y
			, 0);
	}
	m_glWindow->redraw(true, false);
}


void ForegroundPolylineEditor::getPoints(std::vector<CCVector3d>& polyline)
{
	polyline = m_3DPoints;
}

void ForegroundPolylineEditor::setDraw(int max_points_num,bool isClosed)
{
	this->max_points_num = max_points_num;
	this->isClosed = isClosed;
}

void ForegroundPolylineEditor::resetDraw()
{
	max_points_num = INF;
	isClosed = true;
}

// ============================================================================ CloudObjectTreeWidget
#include <QMenu>
#include <QAction>
#include <QHeaderView>

CloudObjectTreeWidget::CloudObjectTreeWidget(QWidget* parent)
	: QTreeWidget(parent)
{
	setColumnCount(1);
	setHeaderLabel("对象目录");
	setSelectionMode(QAbstractItemView::ExtendedSelection);
	setContextMenuPolicy(Qt::DefaultContextMenu);

	connect(this, &CloudObjectTreeWidget::async_refresh, this, &CloudObjectTreeWidget::refresh, Qt::QueuedConnection);

	// 复选框设置可见性
	connect(this, &QTreeWidget::itemChanged, this, [&](QTreeWidgetItem * item, int column)
	{
		if (!item || column != 0)
			return;

		auto obj = static_cast<ccHObject*>(item->data(0, Qt::UserRole).value<void*>());
		if (!obj)
			return;

		bool visible = (item->checkState(0) == Qt::Checked);
		if (!visible && *pp_select_cloud == obj)
		{
			obj->setSelected(false);
			*pp_select_cloud = nullptr;
		}

		std::function<void(ccHObject*)> dfs = [&](ccHObject* object)
		{
			object->setVisible(visible);
			for (int i = 0; i < object->getChildrenNumber(); ++i)
			{
				ccHObject* child = object->getChild(i);
				if (child)
				{
					dfs(child);
				}
			}
		};
		dfs(obj);
		emit async_refresh();

		if (m_glWindow)
			m_glWindow->redraw();
	});

	// 点击设置选中点云
	connect(this, &QTreeWidget::itemClicked, this, [&](QTreeWidgetItem* item, int column)
	{
		if (!item)return;
		auto obj = static_cast<ccHObject*>(item->data(0, Qt::UserRole).value<void*>());
		if (!obj || !obj->isVisible()) 
			return;

		if(*pp_select_cloud)(*pp_select_cloud)->setSelected(false);

		obj->setSelected(true);
		*pp_select_cloud = obj;

		if (m_glWindow)
			m_glWindow->redraw();
	});
}

void CloudObjectTreeWidget::initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud, const std::vector<ccHObject*>& objects)
{
	m_glWindow = win;
	m_app = app;
	pp_select_cloud = select_cloud;

	if (objects.size())
	{
		originalDisplay = objects[0]->getDisplay();
		for (auto object : objects)
		{
			// if (!object->getParent()) // 只用放入所有根节点
			{
				if (object->isA(CC_TYPES::POINT_CLOUD))
				{
					addCloud(static_cast<ccPointCloud*>(object));
				}
				else
				{
					m_glWindow->addToOwnDB(object);
				}
			}
		}
	}
	refresh();
}

void CloudObjectTreeWidget::addCloud(ccPointCloud* cloud, ccHObject* parent)
{
	// 查找名为“intensity”的标量字段
	int sfIdx = -1;
	const int sfCount = cloud->getNumberOfScalarFields();
	for (int i = 0; i < sfCount; ++i)
	{
		if (QString::fromStdString(cloud->getScalarField(i)->getName()).contains("intensity", Qt::CaseInsensitive))
		{
			sfIdx = i;
			break;
		}
	}

	// 如果找到了强度标量字段
	if (sfIdx >= 0)
	{
		// 设置该标量字段作为颜色显示
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);  // 显示标量字段
		cloud->showColors(true);  // 启用颜色显示
	}
	else
	{
		// 如果没有强度标量字段，保持默认行为
		cloud->showSF(false);
		//cloud->showColors(false);
	}

	cloud->setEnabled(true);
	cloud->setVisible(true);

	if(parent)
	{
		parent->addChild(cloud);
	}
	else
	{
		m_glWindow->addToOwnDB(cloud);
	}
}

void CloudObjectTreeWidget::refresh()
{
	blockSignals(true);

	clear();

	if (!m_glWindow)
		return;

	ccHObject* dbRoot = m_glWindow->getOwnDB();
	if (!dbRoot || dbRoot->getChildrenNumber() == 0)
		return;

	for (int i = 0; i < dbRoot->getChildrenNumber(); i++)
	{
		if (!i)root = dbRoot->getChild(i);
		loadTreeItem(dbRoot->getChild(i), nullptr);
	}
	expandAll();  // 展开所有项

	// 恢复信号处理
	blockSignals(false);
}

void CloudObjectTreeWidget::loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem)
{
	// 创建树项
	QTreeWidgetItem* item = new QTreeWidgetItem(parentItem);
	item->setText(0, object->getName());  // 设置项的文本为对象的名称
	item->setCheckState(0, object->isVisible() ? Qt::Checked : Qt::Unchecked);  // 设置复选框状态
	item->setData(0, Qt::UserRole, QVariant::fromValue<void*>(object));  // 将对象绑定到该项

	// 设置项标志，使其支持用户勾选
	item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
	// 如果有父项，就将当前项作为父项的子项添加
	if (parentItem)
	{
		parentItem->addChild(item);
	}
	else
	{
		addTopLevelItem(item);  // 如果没有父项，作为根项添加
	}
	item->setSelected(object->isSelected());

	// 递归加载子项
	for (int i = 0; i < object->getChildrenNumber(); ++i)
	{
		ccHObject* child = object->getChild(i);
		if (child)
		{
			child->setDisplay(object->getDisplay()); // 在同窗口显示
			loadTreeItem(child, item);  // 将子节点递归挂载到当前项下
		}
	}
}

void CloudObjectTreeWidget::contextMenuEvent(QContextMenuEvent* event)
{
	QMenu menu(this);

	QAction* delAct = new QAction("删除所选对象", &menu);

	connect(delAct, &QAction::triggered, this, [=]()
		{
			const QList<QTreeWidgetItem*> selItems = selectedItems();
			if (!m_glWindow || selItems.isEmpty())
				return;

			QSet<ccHObject*> delSet;
			for (auto* item : selItems)
			{
				auto obj = static_cast<ccHObject*>(item->data(0, Qt::UserRole).value<void*>());
				if (obj && obj != root)
					delSet.insert(obj);
			}

			for (auto* obj : delSet)
			{
				bool skip = false;
				for (ccHObject* p = obj->getParent(); p; p = p->getParent())
				{
					if (delSet.contains(p))
					{
						skip = true;
						break;
					}
				}
				if (skip)
					continue;

				if (*pp_select_cloud == obj)
					*pp_select_cloud = nullptr;

				if (obj->getParent())obj->getParent()->removeChild(obj);
				else m_glWindow->removeFromOwnDB(obj);
			}

			refresh();
		});


	menu.addAction(delAct);

	menu.exec(event->globalPos());
}

void CloudObjectTreeWidget::getAllPointClouds(std::vector<ccPointCloud*>& pointClouds)
{
	if (!m_glWindow)
		return;

	ccHObject* dbRoot = m_glWindow->getOwnDB();
	if (!dbRoot || dbRoot->getChildrenNumber() == 0)
		return;

	for (int i = 0; i < dbRoot->getChildrenNumber(); ++i)
	{
		ccHObject* child = dbRoot->getChild(i);
		if (child)
		{
			getAllPointCloudsRecursive(child, pointClouds);
		}
	}
}

void CloudObjectTreeWidget::getAllPointCloudsRecursive(ccHObject* object, std::vector<ccPointCloud*>& pointClouds)
{
	if (ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(object))
	{
		if (cloud->isVisible())
			pointClouds.push_back(cloud);
	}

	for (int i = 0; i < object->getChildrenNumber(); ++i)
	{
		ccHObject* child = object->getChild(i);
		if (child)
		{
			getAllPointCloudsRecursive(child, pointClouds);
		}
	}
}

void CloudObjectTreeWidget::relase()
{
	std::function<void(ccHObject*)> dfsChild = [&](ccHObject* object)
	{
		if (!object)
			return;

		object->setVisible(true);
		object->setSelected(false);

		for (int i = 0; i < object->getChildrenNumber(); i++)
		{
			ccHObject* child = object->getChild(i);
			if (child)
			{
				child->setDisplay(object->getDisplay());
				dfsChild(child);
			}
		}
	};

	if (!m_glWindow)
		return;

	ccHObject* dbRoot = m_glWindow->getOwnDB();
	if (!dbRoot || dbRoot->getChildrenNumber() == 0)
		return;

	for (int i = 0; i < dbRoot->getChildrenNumber(); i++)
	{
		dbRoot->getChild(i)->setDisplay(originalDisplay);
		m_app->addToDB(dbRoot->getChild(i));
		dfsChild(dbRoot->getChild(i));
	}
}
