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
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <nlohmann/json.hpp>
#include "CloudProcess.h"
#include "RoadMarkingExtract.h"
#include "PointCloudSelector.h"

using namespace roadmarking;
using namespace cc_interact;
using namespace cc_drawing;
using json = nlohmann::json;

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
	m_pointCloudSelector = new PointCloudSelector(m_glWindow);
	m_pointCloudSelector->setSelectCloudPtr(&p_select_cloud);

	m_pointCloudDrawer = new PointCloudDrawer(m_glWindow);
	m_pointCloudDrawer->setSelectCloudPtr(&p_select_cloud);


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

		connect(m_pointCloudSelector, &PointCloudSelector::draw_finish, [viewGroup]()
			{
				for (int i = 0; i < viewGroup->layout()->count(); ++i) {
					QToolButton* btn = qobject_cast<QToolButton*>(viewGroup->layout()->itemAt(i)->widget());
					if (btn) {
						btn->setEnabled(true);
					}
				}
			});

		connect(m_pointCloudSelector, &PointCloudSelector::draw_start, [viewGroup]()
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

		connect(m_pointCloudDrawer, &PointCloudDrawer::draw_finish, [viewGroup]()
			{
				for (int i = 0; i < viewGroup->layout()->count(); ++i) {
					QToolButton* btn = qobject_cast<QToolButton*>(viewGroup->layout()->itemAt(i)->widget());
					if (btn) {
						btn->setEnabled(true);
					}
				}
			});

		connect(m_pointCloudDrawer, &PointCloudDrawer::draw_start, [viewGroup]()
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



	std::vector<QButtonGroup*> allGroups;
	// ==================================== 功能按钮组工具函数
	auto addGroupedButton = [&](const QString& text,
		std::function<void()> callback,
		QGroupBox* parentGroup,
		QVBoxLayout* layout,
		QButtonGroup* group)
	{
		QPushButton* btn = new QPushButton(text, parentGroup);
		btn->setCheckable(true);
		layout->addWidget(btn);
		connect(btn, &QPushButton::clicked, this, std::move(callback));
		group->addButton(btn);
	};

	// ==================================== 功能按钮组：过滤类
	{
		QGroupBox* filterGroup = new QGroupBox("点云过滤", this);
		QVBoxLayout* filterLayout = new QVBoxLayout(filterGroup);
		QButtonGroup* filterButtonGroup = new QButtonGroup(this);
		allGroups.push_back(filterButtonGroup);
		filterButtonGroup->setExclusive(true);

		addGroupedButton("选择强度阈值过滤点云", [this]() { onFilteCloudByIntensity(); }, filterGroup, filterLayout, filterButtonGroup);
		addGroupedButton("选择高程阈值过滤点云", [this]() { onFilteCloudByZ(); }, filterGroup, filterLayout, filterButtonGroup);

		filterGroup->setLayout(filterLayout);
		leftLayout->addWidget(filterGroup);
	}

	// ==================================== 功能按钮组：剪切类
	{
		QGroupBox* clipGroup = new QGroupBox("点云剪切", this);
		QVBoxLayout* clipLayout = new QVBoxLayout(clipGroup);
		QButtonGroup* clipButtonGroup = new QButtonGroup(this);
		allGroups.push_back(clipButtonGroup);
		clipButtonGroup->setExclusive(true);

		addGroupedButton("多边形截取点云", [this]() { onBoxClip(); }, clipGroup, clipLayout, clipButtonGroup);
		addGroupedButton("矩形截取点云", [this]() { onRectClip(); }, clipGroup, clipLayout, clipButtonGroup);
		addGroupedButton("模型制作", [this]() {onMakeModel(); }, clipGroup, clipLayout, clipButtonGroup);

		clipGroup->setLayout(clipLayout);
		leftLayout->addWidget(clipGroup);
	}

	// ==================================== 功能按钮组：提取类
	{
		QGroupBox* extractGroup = new QGroupBox("特征提取", this);
		QVBoxLayout* extractLayout = new QVBoxLayout(extractGroup);
		QButtonGroup* extractButtonGroup = new QButtonGroup(this);
		allGroups.push_back(extractButtonGroup);
		extractButtonGroup->setExclusive(true);

		addGroupedButton("全自动提取", [this]() { onAutoExtract(); }, extractGroup, extractLayout, extractButtonGroup);
		addGroupedButton("框选提取", [this]() { onBoxSelectExtract(); }, extractGroup, extractLayout, extractButtonGroup);
		addGroupedButton("点选生长提取", [this]() { onPointGrowExtract(); }, extractGroup, extractLayout, extractButtonGroup);
		addGroupedButton("框选斑马线提取", [this]() { onZebraExtract(); }, extractGroup, extractLayout, extractButtonGroup);

		extractGroup->setLayout(extractLayout);
		leftLayout->addWidget(extractGroup);
	}

	// ==================================== 功能按钮组：匹配类（新增）
	{
		QGroupBox* matchGroup = new QGroupBox("点云匹配", this);
		QVBoxLayout* matchLayout = new QVBoxLayout(matchGroup);
		QButtonGroup* matchButtonGroup = new QButtonGroup(this);
		allGroups.push_back(matchButtonGroup);
		matchButtonGroup->setExclusive(true);

		addGroupedButton("直接匹配模板", [this]() { onMatchTemplateDirect(); }, matchGroup, matchLayout, matchButtonGroup);
		addGroupedButton("框选匹配模板", [this]() { onMatchTemplateByBox();   }, matchGroup, matchLayout, matchButtonGroup);
		addGroupedButton("点击匹配模板", [this]() { onMatchTemplateByClick(); }, matchGroup, matchLayout, matchButtonGroup);

		matchGroup->setLayout(matchLayout);
		leftLayout->addWidget(matchGroup);
	}


	// ==================================== 启用/禁用控制（所有组按钮）
	{
		connect(m_pointCloudSelector, &PointCloudSelector::draw_start, [allGroups]() {
			for (auto* group : allGroups)
			{
				for (auto* btn : group->buttons())
					btn->setEnabled(false);
			}
			});

		connect(m_pointCloudSelector, &PointCloudSelector::draw_finish, [allGroups]() {
			for (auto* group : allGroups)
			{
				for (auto* btn : group->buttons())
					btn->setEnabled(true);
			}
			});

		connect(m_pointCloudDrawer, &PointCloudDrawer::draw_start, [allGroups]() {
			for (auto* group : allGroups)
			{
				for (auto* btn : group->buttons())
					btn->setEnabled(false);
			}
			});

		connect(m_pointCloudDrawer, &PointCloudDrawer::draw_finish, [allGroups]() {
			for (auto* group : allGroups)
			{
				for (auto* btn : group->buttons())
					btn->setEnabled(true);
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

	connect(m_pointCloudSelector, &PointCloudSelector::update_tree, m_objectTree, [=]() { m_objectTree->refresh(); });
	connect(m_pointCloudSelector, &PointCloudSelector::draw_start, [&]() {m_selectionMode = DRAW_SELECTION; });
	connect(m_pointCloudSelector, &PointCloudSelector::draw_finish, [&](){m_selectionMode = ENTITY_SELECTION;});

	connect(m_pointCloudDrawer, &PointCloudDrawer::update_tree, m_objectTree, [=]() { m_objectTree->refresh(); });
	connect(m_pointCloudDrawer, &PointCloudDrawer::draw_start, [&]() {m_selectionMode = DRAW_MODEL; });
	connect(m_pointCloudDrawer, &PointCloudDrawer::draw_finish, [&]() {m_selectionMode = ENTITY_SELECTION; });

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

	if (m_pointCloudSelector)
	{
		delete m_pointCloudSelector;
	}

	if (m_pointCloudDrawer)
	{
		delete m_pointCloudDrawer;
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
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::PolylineClosed);
	m_pointCloudSelector->setCallbackfunc([=]
		{
			std::vector<CCVector3d> polyline;
			m_pointCloudSelector->getPoints(polyline);
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
	if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))
	{
		ccLog::Error("未选择点云");
		return;
	}
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::PolylineOpen, 2);
	m_pointCloudSelector->setCallbackfunc([&]
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
			m_pointCloudSelector->getPoints(polyline);

			if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))return;

			CCVector3 p0, v0;
			p0.x = polyline[0].x;
			p0.y = polyline[0].y;
			//p0.z = ctrolPoints[0].z;

			v0.x = polyline[1].x - p0.x;
			v0.y = polyline[1].y - p0.y;
			//v0.z = ctrolPoints[1].z - p0.z;
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

void qSignExtractDlg::onZebraExtract()
{
	if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))
	{
		ccLog::Error("未选择点云");
		return;
	}
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::PolylineClosed);
	m_pointCloudSelector->setCallbackfunc([&]
		{
			SettingsDialog settingsDialog;
			settingsDialog.setDescription(
				"参数：\n"
				"矩形宽度(m)：主方向投影分析的栅格宽度\n"
				"密度阈值：判断白线的投影密度阈值\n"
				"最小白线长度(m)：有效白线段的最小长度\n"
				"是否显示调试结果"
			);

			settingsDialog.registerComponent<float>("矩形宽度", "binWidth", 0.1f);
			settingsDialog.registerComponent<int>("密度阈值", "densityThreshold", 10);
			settingsDialog.registerComponent<float>("最小白线长度", "minStripeLength", 1.0f);
			settingsDialog.registerComponent<bool>("调试显示", "debugEnabled", true);

			if (settingsDialog.exec() != QDialog::Accepted)
				return;

			QMap<QString, QVariant> parameters = settingsDialog.getParameters();

			float binWidth = parameters["binWidth"].toFloat();
			int densityThreshold = parameters["densityThreshold"].toInt();
			float minStripeLength = parameters["minStripeLength"].toFloat();
			bool debugEnabled = parameters["debugEnabled"].toBool();

			std::vector<CCVector3d> polyline;
			m_pointCloudSelector->getPoints(polyline);
			std::vector <ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			ccPointCloud* cloud = new ccPointCloud;
			CloudProcess::crop_cloud_with_polygon(clouds, polyline, cloud);

			ccPointCloud* outCloud = new ccPointCloud;
			std::vector<CCVector3> centers;
			//CloudProcess::extract_zebra_by_projection(cloud, binWidth, densityThreshold, minStripeLength,m_glWindow, outCloud, centers);
			CloudProcess::extract_zebra_by_struct(cloud, m_glWindow);

			delete cloud;
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
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::PolylineClosed);
	m_pointCloudSelector->setCallbackfunc([&]
		{
			std::vector<CCVector3d> polyline;
			m_pointCloudSelector->getPoints(polyline);
			std::vector <ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			ccPointCloud* cloud = new ccPointCloud;
			CloudProcess::crop_cloud_with_polygon(clouds, polyline, cloud);

			cloud->setName("cloud_cropped");
			addCloudToDB(cloud);
			m_objectTree->async_refresh();
		});
}

void qSignExtractDlg::onMatchTemplateDirect()
{
	if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))
	{
		ccLog::Error("未选择点云");
		return;
	}

	auto p_cloud = PointCloudIO::convert_to_ccCloudPtr(p_select_cloud);
	p_select_cloud->addChild(CloudProcess::apply_roadmarking_vectorization(p_cloud));
	m_objectTree->async_refresh();
}

void qSignExtractDlg::onMatchTemplateByBox()
{
	// 先让用户在视图中用矩形框选区域
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::Rectangle);
	m_pointCloudSelector->setCallbackfunc([&]()
		{
			// 获取用户绘制的闭合多边形（矩形四角）
			std::vector<CCVector3d> polyline;
			m_pointCloudSelector->getPoints(polyline);

			std::vector <ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			ccPointCloud* cloud = new ccPointCloud;
			CloudProcess::crop_cloud_with_polygon(clouds, polyline, cloud);
			
			auto markings = CloudProcess::apply_roadmarking_vectorization(cloud);

			if (p_select_cloud)
			{
				p_select_cloud->addChild(markings);
			}
			else
			{
				cloud->setName("cloud_cropped");
				m_glWindow->addToOwnDB(cloud);
				m_objectTree->refresh();
				cloud->addChild(markings);
			}
			m_objectTree->refresh();
		});
}

void qSignExtractDlg::onMatchTemplateByClick()
{
	
}

void qSignExtractDlg::onRectClip()
{
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::Rectangle);
	m_pointCloudSelector->setCallbackfunc([&]
		{
			std::vector<CCVector3d> polyline;
			m_pointCloudSelector->getPoints(polyline);
			std::vector <ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			ccPointCloud* cloud = new ccPointCloud;
			CloudProcess::crop_cloud_with_polygon(clouds, polyline, cloud);

			cloud->setName("cloud_cropped");
			addCloudToDB(cloud);
			m_objectTree->async_refresh();
		});
}

#include <pcl/io/pcd_io.h>
void qSignExtractDlg::onMakeModel()
{
	if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))
	{
		QMessageBox::critical(nullptr, "错误", "未选择点云");
		return;
	}

	// 选择 JSON 模板文件
	QString jsonFilePath = QFileDialog::getOpenFileName(
		nullptr, "选择 JSON 模板文件", "", "JSON Files (*.json);;All Files (*)");
	if (jsonFilePath.isEmpty()) {
		QMessageBox::information(nullptr, "提示", "未选择文件");
		return;
	}

	// 读取 JSON 文件
	json j;
	{
		std::ifstream inputFile(jsonFilePath.toStdString());
		if (inputFile.is_open()) {
			try {
				inputFile >> j;
			}
			catch (...) {
				QMessageBox::critical(nullptr, "错误", "JSON 读取失败，可能格式错误");
				return;
			}
			inputFile.close();
		}
		else {
			QMessageBox::critical(nullptr, "错误", "无法打开 JSON 文件");
			return;
		}
	}

	// 用户输入模板名称
	bool ok;
	QString modelName = QInputDialog::getText(
		nullptr, "输入模板名称", "请输入模板名称：", QLineEdit::Normal, "", &ok);
	if (!ok || modelName.isEmpty()) {
		QMessageBox::information(nullptr, "提示", "未输入模板名称");
		return;
	}

	ccPointCloud* p_cloud = static_cast<ccPointCloud*>(p_select_cloud);
	if (p_cloud->size() > 100000)
	{
		QMessageBox::critical(nullptr, "错误", "点云太大");
		return;
	}
	// 获取原始点云和轮廓
	PCLCloudPtr pclCloud = PointCloudIO::convert_to_PCLCloudPtr(p_cloud);
	PCLCloudPtr outlineCloud = CloudProcess::extract_outline(pclCloud);
	if (!outlineCloud) {
		QMessageBox::critical(nullptr, "错误", "轮廓提取失败");
		return;
	}

	// 获取用户绘制的折线
	std::vector<PCLPoint> controlPoints;
	std::vector<std::vector<int>> polylines;

	m_pointCloudDrawer->startDraw();
	m_pointCloudDrawer->setCallbackfunc([=]() mutable
		{
			// ========== A. 从绘制器获取所有折线（多条） ==========
			std::vector<std::vector<CCVector3d>> allPolylines;
			m_pointCloudDrawer->getPoints(allPolylines);

			// 如果用户没有画任何折线，就提示并返回
			if (allPolylines.empty())
			{
				QMessageBox::warning(nullptr, "提示", "未检测到任何折线，无法生成模板。");
				return;
			}

			// ========== B. 获取当前场景中所有点云，并执行裁剪 ==========
			std::vector<ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			// ========== C. 准备保存路径（PCD） ==========
			QFileInfo jsonInfo(jsonFilePath);
			QString dirPath = jsonInfo.absolutePath();
			QString modelPath = dirPath + "/" + modelName + ".pcd";
			QString outlinePath = dirPath + "/" + modelName + "_outline.pcd";

			// --------- 1. 对原始裁剪后点云进行平面化处理，并保存到 modelPath ---------
			{
				// 深拷贝出一份新的点云
				PCLCloudPtr flatCloud(new pcl::PointCloud<pcl::PointXYZ>);
				*flatCloud = *pclCloud; // 逐点复制原始 PCLCloudPtr

				// 将所有点的 z 坐标置为 0
				for (auto& pt : flatCloud->points)
				{
					pt.z = 0.0f;
				}

				// 写入磁盘到 modelPath
				if (pcl::io::savePCDFileASCII(modelPath.toStdString(), *flatCloud) < 0)
				{
					QMessageBox::warning(nullptr, "错误", "保存平面化后的 PCD 文件失败: " + modelPath);
					return;
				}
			}

			// --------- 2. 如果存在轮廓点云，则对其进行平面化并保存到 outlinePath ---------
			if (outlineCloud && !outlineCloud->empty())
			{
				PCLCloudPtr flatOutline(new pcl::PointCloud<pcl::PointXYZ>);
				*flatOutline = *outlineCloud; // 深拷贝

				// 同样将轮廓点云的 z 全部置为 0
				for (auto& pt : flatOutline->points)
				{
					pt.z = 0.0f;
				}

				if (pcl::io::savePCDFileASCII(outlinePath.toStdString(), *flatOutline) < 0)
				{
					QMessageBox::warning(nullptr, "错误", "保存平面化后的轮廓 PCD 文件失败: " + outlinePath);
					return;
				}
			}

			// ========== D. 构造 JSON 对象并写入文件 ==========
			json modelJson;
			modelJson["_comment"] = "该模板包含多条折线，每条折线存放为一组 3D 坐标数组";
			modelJson["name"] = modelName.toStdString();
			modelJson["raw_point_cloud_path"] = modelPath.toStdString();
			modelJson["outline_point_cloud_path"] = outlinePath.toStdString();

			// 直接按折线分组，每条 polyline 是一个点列表，且都投影到 z = 0 平面
			for (const auto& poly : allPolylines)
			{
				json lineArray = json::array();
				for (const auto& cc3d : poly)
				{
					// 保证折线的所有顶点 z 坐标都为 0
					lineArray.push_back({ cc3d.x, cc3d.y, 0.0 });
				}
				modelJson["graph_elements"].push_back({
					{ "type", "polyline" },
					{ "points", lineArray }
					});
			}

			if (!j.contains("models") || !j["models"].is_array())
			{
				j["models"] = json::array();
			}
			j["models"].push_back(modelJson);

			// 写入磁盘（覆盖原 JSON 文件）
			{
				std::ofstream outputFile(jsonFilePath.toStdString());
				if (!outputFile.is_open())
				{
					QMessageBox::warning(nullptr, "错误", "无法打开 JSON 文件: " + jsonFilePath);
					return;
				}
				outputFile << j.dump(4); // 缩进 4 空格
				outputFile.close();
			}

			QMessageBox::information(nullptr, "成功", "模板已成功保存到 JSON 文件！");
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
		m_pointCloudSelector->onLeftButtonClicked(x, y);
	}
	else if (m_selectionMode == DRAW_MODEL)
	{
		// 绘制模式下，左键点击也要通知 Drawer
		m_pointCloudDrawer->onLeftButtonClicked(x, y);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onLeftButtonDoubleClicked(int x, int y)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_pointCloudSelector->onDoubleLeftButtonClicked(x, y);
	}
	else if (m_selectionMode == DRAW_MODEL)
	{
		// 绘制模式下，左键双击结束当前折线或切换状态
		m_pointCloudDrawer->onDoubleLeftButtonClicked(x, y);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onRightButtonDoubleClicked(int x, int y)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_pointCloudSelector->onDoubleRightButtonClicked(x, y);
	}
	else if (m_selectionMode == DRAW_MODEL)
	{
		// 绘制模式下，右键双击也可结束或退出绘制
		m_pointCloudDrawer->onDoubleRightButtonClicked(x, y);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_pointCloudSelector->onMouseMoved(x, y, button);
	}
	else if (m_selectionMode == DRAW_MODEL)
	{
		// 绘制模式下，鼠标移动用于更新折线跟踪
		m_pointCloudDrawer->onMouseMoved(x, y, button);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::onMouseWheelRotated(int delta)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_pointCloudSelector->onMouseWheelRotated(delta);
	}
	else if (m_selectionMode == DRAW_MODEL)
	{
		// 如果需要，绘制模式下滚轮缩放时也可以更新折线的投影
		m_pointCloudDrawer->onMouseWheelRotated(delta);
	}
	m_glWindow->redraw();
}

void qSignExtractDlg::keyPressEvent(QKeyEvent* event)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_pointCloudSelector->onKeyPressEvent(event);
	}
	else if (m_selectionMode == DRAW_MODEL)
	{
		// 绘制模式下，按键（如 F/G）用于结束折线或退出绘制
		m_pointCloudDrawer->onKeyPressEvent(event);
	}
}

void qSignExtractDlg::onButtonReleased()
{
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

void qSignExtractDlg::showThresholdHistogram(ccPointCloud* pointCloud, bool isfilterIntensity,bool is_has_threshold, float lowerThreshold, float upperThreshold)
{
	histogramWidget->setPointCloud(pointCloud, isfilterIntensity); // 设置点云数据

	histogramWidget->setUpperAndLowerThreshold(is_has_threshold, lowerThreshold, upperThreshold); // 设置阈值

	histogramWidget->show();
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

	m_glWindow->redraw();

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
