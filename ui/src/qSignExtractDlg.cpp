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
	setWindowTitle("·�����ʶ��");
	resize(1200, 800);
	setFocusPolicy(Qt::StrongFocus);

	QHBoxLayout* mainLayout = new QHBoxLayout(this);
	QVBoxLayout* leftLayout = new QVBoxLayout();

	m_glWidget = nullptr;
	if (m_app)
		m_app->createGLWindow(m_glWindow, m_glWidget);

	// ==================================== ǰ��������
	m_pointCloudSelector = new PointCloudSelector(m_glWindow);
	m_pointCloudSelector->setSelectCloudPtr(&p_select_cloud);

	m_pointCloudDrawer = new PointCloudDrawer(m_glWindow);
	m_pointCloudDrawer->setSelectCloudPtr(&p_select_cloud);


	// ==================================== ��ֵɸѡ
	histogramWidget = new ThresholdHistogramWidget(this);
	histogramWidget->setWindowFlag(Qt::Window);
	histogramWidget -> hide();
	connect(histogramWidget, &ThresholdHistogramWidget::addCloudToDB, this, &qSignExtractDlg::addCloudToDB);
	// ==================================== ����Ŀ¼
	{
		QGroupBox* objectGroup = new QGroupBox("����Ŀ¼", this);
		m_objectTree = new CloudObjectTreeWidget(objectGroup);
		m_objectTree->initialize(m_glWindow, m_app, &p_select_cloud, {});

		QVBoxLayout* objectLayout = new QVBoxLayout(objectGroup);
		objectLayout->addWidget(m_objectTree);
		objectGroup->setLayout(objectLayout);
		leftLayout->addWidget(objectGroup);
	}

	// ==================================== ��ͼ�л�
	{
		QGroupBox* viewGroup = new QGroupBox("��ͼ�л�", this);
		QHBoxLayout* viewLayout = new QHBoxLayout(viewGroup);  // ���� parent Ϊ viewGroup

		auto addViewButton = [&](QHBoxLayout* layout, const QString& name, CC_VIEW_ORIENTATION view)
		{
			QToolButton* btn = new QToolButton(viewGroup);
			btn->setText(name);
			layout->addWidget(btn);
			connect(btn, &QToolButton::clicked, [=]() {
				if (m_glWindow) m_glWindow->setView(view);
				});
		};

		addViewButton(viewLayout, "ǰ��ͼ", CC_FRONT_VIEW);
		addViewButton(viewLayout, "����ͼ", CC_LEFT_VIEW);
		addViewButton(viewLayout, "����ͼ", CC_RIGHT_VIEW);
		addViewButton(viewLayout, "����ͼ", CC_TOP_VIEW);
		addViewButton(viewLayout, "����ͼ", CC_BACK_VIEW);
		addViewButton(viewLayout, "�ײ���ͼ", CC_BOTTOM_VIEW);

		viewGroup->setLayout(viewLayout);
		leftLayout->addWidget(viewGroup);

		QToolButton* btn = new QToolButton(viewGroup);  // ���� parent Ϊ viewGroup
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
	// ==================================== ���ܰ�ť�鹤�ߺ���
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

	// ==================================== ���ܰ�ť�飺������
	{
		QGroupBox* filterGroup = new QGroupBox("���ƹ���", this);
		QVBoxLayout* filterLayout = new QVBoxLayout(filterGroup);
		QButtonGroup* filterButtonGroup = new QButtonGroup(this);
		allGroups.push_back(filterButtonGroup);
		filterButtonGroup->setExclusive(true);

		addGroupedButton("ѡ��ǿ����ֵ���˵���", [this]() { onFilteCloudByIntensity(); }, filterGroup, filterLayout, filterButtonGroup);
		addGroupedButton("ѡ��߳���ֵ���˵���", [this]() { onFilteCloudByZ(); }, filterGroup, filterLayout, filterButtonGroup);

		filterGroup->setLayout(filterLayout);
		leftLayout->addWidget(filterGroup);
	}

	// ==================================== ���ܰ�ť�飺������
	{
		QGroupBox* clipGroup = new QGroupBox("���Ƽ���", this);
		QVBoxLayout* clipLayout = new QVBoxLayout(clipGroup);
		QButtonGroup* clipButtonGroup = new QButtonGroup(this);
		allGroups.push_back(clipButtonGroup);
		clipButtonGroup->setExclusive(true);

		addGroupedButton("����ν�ȡ����", [this]() { onBoxClip(); }, clipGroup, clipLayout, clipButtonGroup);
		addGroupedButton("���ν�ȡ����", [this]() { onRectClip(); }, clipGroup, clipLayout, clipButtonGroup);
		addGroupedButton("ģ������", [this]() {onMakeModel(); }, clipGroup, clipLayout, clipButtonGroup);

		clipGroup->setLayout(clipLayout);
		leftLayout->addWidget(clipGroup);
	}

	// ==================================== ���ܰ�ť�飺��ȡ��
	{
		QGroupBox* extractGroup = new QGroupBox("������ȡ", this);
		QVBoxLayout* extractLayout = new QVBoxLayout(extractGroup);
		QButtonGroup* extractButtonGroup = new QButtonGroup(this);
		allGroups.push_back(extractButtonGroup);
		extractButtonGroup->setExclusive(true);

		addGroupedButton("ȫ�Զ���ȡ", [this]() { onAutoExtract(); }, extractGroup, extractLayout, extractButtonGroup);
		addGroupedButton("��ѡ��ȡ", [this]() { onBoxSelectExtract(); }, extractGroup, extractLayout, extractButtonGroup);
		addGroupedButton("��ѡ������ȡ", [this]() { onPointGrowExtract(); }, extractGroup, extractLayout, extractButtonGroup);
		addGroupedButton("��ѡ��������ȡ", [this]() { onZebraExtract(); }, extractGroup, extractLayout, extractButtonGroup);

		extractGroup->setLayout(extractLayout);
		leftLayout->addWidget(extractGroup);
	}

	// ==================================== ���ܰ�ť�飺ƥ���ࣨ������
	{
		QGroupBox* matchGroup = new QGroupBox("����ƥ��", this);
		QVBoxLayout* matchLayout = new QVBoxLayout(matchGroup);
		QButtonGroup* matchButtonGroup = new QButtonGroup(this);
		allGroups.push_back(matchButtonGroup);
		matchButtonGroup->setExclusive(true);

		addGroupedButton("ֱ��ƥ��ģ��", [this]() { onMatchTemplateDirect(); }, matchGroup, matchLayout, matchButtonGroup);
		addGroupedButton("��ѡƥ��ģ��", [this]() { onMatchTemplateByBox();   }, matchGroup, matchLayout, matchButtonGroup);
		addGroupedButton("���ƥ��ģ��", [this]() { onMatchTemplateByClick(); }, matchGroup, matchLayout, matchButtonGroup);

		matchGroup->setLayout(matchLayout);
		leftLayout->addWidget(matchGroup);
	}


	// ==================================== ����/���ÿ��ƣ������鰴ť��
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

	// ==================================== GL��������
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
			NO_PICKING,                           // ����ʰȡ���ܣ��û��޷�ѡ���κ�����
			ENTITY_PICKING,                       // ѡ������ʵ�壬�����û�ѡ������ơ������ʵ��
			ENTITY_RECT_PICKING,                  // ��������ѡ�������û�ͨ�����ο�ѡ����ʵ��
			FAST_PICKING,                         // ����ѡ���Ż������ܣ������Ƚϵͣ������ڴ����ݼ�
			POINT_PICKING,                        // ѡ�񵥸��㣬�����û�ѡ������еĵ�
			TRIANGLE_PICKING,                     // ѡ�������Σ������û�ѡ�������е���������Ƭ
			POINT_OR_TRIANGLE_PICKING,            // ѡ���������Σ������û�ѡ������еĵ�������е�������
			POINT_OR_TRIANGLE_OR_LABEL_PICKING,   // ѡ��㡢�����λ��ǩ�������û�ѡ������еĵ㡢�����е������λ��ǩ������Ʊ�ע��
			LABEL_PICKING,                        // ѡ���ǩ�������û�ѡ������еı�ǩ������ע����Ϣ
			DEFAULT_PICKING,                      // Ĭ��ʰȡģʽ��ͨ���� `ENTITY_PICKING` ��ͬ������ѡ��ʵ��
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

	// ==================================== һЩ�ź�

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
		ccLog::Error("δѡ�����");
		return;
	}
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::PolylineOpen, 2);
	m_pointCloudSelector->setCallbackfunc([&]
		{
			SettingsDialog settingsDialog;
			settingsDialog.setDescription(
				"������\n"
				"����ģʽ���Ƿ���������ʾ��������ʾ�ڴ����ϣ�\n"
				"�Ƿ���ж���� csf ��ȡ���棺\n"
				"�Ƿ����ֱ�ߣ�\n"
				"�Ƿ�ʹ�ö�̬���Σ�\n"
				"���ο��(m)���������Ŀ��\n"
				"��������(m)��ÿ����չ�ĳ���\n"
				"��С������ÿ���߶���Ҫ���������ٵ���\n"
				"������۽Ƕ�(��)���߶μ������������۽Ƕ�\n"
				"�����Ծ������������Ծ�������������ڴ���Ͽ����ߣ�"
			);

			// ����ֵ
			settingsDialog.registerComponent<bool>("����������ʾ", "debugEnabled", false);
			settingsDialog.registerComponent<bool>("������ȡ����", "isGetGround", false);
			settingsDialog.registerComponent<bool>("���ֱ��", "doFitLine", false);
			settingsDialog.registerComponent<bool>("ʹ�ö�̬����", "useDynamicRect", true);

			// ����ֵ
			settingsDialog.registerComponent<float>("���ο��", "W", 1.0f);
			settingsDialog.registerComponent<float>("��������", "L", 2.0f);
			settingsDialog.registerComponent<float>("������۽Ƕ�(��)", "theta_max_degrees", 45.0f);

			// ����ֵ
			settingsDialog.registerComponent<int>("��С����", "Nmin", 20);
			settingsDialog.registerComponent<int>("�����Ծ����", "Kmax", 10);

			if (settingsDialog.exec() != QDialog::Accepted)
				return;

			QMap<QString, QVariant> parameters = settingsDialog.getParameters();

			// ������ȡ
			bool debugEnabled = parameters["debugEnabled"].toBool();
			bool isGetGround = parameters["isGetGround"].toBool();
			bool doFitLine = parameters["doFitLine"].toBool();
			bool useDynamicRect = parameters["useDynamicRect"].toBool();
			double W = parameters["W"].toFloat();
			double L = parameters["L"].toFloat();
			int    Nmin = parameters["Nmin"].toUInt();
			double theta_max = parameters["theta_max_degrees"]
				.toFloat() * M_PI / 180.0; // �Ƕ�ת����
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
			// ����ccPolyline����
			ccPolyline* polylineObj = new ccPolyline(ccCloud);

			// ��line�еĵ���ӵ�polylineObj��
			for (const auto& point : line)
			{
				ccCloud->addPoint(CCVector3(point.x, point.y, point.z)); // ��ӵ�
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
		ccLog::Error("δѡ�����");
		return;
	}
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::PolylineClosed);
	m_pointCloudSelector->setCallbackfunc([&]
		{
			SettingsDialog settingsDialog;
			settingsDialog.setDescription(
				"������\n"
				"���ο��(m)��������ͶӰ������դ����\n"
				"�ܶ���ֵ���жϰ��ߵ�ͶӰ�ܶ���ֵ\n"
				"��С���߳���(m)����Ч���߶ε���С����\n"
				"�Ƿ���ʾ���Խ��"
			);

			settingsDialog.registerComponent<float>("���ο��", "binWidth", 0.1f);
			settingsDialog.registerComponent<int>("�ܶ���ֵ", "densityThreshold", 10);
			settingsDialog.registerComponent<float>("��С���߳���", "minStripeLength", 1.0f);
			settingsDialog.registerComponent<bool>("������ʾ", "debugEnabled", true);

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
		ccLog::Error("δѡ�����");
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
		ccLog::Error("δѡ�����");
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
		ccLog::Error("δѡ�����");
		return;
	}

	auto p_cloud = PointCloudIO::convert_to_ccCloudPtr(p_select_cloud);
	p_select_cloud->addChild(CloudProcess::apply_roadmarking_vectorization(p_cloud));
	m_objectTree->async_refresh();
}

void qSignExtractDlg::onMatchTemplateByBox()
{
	// �����û�����ͼ���þ��ο�ѡ����
	m_pointCloudSelector->startDraw();
	m_pointCloudSelector->setDraw(DrawMode::Rectangle);
	m_pointCloudSelector->setCallbackfunc([&]()
		{
			// ��ȡ�û����Ƶıպ϶���Σ������Ľǣ�
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
		QMessageBox::critical(nullptr, "����", "δѡ�����");
		return;
	}

	// ѡ�� JSON ģ���ļ�
	QString jsonFilePath = QFileDialog::getOpenFileName(
		nullptr, "ѡ�� JSON ģ���ļ�", "", "JSON Files (*.json);;All Files (*)");
	if (jsonFilePath.isEmpty()) {
		QMessageBox::information(nullptr, "��ʾ", "δѡ���ļ�");
		return;
	}

	// ��ȡ JSON �ļ�
	json j;
	{
		std::ifstream inputFile(jsonFilePath.toStdString());
		if (inputFile.is_open()) {
			try {
				inputFile >> j;
			}
			catch (...) {
				QMessageBox::critical(nullptr, "����", "JSON ��ȡʧ�ܣ����ܸ�ʽ����");
				return;
			}
			inputFile.close();
		}
		else {
			QMessageBox::critical(nullptr, "����", "�޷��� JSON �ļ�");
			return;
		}
	}

	// �û�����ģ������
	bool ok;
	QString modelName = QInputDialog::getText(
		nullptr, "����ģ������", "������ģ�����ƣ�", QLineEdit::Normal, "", &ok);
	if (!ok || modelName.isEmpty()) {
		QMessageBox::information(nullptr, "��ʾ", "δ����ģ������");
		return;
	}

	ccPointCloud* p_cloud = static_cast<ccPointCloud*>(p_select_cloud);
	if (p_cloud->size() > 100000)
	{
		QMessageBox::critical(nullptr, "����", "����̫��");
		return;
	}
	// ��ȡԭʼ���ƺ�����
	PCLCloudPtr pclCloud = PointCloudIO::convert_to_PCLCloudPtr(p_cloud);
	PCLCloudPtr outlineCloud = CloudProcess::extract_outline(pclCloud);
	if (!outlineCloud) {
		QMessageBox::critical(nullptr, "����", "������ȡʧ��");
		return;
	}

	// ��ȡ�û����Ƶ�����
	std::vector<PCLPoint> controlPoints;
	std::vector<std::vector<int>> polylines;

	m_pointCloudDrawer->startDraw();
	m_pointCloudDrawer->setCallbackfunc([=]() mutable
		{
			// ========== A. �ӻ�������ȡ�������ߣ������� ==========
			std::vector<std::vector<CCVector3d>> allPolylines;
			m_pointCloudDrawer->getPoints(allPolylines);

			// ����û�û�л��κ����ߣ�����ʾ������
			if (allPolylines.empty())
			{
				QMessageBox::warning(nullptr, "��ʾ", "δ��⵽�κ����ߣ��޷�����ģ�塣");
				return;
			}

			// ========== B. ��ȡ��ǰ���������е��ƣ���ִ�вü� ==========
			std::vector<ccPointCloud*> clouds;
			m_objectTree->getAllPointClouds(clouds);

			// ========== C. ׼������·����PCD�� ==========
			QFileInfo jsonInfo(jsonFilePath);
			QString dirPath = jsonInfo.absolutePath();
			QString modelPath = dirPath + "/" + modelName + ".pcd";
			QString outlinePath = dirPath + "/" + modelName + "_outline.pcd";

			// --------- 1. ��ԭʼ�ü�����ƽ���ƽ�滯���������浽 modelPath ---------
			{
				// �����һ���µĵ���
				PCLCloudPtr flatCloud(new pcl::PointCloud<pcl::PointXYZ>);
				*flatCloud = *pclCloud; // ��㸴��ԭʼ PCLCloudPtr

				// �����е�� z ������Ϊ 0
				for (auto& pt : flatCloud->points)
				{
					pt.z = 0.0f;
				}

				// д����̵� modelPath
				if (pcl::io::savePCDFileASCII(modelPath.toStdString(), *flatCloud) < 0)
				{
					QMessageBox::warning(nullptr, "����", "����ƽ�滯��� PCD �ļ�ʧ��: " + modelPath);
					return;
				}
			}

			// --------- 2. ��������������ƣ���������ƽ�滯�����浽 outlinePath ---------
			if (outlineCloud && !outlineCloud->empty())
			{
				PCLCloudPtr flatOutline(new pcl::PointCloud<pcl::PointXYZ>);
				*flatOutline = *outlineCloud; // ���

				// ͬ�����������Ƶ� z ȫ����Ϊ 0
				for (auto& pt : flatOutline->points)
				{
					pt.z = 0.0f;
				}

				if (pcl::io::savePCDFileASCII(outlinePath.toStdString(), *flatOutline) < 0)
				{
					QMessageBox::warning(nullptr, "����", "����ƽ�滯������� PCD �ļ�ʧ��: " + outlinePath);
					return;
				}
			}

			// ========== D. ���� JSON ����д���ļ� ==========
			json modelJson;
			modelJson["_comment"] = "��ģ������������ߣ�ÿ�����ߴ��Ϊһ�� 3D ��������";
			modelJson["name"] = modelName.toStdString();
			modelJson["raw_point_cloud_path"] = modelPath.toStdString();
			modelJson["outline_point_cloud_path"] = outlinePath.toStdString();

			// ֱ�Ӱ����߷��飬ÿ�� polyline ��һ�����б��Ҷ�ͶӰ�� z = 0 ƽ��
			for (const auto& poly : allPolylines)
			{
				json lineArray = json::array();
				for (const auto& cc3d : poly)
				{
					// ��֤���ߵ����ж��� z ���궼Ϊ 0
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

			// д����̣�����ԭ JSON �ļ���
			{
				std::ofstream outputFile(jsonFilePath.toStdString());
				if (!outputFile.is_open())
				{
					QMessageBox::warning(nullptr, "����", "�޷��� JSON �ļ�: " + jsonFilePath);
					return;
				}
				outputFile << j.dump(4); // ���� 4 �ո�
				outputFile.close();
			}

			QMessageBox::information(nullptr, "�ɹ�", "ģ���ѳɹ����浽 JSON �ļ���");
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
		// ����ģʽ�£�������ҲҪ֪ͨ Drawer
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
		// ����ģʽ�£����˫��������ǰ���߻��л�״̬
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
		// ����ģʽ�£��Ҽ�˫��Ҳ�ɽ������˳�����
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
		// ����ģʽ�£�����ƶ����ڸ������߸���
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
		// �����Ҫ������ģʽ�¹�������ʱҲ���Ը������ߵ�ͶӰ
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
		// ����ģʽ�£��������� F/G�����ڽ������߻��˳�����
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
	histogramWidget->setPointCloud(pointCloud, isfilterIntensity); // ���õ�������

	histogramWidget->setUpperAndLowerThreshold(is_has_threshold, lowerThreshold, upperThreshold); // ������ֵ

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
	setHeaderLabel("����Ŀ¼");
	setSelectionMode(QAbstractItemView::ExtendedSelection);
	setContextMenuPolicy(Qt::DefaultContextMenu);

	connect(this, &CloudObjectTreeWidget::async_refresh, this, &CloudObjectTreeWidget::refresh, Qt::QueuedConnection);

	// ��ѡ�����ÿɼ���
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

	// �������ѡ�е���
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
			// if (!object->getParent()) // ֻ�÷������и��ڵ�
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
	// ������Ϊ��intensity���ı����ֶ�
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

	// ����ҵ���ǿ�ȱ����ֶ�
	if (sfIdx >= 0)
	{
		// ���øñ����ֶ���Ϊ��ɫ��ʾ
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);  // ��ʾ�����ֶ�
		cloud->showColors(true);  // ������ɫ��ʾ
	}
	else
	{
		// ���û��ǿ�ȱ����ֶΣ�����Ĭ����Ϊ
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
	expandAll();  // չ��������

	m_glWindow->redraw();

	// �ָ��źŴ���
	blockSignals(false);
}

void CloudObjectTreeWidget::loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem)
{
	// ��������
	QTreeWidgetItem* item = new QTreeWidgetItem(parentItem);
	item->setText(0, object->getName());  // ��������ı�Ϊ���������
	item->setCheckState(0, object->isVisible() ? Qt::Checked : Qt::Unchecked);  // ���ø�ѡ��״̬
	item->setData(0, Qt::UserRole, QVariant::fromValue<void*>(object));  // ������󶨵�����

	// �������־��ʹ��֧���û���ѡ
	item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
	// ����и���ͽ���ǰ����Ϊ������������
	if (parentItem)
	{
		parentItem->addChild(item);
	}
	else
	{
		addTopLevelItem(item);  // ���û�и����Ϊ�������
	}
	item->setSelected(object->isSelected());

	// �ݹ��������
	for (int i = 0; i < object->getChildrenNumber(); ++i)
	{
		ccHObject* child = object->getChild(i);
		if (child)
		{
			child->setDisplay(object->getDisplay()); // ��ͬ������ʾ
			loadTreeItem(child, item);  // ���ӽڵ�ݹ���ص���ǰ����
		}
	}
}

void CloudObjectTreeWidget::contextMenuEvent(QContextMenuEvent* event)
{
	QMenu menu(this);

	QAction* delAct = new QAction("ɾ����ѡ����", &menu);

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
