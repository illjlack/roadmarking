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
	setWindowTitle("·�����ʶ��");
	resize(1200, 800);
	setFocusPolicy(Qt::StrongFocus);

	QHBoxLayout* mainLayout = new QHBoxLayout(this);
	QVBoxLayout* leftLayout = new QVBoxLayout();

	m_glWidget = nullptr;
	if (m_app)
		m_app->createGLWindow(m_glWindow, m_glWidget);

	// ==================================== ǰ��������
	m_foregroundPolylineEditor = new ForegroundPolylineEditor(m_glWindow);
	m_foregroundPolylineEditor->setSelectCloudPtr(&p_select_cloud);


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



	// ==================================== ���ܰ�ť��
	{
		QGroupBox* functionGroup = new QGroupBox("����ѡ��", this);
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

		addCheckableButton("ѡ��ǿ�ȹ��˵���", [this]() { onFilteCloud(); });
		addCheckableButton("ȫ�Զ���ȡ", [this]() { onAutoExtract(); });
		addCheckableButton("��ѡ��ȡ", [this]() { onBoxSelectExtract(); });
		addCheckableButton("��ѡ������ȡ", [this]() { onPointGrowExtract(); });
		addCheckableButton("��ѡ��ȡ����", [this]() { onBoxClip(); });

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
	RoadMarkingExtract::automaticExtraction(PointCloudIO::convertToCCCloud(p_select_cloud), m_app);
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
			CloudProcess::cropPointCloudWithFineSelection(clouds, polyline, cloud);

			cloud->setName("cloud_cropped");
			m_glWindow->addToOwnDB(cloud);
			m_objectTree->refresh();

			if (p_select_cloud)p_select_cloud->setSelected(false);
			p_select_cloud = cloud;

			RoadMarkingExtract::automaticExtraction(PointCloudIO::convertToCCCloud(p_select_cloud), m_app);
			m_objectTree->refresh();
		});
	m_objectTree->refresh();
}

void qSignExtractDlg::onPointGrowExtract()
{
	m_foregroundPolylineEditor->startDraw();
	m_foregroundPolylineEditor->setDraw(2, false);
	m_foregroundPolylineEditor->setCallbackfunc([&]
		{
			std::vector<CCVector3d> polyline;
			m_foregroundPolylineEditor->getPoints(polyline);


		});
	m_objectTree->refresh();
}

void qSignExtractDlg::onFilteCloud()
{
	if (!p_select_cloud || !dynamic_cast<ccPointCloud*>(p_select_cloud))
	{
		ccLog::Error("δѡ�����");
		return;
	}

	ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(p_select_cloud);

	showThresholdHistogram(cloud);
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
			CloudProcess::cropPointCloudWithFineSelection(clouds, polyline, cloud);

			cloud->setName("cloud_cropped");
			m_glWindow->addToOwnDB(cloud);
			m_objectTree->refresh();

		});
	m_objectTree->refresh();
}

void qSignExtractDlg::addCloudToDB(ccPointCloud* cloud)
{
	m_glWindow->addToOwnDB(cloud);
	m_objectTree->refresh();
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
}

// Ӧ���Ǻܶ��������������ݼ������ˣ�ֻ�в��ּ���Ч
void qSignExtractDlg::keyPressEvent(QKeyEvent* event)
{
	if (m_selectionMode == DRAW_SELECTION)
	{
		m_foregroundPolylineEditor->onKeyPressEvent(event);
	}
}

void qSignExtractDlg::showThresholdHistogram(ccPointCloud* pointCloud, bool is_has_threshold, float lowerThreshold, float upperThreshold)
{
	histogramWidget->setPointCloud(pointCloud); // ���õ�������

	histogramWidget->setUpperAndLowerThreshold(is_has_threshold, lowerThreshold, upperThreshold); // ������ֵ

	histogramWidget->show();
}


// ============================================================================ ForegroundPolylineEditor

ForegroundPolylineEditor::ForegroundPolylineEditor(ccGLWindowInterface* glWindow)
	: m_glWindow(glWindow), m_pointCloud(new ccPointCloud()), m_foregroundPolyline(new ccPolyline(m_pointCloud))
{
	m_foregroundPolyline->addChild(m_pointCloud);
	m_foregroundPolyline->setColor(ccColor::green);
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
		delete m_pointCloud;  // ɾ���ڲ������ĵ���
		m_pointCloud = nullptr;
	}
	// �ͷ���Դ
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
	// ���ڵ�׼��
	{
		// ���ݽ�����ʽ������ʱ�ָ�������������ť
		interaction_flags_backup = m_glWindow->getInteractionMode();
		picking_mode_backup = m_glWindow->getPickingMode();
		emit draw_start();

		// �̶�Ϊ�������µ�������ͼ����ֹ�����ת
		m_glWindow->setView(CC_TOP_VIEW);
		//if (pp_select_cloud && *pp_select_cloud)
		//{
		//	ccBBox bbox = (*pp_select_cloud)->getOwnBB();
		//	m_glWindow->updateConstellationCenterAndZoom(&bbox);
		//}
		m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_PAN_ONLY | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
		m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);
	}

	// ���ݵ�׼��
	{
		// ����������ߣ���ɾ����
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
	if (isClosed)closeLine();

	if (doAction && m_callback)
	{
		// ִ�������ڼ���ʾ�պ�����
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
	// ���ûص�����
	m_callback = callback;
}

void ForegroundPolylineEditor::closeLine()
{
	if (m_pointCloud->size())
	{
		CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 1));
		*lastPoint = *m_pointCloud->getPoint(0);
		m_3DPoints.push_back(m_3DPoints[0]);
		m_glWindow->redraw();
		QApplication::processEvents();
	}
}

void ForegroundPolylineEditor::onLeftButtonClicked(int x, int y)
{
	if (isFreezeUI)return;
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	if (!m_pointCloud->size())
	{
		m_pointCloud->addPoint(newPoint);
		m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1));
	}

	// ��һ����̶�
	{
		CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 1));
		*lastPoint = newPoint;
		ccGLCameraParameters camera;
		m_glWindow->getGLCameraParameters(camera);

		// ������ͬ ForegroundPolylineEditor::updatePoly()
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
	m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1));
	m_glWindow->redraw(true, false);
}

void ForegroundPolylineEditor::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (isFreezeUI)return;

	if (!m_pointCloud->size())
		return;

	// �϶�����2D��Ļ�ϵĵ�
	if (button == Qt::RightButton)
	{
		updatePoly();
	}

	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 1));
	*lastPoint = newPoint;
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

	// ��������(viewport[0] + viewport[2]/2, viewport[1] + viewport[3]/2)��gl��Ļ�������Ͻǣ� ��Ҫ�Ƶ�����
	// ��֪���ڲ��Ĺ���Ϊʲô����������Ľ��
	// �ֶ�����
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
	setHeaderLabel("����Ŀ¼");
	setSelectionMode(QAbstractItemView::ExtendedSelection);
	setContextMenuPolicy(Qt::DefaultContextMenu);

	// ��ѡ�����ÿɼ���
	connect(this, &QTreeWidget::itemChanged, this, [=](QTreeWidgetItem * item, int column)
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
		obj->setVisible(visible);

		if (m_glWindow)
			m_glWindow->redraw();
	});

	// �������ѡ�е���
	connect(this, &QTreeWidget::itemClicked, this, [=](QTreeWidgetItem* item, int column)
	{
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

		for (auto* item : selItems)
		{
			auto obj = static_cast<ccHObject*>(item->data(0, Qt::UserRole).value<void*>());
			if (!obj || obj == root) // ���ڵ㲻�ܱ�ɾ��  
				continue;
			if (*pp_select_cloud == obj)
			{
				*pp_select_cloud = nullptr;
			}
			m_app->removeFromDB(obj);
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

	// �ݹ�������������еĵ��ƶ���
	for (int i = 0; i < dbRoot->getChildrenNumber(); ++i)
	{
		ccHObject* child = dbRoot->getChild(i);
		if (child)
		{
			// �����ݹ�����Ӷ���
			getAllPointCloudsRecursive(child, pointClouds);
		}
	}
}

void CloudObjectTreeWidget::getAllPointCloudsRecursive(ccHObject* object, std::vector<ccPointCloud*>& pointClouds)
{
	// ���ö����Ƿ�ΪccPointCloud����
	if (ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(object))
	{
		if (cloud->isVisible())
			pointClouds.push_back(cloud);  // ���õ��ƶ�����ӵ����������
	}

	// �ݹ�����Ӷ���
	for (int i = 0; i < object->getChildrenNumber(); ++i)
	{
		ccHObject* child = object->getChild(i);
		if (child)
		{
			// �����ݹ�����Ӷ���
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

		// �ָ�״̬
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
