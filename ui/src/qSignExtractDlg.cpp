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

#include "CloudProcess.h"
#include "RoadMarkingExtract.h"

using namespace roadmarking;



/// <summary>
/// ��ȡ������ƽ�滯
/// </summary>
/// <param name="ccCloud">ԭʼCC����</param>
/// <returns>zֵΪ0�ĵ������, �̱߳�����"elevation"����</returns>
ccCloudPtr get_ground_xy_cloud(ccCloudPtr ccCloud)
{
	//float targetVoxelSize = 0.2;
	//float euclideanClusterRadius = 0.3;
	//float curvatureBaseThreshold = 0.01;
	//float angleDegrees = 5.0;
	//float angleThreshold = cos(angleDegrees * M_PI / 180.0);  // �Ƕ�ת�����ټ�������
	//float searchRadius = 0.3;
	//float gridSize = 0.05;


	//PCLCloudPtr pclCloud = PointCloudIO::convertToPCLCloud(ccCloud);
	//PCLCloudPtr voxelCloud = CloudProcess::applyVoxelGridFiltering(pclCloud, targetVoxelSize);
	//ccCloudPtr groundCloud = CloudProcess::applyCSFGroundExtraction(PointCloudIO::convertToCCCloud(voxelCloud));
	//PCLCloudPtr largestClusterCloud = CloudProcess::extractMaxCloudByEuclideanCluster(PointCloudIO::convertToPCLCloud(groundCloud), euclideanClusterRadius);
	//PCLCloudPtr roadCloud = CloudProcess::extractRoadPoints(largestClusterCloud, searchRadius, angleThreshold, curvatureBaseThreshold);
	//ccCloudPtr oRoadCloud = CloudProcess::CropBySparseCloud(ccCloud, roadCloud);

	ccCloudPtr groundCloud = CloudProcess::applyCSFGroundExtraction(ccCloud);

	ccCloudPtr groundCloud_xy(new ccPointCloud);

	size_t scalarFieldCount = groundCloud->getNumberOfScalarFields();
	for (size_t fieldIdx = 0; fieldIdx < scalarFieldCount; ++fieldIdx)
	{
		const std::string sfName = groundCloud->getScalarField(fieldIdx)->getName();
		groundCloud_xy->addScalarField(sfName);
	}
	int elevationSfIdx = groundCloud_xy->addScalarField("Elevation");

	for (size_t i = 0; i < groundCloud->getPointSize(); i++)
	{
		CCVector3 point = *groundCloud->getPoint(i);
		ScalarType elev = point.z;
		point.z = 0;
		groundCloud_xy->addPoint(point);
		for (size_t fieldIdx = 0; fieldIdx < scalarFieldCount; ++fieldIdx)
		{
			ScalarType val = groundCloud->getScalarField(fieldIdx)->getValue(i);
			groundCloud_xy->getScalarField(fieldIdx)->addElement(val);
		}
		groundCloud_xy->getScalarField(elevationSfIdx)->addElement(elev);
	}

	for (size_t fieldIdx = 0; fieldIdx < groundCloud_xy->getNumberOfScalarFields(); ++fieldIdx)
	{
		groundCloud_xy->getScalarField(fieldIdx)->computeMinAndMax();
	}
	return groundCloud_xy;
}

// =========================================================================================================================== qSignExtractDlg
qSignExtractDlg::qSignExtractDlg(ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, m_app(app)
	, m_cloudBackup(new CloudBackup())
{
	setWindowTitle("·�����ʶ��");
	resize(1200, 800);

	QHBoxLayout* mainLayout = new QHBoxLayout(this);
	QVBoxLayout* leftLayout = new QVBoxLayout();

	m_glWidget = nullptr;
	if (m_app)
		m_app->createGLWindow(m_glWindow, m_glWidget);

	// ==================================== ����Ŀ¼
	{
		QGroupBox* objectGroup = new QGroupBox("����Ŀ¼", this);
		m_objectTree = new CloudObjectTreeWidget(objectGroup);
		m_objectTree->initialize(m_glWindow, m_app, &p_select_cloud);

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

		connect(this, &qSignExtractDlg::enableButtons, [viewGroup]()
			{
				for (int i = 0; i < viewGroup->layout()->count(); ++i) {
					QToolButton* btn = qobject_cast<QToolButton*>(viewGroup->layout()->itemAt(i)->widget());
					if (btn) {
						btn->setEnabled(true);
					}
				}
			});

		connect(this, &qSignExtractDlg::disableButtons, [viewGroup]()
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

		addCheckableButton("ȫ�Զ���ȡ", [this]() { onAutoExtract(); });
		addCheckableButton("��ѡ��ȡ", [this]() { onBoxSelectExtract(); });
		addCheckableButton("��ѡ������ȡ", [this]() { onPointGrowExtract(); });
		addCheckableButton("��ѡ��ȡ����", [this]() { onBoxClip(); });

		functionGroup->setLayout(functionLayout);
		leftLayout->addWidget(functionGroup);


		connect(this, &qSignExtractDlg::enableButtons, [buttonGroup]()
			{
				for (auto* btn : buttonGroup->buttons())
				{
					btn->setEnabled(true);
				}
			});

		connect(this, &qSignExtractDlg::disableButtons, [buttonGroup]()
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

		connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::entitySelectionChanged, this, &qSignExtractDlg::onEntitySelectionChanged);
		connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::itemPicked, this, &qSignExtractDlg::onItemPicked);
		connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::itemPickedFast, this, &qSignExtractDlg::onItemPickedFast);

		connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::leftButtonClicked, this, &qSignExtractDlg::onLeftButtonClicked);
		connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::mouseMoved, this, &qSignExtractDlg::onMouseMoved);
		connect(m_glWindow->signalEmitter(), &ccGLWindowSignalEmitter::buttonReleased, this, &qSignExtractDlg::onButtonReleased);
	}
}
qSignExtractDlg::~qSignExtractDlg()
{
	if (m_glWindow)
	{
		m_glWindow->getOwnDB()->removeAllChildren();
		m_cloudBackup->restore();

		if (m_app)
		{
			m_app->destroyGLWindow(m_glWindow);
			m_glWindow = nullptr;
		}
	}
}

bool qSignExtractDlg::setCloud(ccCloudPtr cloud)
{
	if (!cloud || !m_glWindow || !m_app)
		return false;
	if (cloud->size() < 10)
	{
		ccLog::Error("��������̫�٣�");
		return false;
	}

	m_cloudBackup->backup(cloud);

	if (!cloud->hasColors())
	{
		bool success = cloud->hasDisplayedScalarField() ?
			cloud->convertCurrentScalarFieldToColors() :
			cloud->setColor(ccColor::white);

		if (!success)
		{
			ccLog::Error("��ɫ����ʧ�ܣ�");
			return false;
		}
	}

	cloud->convertRGBToGreyScale();
	cloud->setEnabled(true);
	cloud->setVisible(true);
	cloud->setSelected(false);
	cloud->showColors(true);
	cloud->showSF(false);
	m_glWindow->addToOwnDB(cloud.release());

	ccBBox bbox = cloud->getOwnBB();
	m_glWindow->updateConstellationCenterAndZoom(&bbox);
	m_glWindow->redraw();

	m_objectTree->refresh();
	return true;
}

CCVector3 qSignExtractDlg::screenToWorld(int x, int y)
{
	CCVector3d A, B;
	QPointF glPos = m_glWindow->toCornerGLCoordinates(x, y);
	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);
	camera.unproject(CCVector3(glPos.x(), glPos.y(), 0.0f), A);
	camera.unproject(CCVector3(glPos.x(), glPos.y(), 1.0f), B);
	CCVector3d dir = B - A;
	double t = -A.z / dir.z;
	return (A + t * dir).toPC();
}

void qSignExtractDlg::onAutoExtract()
{
	RoadMarkingExtract::automaticExtraction(PointCloudIO::convertToCCCloud(p_select_cloud), m_app);
	m_objectTree->refresh();
}

void qSignExtractDlg::onBoxSelectExtract()
{
	startDraw();
	m_selectionMode = POINT_SELECTION;
	// �̶�Ϊ�������µ�������ͼ����ʼΪ1��1��С����ֹ�����ת
	m_glWindow->setView(CC_TOP_VIEW);
	if (p_select_cloud)
	{
		ccBBox bbox = p_select_cloud->getOwnBB();
		m_glWindow->updateConstellationCenterAndZoom(&bbox);

		{
			ccBBox bbox = p_select_cloud->getOwnBB(true);
			// ��ȡ��Χ�е��ĸ���
			CCVector3 p0 = bbox.maxCorner();
			CCVector3 p1 = bbox.minCorner();
			p0.x += 5; p0.y += 5;
			p1.x -= 5; p1.y -= 5;
			// ����ͶӰ�� 2D ƽ�棨XY��������
			// ��������ֻ���� 2D ��Χ�У����Խ� Z ��������Ϊ 0
			CCVector3 p0_2D(p0.x, p0.y, 0);
			CCVector3 p1_2D(p1.x, p0.y, 0);  // ���½ǵ����½�
			CCVector3 p2_2D(p1.x, p1.y, 0);  // ���½ǵ����Ͻ�
			CCVector3 p3_2D(p0.x, p1.y, 0);  // ���Ͻǵ����Ͻ�

			// ����һ���µ� ccPolyline ����
			ccPointCloud* polylineCloud = new ccPointCloud("2D Bounding Box");

			// ��Ӱ�Χ�е��ĸ��ǵ㵽 polylineCloud
			polylineCloud->addPoint(p0_2D);
			polylineCloud->addPoint(p1_2D);
			polylineCloud->addPoint(p2_2D);
			polylineCloud->addPoint(p3_2D);


			// ����һ�� ccPolyline ����
			ccPolyline* boundingBox = new ccPolyline(polylineCloud);


			// Ԥ���ռ��������ߵ�����
			boundingBox->reserve(static_cast<unsigned>(polylineCloud->size()));

			// �������е�ÿ������ӵ� ccPolyline ��
			for (size_t i = 0; i < polylineCloud->size(); ++i)
			{
				boundingBox->addPointIndex(static_cast<unsigned>(i));
			}

			/*
			m_segmentationPoly = new ccPolyline(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
			m_segmentationPoly->setForeground(true);
			m_segmentationPoly->setColor(ccColor::green);
			m_segmentationPoly->showColors(true);
			m_segmentationPoly->set2DMode(true);
			*/

			// ������ɫ
			//boundingBox->setColor(ccColor::red); // ��������Ϊ��ɫ
			//boundingBox->showColors(true);
			//boundingBox->setWidth(1.0);
			//boundingBox->set2DMode(true);
			//boundingBox->setForeground(true);

			boundingBox->setForeground(true);
			boundingBox->setColor(ccColor::green);
			boundingBox->showColors(true);
			boundingBox->set2DMode(true);

			p_select_cloud->addChild(polylineCloud);
			p_select_cloud->addChild(boundingBox);
			m_objectTree->refresh();
		}

	}
	m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_PAN_ONLY);
	m_glWindow->setPickingMode(ccGLWindowInterface::FAST_PICKING);

	
}

void qSignExtractDlg::onPointGrowExtract()
{
	
}

void qSignExtractDlg::onBoxClip()
{
	
}



void qSignExtractDlg::onItemPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3&, const CCVector3d&)
{

}

// ���ٵ���������������л���(�̶�������ͼ����ά���񱣴����߳�)
void qSignExtractDlg::onItemPickedFast(ccHObject* entity, int subEntityID, int x, int y)
{
	
	
}

void qSignExtractDlg::onLeftButtonClicked(int x, int y)
{
	m_glWindow->redraw();
}

void qSignExtractDlg::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	m_glWindow->redraw();
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
}

void qSignExtractDlg::startDraw()
{
	interaction_flags_backup = m_glWindow->getInteractionMode();
	picking_mode_backup = m_glWindow->getPickingMode();
	emit disableButtons();
}

void qSignExtractDlg::finishDraw()
{
	m_glWindow->setInteractionMode(interaction_flags_backup);
	m_glWindow->setPickingMode(picking_mode_backup);
	emit disableButtons();
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

void CloudObjectTreeWidget::initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud)
{
	m_glWindow = win;
	m_app = app;
	pp_select_cloud = select_cloud;
	refresh();
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

// ============================================================================ CloudBackup
CloudBackup::CloudBackup()
	: ref(nullptr)
	, originalDisplay(nullptr)
	, wasVisible(true)
	, wasEnabled(true)
	, wasSelected(false)
	, colorsWereDisplayed(false)
	, sfWasDisplayed(false)
	, displayedSFIndex(-1)
{
}

CloudBackup::~CloudBackup()
{
	clear();
}

void CloudBackup::backup(ccCloudPtr cloud)
{
	ref = cloud;
	if (!cloud)
		return;

	originalDisplay = cloud->getDisplay();
	wasVisible = cloud->isVisible();
	wasEnabled = cloud->isEnabled();
	wasSelected = cloud->isSelected();
	colorsWereDisplayed = cloud->colorsShown();
	sfWasDisplayed = cloud->sfShown();
	displayedSFIndex = cloud->getCurrentDisplayedScalarFieldIndex();
}

void CloudBackup::restore()
{
	if (!ref)
		return;

	ref->setDisplay(originalDisplay);
	ref->setVisible(wasVisible);
	ref->setEnabled(wasEnabled);
	ref->setSelected(wasSelected);
	ref->showColors(colorsWereDisplayed);
	ref->showSF(sfWasDisplayed);
	ref->setCurrentDisplayedScalarField(displayedSFIndex);

	std::function<void(ccHObject*)> dfsChild = [&](ccHObject* parent)
	{
		if (!parent)
			return;
		for (int i = 0; i < parent->getChildrenNumber(); i++)
		{
			ccHObject* child = parent->getChild(i);
			if (child)
			{
				child->setDisplay(parent->getDisplay());
				dfsChild(child);
			}
		}
	};

	dfsChild(ref.get());
}

void CloudBackup::clear()
{
	ref = nullptr;
}
