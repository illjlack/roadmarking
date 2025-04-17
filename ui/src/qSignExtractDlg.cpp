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
/// 提取地面且平面化
/// </summary>
/// <param name="ccCloud">原始CC点云</param>
/// <returns>z值为0的地面点云, 高程保存在"elevation"标量</returns>
ccCloudPtr get_ground_xy_cloud(ccCloudPtr ccCloud)
{
	//float targetVoxelSize = 0.2;
	//float euclideanClusterRadius = 0.3;
	//float curvatureBaseThreshold = 0.01;
	//float angleDegrees = 5.0;
	//float angleThreshold = cos(angleDegrees * M_PI / 180.0);  // 角度转弧度再计算余弦
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
	setWindowTitle("路标点云识别");
	resize(1200, 800);

	QHBoxLayout* mainLayout = new QHBoxLayout(this);
	QVBoxLayout* leftLayout = new QVBoxLayout();

	m_glWidget = nullptr;
	if (m_app)
		m_app->createGLWindow(m_glWindow, m_glWidget);

	// ==================================== 对象目录
	{
		QGroupBox* objectGroup = new QGroupBox("对象目录", this);
		m_objectTree = new CloudObjectTreeWidget(objectGroup);
		m_objectTree->initialize(m_glWindow, m_app, &p_select_cloud);

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

		addCheckableButton("全自动提取", [this]() { onAutoExtract(); });
		addCheckableButton("框选提取", [this]() { onBoxSelectExtract(); });
		addCheckableButton("点选生长提取", [this]() { onPointGrowExtract(); });
		addCheckableButton("框选截取点云", [this]() { onBoxClip(); });

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
		ccLog::Error("点云数据太少！");
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
			ccLog::Error("颜色处理失败！");
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
	// 固定为从上往下的正交视图，初始为1：1大小，禁止相机旋转
	m_glWindow->setView(CC_TOP_VIEW);
	if (p_select_cloud)
	{
		ccBBox bbox = p_select_cloud->getOwnBB();
		m_glWindow->updateConstellationCenterAndZoom(&bbox);

		{
			ccBBox bbox = p_select_cloud->getOwnBB(true);
			// 获取包围盒的四个角
			CCVector3 p0 = bbox.maxCorner();
			CCVector3 p1 = bbox.minCorner();
			p0.x += 5; p0.y += 5;
			p1.x -= 5; p1.y -= 5;
			// 计算投影到 2D 平面（XY）的坐标
			// 由于我们只关心 2D 包围盒，所以将 Z 坐标设置为 0
			CCVector3 p0_2D(p0.x, p0.y, 0);
			CCVector3 p1_2D(p1.x, p0.y, 0);  // 左下角到右下角
			CCVector3 p2_2D(p1.x, p1.y, 0);  // 右下角到右上角
			CCVector3 p3_2D(p0.x, p1.y, 0);  // 右上角到左上角

			// 创建一个新的 ccPolyline 对象
			ccPointCloud* polylineCloud = new ccPointCloud("2D Bounding Box");

			// 添加包围盒的四个角点到 polylineCloud
			polylineCloud->addPoint(p0_2D);
			polylineCloud->addPoint(p1_2D);
			polylineCloud->addPoint(p2_2D);
			polylineCloud->addPoint(p3_2D);


			// 创建一条 ccPolyline 对象
			ccPolyline* boundingBox = new ccPolyline(polylineCloud);


			// 预留空间用于折线点索引
			boundingBox->reserve(static_cast<unsigned>(polylineCloud->size()));

			// 将折线中的每个点添加到 ccPolyline 中
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

			// 设置颜色
			//boundingBox->setColor(ccColor::red); // 例如设置为红色
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

// 快速点击，可以用来进行绘制(固定正交视图，二维网格保存地面高程)
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
	setHeaderLabel("对象目录");
	setSelectionMode(QAbstractItemView::ExtendedSelection);
	setContextMenuPolicy(Qt::DefaultContextMenu);

	// 复选框设置可见性
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

	// 点击设置选中点云
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
	expandAll();  // 展开所有项
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

		for (auto* item : selItems)
		{
			auto obj = static_cast<ccHObject*>(item->data(0, Qt::UserRole).value<void*>());
			if (!obj || obj == root) // 根节点不能被删除  
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
