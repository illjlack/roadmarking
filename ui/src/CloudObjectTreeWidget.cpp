#include "CloudObjectTreeWidget.h"
#include "PointCloudIO.h"
#include <QTreeWidgetItem>
#include <QMenu>
#include <QContextMenuEvent>
#include <QAction>
#include <ccPointCloud.h>
#include <ccHObject.h>
#include <ccGLWindowInterface.h>
#include <ccMainAppInterface.h>
#include <ccScalarField.h>
#include <QString>
#include <QDebug>
#include "CloudProcess.h"
// ============================================================================ CloudObjectTreeWidget

using namespace roadmarking;

CloudObjectTreeWidget::CloudObjectTreeWidget(QWidget* parent)
	: QTreeWidget(parent)
{
	setColumnCount(1);
	setHeaderLabel("对象目录");
	setSelectionMode(QAbstractItemView::ExtendedSelection);
	setContextMenuPolicy(Qt::DefaultContextMenu);

	connect(this, &CloudObjectTreeWidget::async_refresh, this, &CloudObjectTreeWidget::refresh, Qt::QueuedConnection);

	// 复选框设置可见性
	connect(this, &QTreeWidget::itemChanged, this, [&](QTreeWidgetItem* item, int column)
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

			if (*pp_select_cloud)(*pp_select_cloud)->setSelected(false);

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
	// 使用PointCloudIO的标准方法来设置强度标量字段显示
	PointCloudIO::apply_intensity(cloud);

	cloud->setEnabled(true);
	cloud->setVisible(true);

	if (parent)
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

void CloudObjectTreeWidget::loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem, bool isFold)
{

	QTreeWidgetItem* item = nullptr;
	if (!isFold)
	{
		// 创建树项
		item = new QTreeWidgetItem(parentItem);
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
	}

	// 如果是图元，后面的内容在目录中不显示
	if (dynamic_cast<MetaRoadmarking*>(object))
	{
		isFold = true;
	}

	// 递归加载子项
	for (int i = 0; i < object->getChildrenNumber(); ++i)
	{
		ccHObject* child = object->getChild(i);
		if (child)
		{
			child->setDisplay(object->getDisplay()); // 在同窗口显示
			loadTreeItem(child, item, isFold);  // 将子节点递归挂载到当前项下
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
