#pragma once
#pragma execution_character_set("utf-8")

#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QContextMenuEvent>
#include <QVariant>
#include <vector>
#include "ccGLWindowInterface.h"
#include "ccMainAppInterface.h"
#include "ccHObject.h"
#include "ccPointCloud.h"
#include "ccGenericGLDisplay.h"

/// <summary>
/// CloudObjectTreeWidget 类用于管理点云对象目录树
/// </summary>
class CloudObjectTreeWidget : public QTreeWidget
{
	Q_OBJECT

public:
	explicit CloudObjectTreeWidget(QWidget* parent = nullptr);

	/// <summary>
	/// 初始化目录树，绑定窗口和根节点点云
	/// </summary>
	void initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud, const std::vector<ccHObject*>& objects);

	void addCloud(ccPointCloud* cloud, ccHObject* parent = nullptr);  // 添加点云

	void relase();	 // 释放点云到原窗口

	void getAllPointClouds(std::vector<ccPointCloud*>& pointClouds);  // 获取所有点云

signals:
	void async_refresh();

public slots:
	void refresh();

protected:
	void loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem, bool isFold = false);

	void contextMenuEvent(QContextMenuEvent* event) override;

	void getAllPointCloudsRecursive(ccHObject* object, std::vector<ccPointCloud*>& pointClouds); // 递归获取所有点云

private:
	ccHObject* root = nullptr;  // 根节点
	ccHObject** pp_select_cloud = nullptr;  // 选择的点云对象指针
	ccMainAppInterface* m_app = nullptr;  // 主程序接口
	ccGLWindowInterface* m_glWindow = nullptr;  // OpenGL 窗口接口

	ccGenericGLDisplay* originalDisplay;  // 原始显示对象
};
