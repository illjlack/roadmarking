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
#include <QWidget>
#include <QPainter>
#include <QButtonGroup>
#include <QTreeWidget>
#include <QMouseEvent>
#include <QEvent>
#include "ccGLWindow.h"
#include "ccGLWindowSignalEmitter.h"
#include "ccMainAppInterface.h"
#include "ccBox.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#include "comm.h"

/// <summary>
/// 点云状态备份与恢复工具类
/// 用于插件中临时修改点云属性后，支持一键还原
/// </summary>
struct CloudBackup
{
	CloudBackup();
	~CloudBackup();

	/// 备份状态
	void backup(ccCloudPtr cloud);

	/// 恢复状态
	void restore();

	/// 清理资源
	void clear();

private:
	ccCloudPtr ref;

	// 原始状态备份
	ccGenericGLDisplay* originalDisplay;
	bool wasVisible;
	bool wasEnabled;
	bool wasSelected;
	bool colorsWereDisplayed;
	bool sfWasDisplayed;
	int displayedSFIndex;
	bool hadColors;

	RGBAColorsTableType* backupColors;
};


class CloudObjectTreeWidget;

// 选择模式枚举
enum SelectionMode
{
	NO_SELECTION,      // 非选择状态
	POINT_SELECTION,   // 点选择模式
	RECT_SELECTION,    // 矩形框选模式
	POLY_SELECTION     // 多边形框选模式
};

class qSignExtractDlg : public QDialog
{
	Q_OBJECT

public:
	qSignExtractDlg(ccMainAppInterface* app);
	~qSignExtractDlg();

	/// 设置并显示目标点云
	bool setCloud(ccCloudPtr cloud);

protected:


	// 屏幕坐标转世界坐标（z=0 平面）
	CCVector3 screenToWorld(int x, int y);

private slots:
	void onAutoExtract();        // 全自动提取
	void onBoxSelectExtract();   // 框选提取
	void onPointGrowExtract();   // 点生长提取
	void onBoxClip();            // 框选截取

	void onItemPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3&, const CCVector3d&);
	void onItemPickedFast(ccHObject* entity, int subEntityID, int x, int y);
	void onLeftButtonClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onButtonReleased();
	void onEntitySelectionChanged(ccHObject* entity);
private:
	ccMainAppInterface* m_app = nullptr;
	ccGLWindowInterface* m_glWindow = nullptr;
	QWidget* m_glWidget = nullptr;

	// 目录树 
	CloudObjectTreeWidget* m_objectTree = nullptr;

	// 点云备份
	CloudBackup m_cloudBackup;

	SelectionMode m_selectionMode = POLY_SELECTION;

	bool m_selecting = false;
	QPoint m_selectionStart;
	QPoint m_mousePos;


	ccHObject* p_select_cloud = nullptr;
};


/// <summary>
/// 内部类：对象目录树，用于选中点云，设置点云可见性，删除点云
/// </summary>
class CloudObjectTreeWidget : public QTreeWidget
{
	Q_OBJECT

public:
	explicit CloudObjectTreeWidget(QWidget * parent = nullptr);

	/// 初始化目录树（绑定窗口和 app）
	void initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud);

	// 刷新目录（重新加载树）
	void refresh();


protected:
	// 右键菜单, 提供删除选项
	void contextMenuEvent(QContextMenuEvent* event) override;
	// 迭代的加载树的目录
	void loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem);
private:
	ccGLWindowInterface* m_glWindow = nullptr;
	ccMainAppInterface* m_app = nullptr;
	ccHObject** pp_select_cloud = nullptr;
	ccHObject* root = nullptr;
};
