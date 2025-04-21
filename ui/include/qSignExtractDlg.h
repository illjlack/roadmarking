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

class CloudBackup;
class CloudObjectTreeWidget;
class ForegroundPolylineEditor;

// 选择模式枚举
enum SelectionMode
{
	ENTITY_SELECTION,  // 实体选择状态
	POINT_SELECTION,   // 点选择模式
	DRAW_SELECTION     // 多边形框选模式
};

class qSignExtractDlg : public QDialog
{
	Q_OBJECT

public:
	qSignExtractDlg(ccMainAppInterface* app);
	~qSignExtractDlg();

	/// <summary>
	/// 设置目标点云
	/// </summary>
	/// <param name="cloud">目标点云</param>
	/// <returns>是否成功</returns>
	bool setCloud(ccCloudPtr cloud);

protected:


	
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
	void onMouseWheelRotated(int delta);

private:
	bool m_selecting = false;

	ccHObject* p_select_cloud = nullptr;

	QWidget* m_glWidget = nullptr;
	ccMainAppInterface* m_app = nullptr;
	ccGLWindowInterface* m_glWindow = nullptr;
	
	std::shared_ptr<CloudBackup> m_cloudBackup;
	CloudObjectTreeWidget* m_objectTree = nullptr;
	SelectionMode m_selectionMode = ENTITY_SELECTION;



	ForegroundPolylineEditor* foregroundPolylineEditor;


};

// ================================================================== ForegroundPolylineEditor
class ForegroundPolylineEditor: public QObject
{
	Q_OBJECT
public:
	// 构造函数，传入gl窗口、点云对象等参数
	ForegroundPolylineEditor(ccGLWindowInterface* glWindow);

	// 析构函数，销毁时清理资源
	~ForegroundPolylineEditor();

	void setSelectCloudPtr(ccHObject** select_cloud);
	void setCallbackfunc(std::function<void()> callback);
	void startDraw();

	void onLeftButtonClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onMouseWheelRotated(int delta);

private:
	void updatePoly();
	void finishDraw();

	ccGLWindowInterface* m_glWindow;             // 用于显示的OpenGL窗口
	ccPointCloud* m_pointCloud;                  // 用于绑定折线的点云对象(2D屏幕的坐标)
	ccPolyline* m_foregroundPolyline;            // 前景折线对象
	std::vector<CCVector3d> m_3DPoints;		 // 用于保存点击位置

	// 备份标记
	ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
	ccGLWindowInterface::PICKING_MODE picking_mode_backup;

	ccHObject** pp_select_cloud = nullptr;
	std::function<void()> m_callback;            // 回调函数
signals:
	void draw_start();
	void draw_finish();
};


/// <summary>
/// 对象目录树，用于选中点云，设置点云可见性，删除点云
/// </summary>
class CloudObjectTreeWidget : public QTreeWidget
{
	Q_OBJECT

public:
	explicit CloudObjectTreeWidget(QWidget * parent = nullptr);

	/// <summary>
	/// 初始化，绑定窗口，根节点点云
	/// </summary>
	/// <param name="win">窗口接口</param>
	/// <param name="app">主程序接口</param>
	/// <param name="select_cloud">根节点点云</param>
	void initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud);

	/// <summary>
	/// 刷新目录
	/// </summary>
	void refresh();
protected:
	void contextMenuEvent(QContextMenuEvent* event) override;
	void loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem);
private:
	ccHObject* root = nullptr;
	ccHObject** pp_select_cloud = nullptr;
	ccMainAppInterface* m_app = nullptr;
	ccGLWindowInterface* m_glWindow = nullptr;
};


/// <summary>
/// 点云状态备份与恢复工具类
/// 用于插件中临时修改点云属性后，支持一键还原(现在主要是设置自己及所有孩子的display属性)
/// </summary>
class CloudBackup
{
public:
	CloudBackup();
	~CloudBackup();

	/// <summary>
	/// 备份点云状态
	/// </summary>
	/// <param name="cloud">备份目标点云</param>
	void backup(ccCloudPtr cloud);

	/// <summary>
	/// 恢复点云状态
	/// </summary>
	void restore();

	/// <summary>
	/// 清理备份
	/// </summary>
	void clear();

private:
	int displayedSFIndex;
	bool wasVisible;
	bool wasEnabled;
	bool wasSelected;
	bool colorsWereDisplayed;
	bool sfWasDisplayed;

	ccCloudPtr ref;
	ccGenericGLDisplay* originalDisplay;
};
