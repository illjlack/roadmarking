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
#include "CloudFilterDlg.h"

#define INF 0x3f3f3f3f

// 前向声明
class CloudObjectTreeWidget;
class ForegroundPolylineEditor;

// 选择模式枚举
enum SelectionMode
{
	ENTITY_SELECTION,  // 实体选择状态
	POINT_SELECTION,   // 点选择模式
	DRAW_SELECTION     // 多边形框选模式
};

/// <summary>
/// qSignExtractDlg 类用于点云提取对话框处理
/// </summary>
class qSignExtractDlg : public QDialog
{
	Q_OBJECT

public:
	qSignExtractDlg(ccMainAppInterface* app);
	~qSignExtractDlg();

	bool setCloud(std::vector<ccHObject*> cloud);
protected:
	void keyPressEvent(QKeyEvent* event) override;

private slots:
	void onAutoExtract();        // 全自动提取
	void onBoxSelectExtract();   // 框选提取
	void onPointGrowExtract();   // 点生长提取
	void onBoxClip();            // 框选截取
	void onFilteCloudByIntensity();         // 过滤点云
	void onFilteCloudByZ();

	void onItemPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3&, const CCVector3d&);
	void onItemPickedFast(ccHObject* entity, int subEntityID, int x, int y);
	void onLeftButtonClicked(int x, int y);
	void onLeftButtonDoubleClicked(int x, int y);
	void onRightButtonDoubleClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onButtonReleased();
	void onEntitySelectionChanged(ccHObject* entity);
	void onMouseWheelRotated(int delta);

	void addCloudToDB(ccPointCloud* cloud);

private:
	void showThresholdHistogram(ccPointCloud* pointCloud, bool isfilterIntensity, bool is_has_threshold = true, float lowerThreshold = 50, float upperThreshold = 250);

	bool m_selecting = false;          // 是否正在选择
	ccHObject* p_select_cloud = nullptr; // 选择的点云对象
	ThresholdHistogramWidget* histogramWidget;
	QWidget* m_glWidget = nullptr;      // OpenGL Widget
	ccMainAppInterface* m_app = nullptr; // 主程序接口
	ccGLWindowInterface* m_glWindow = nullptr;  // OpenGL 窗口接口

	CloudObjectTreeWidget* m_objectTree = nullptr; // 对象树
	SelectionMode m_selectionMode = ENTITY_SELECTION; // 选择模式

	ForegroundPolylineEditor* m_foregroundPolylineEditor; // 前景折线编辑器
};

/// <summary>
/// ForegroundPolylineEditor 类用于编辑前景折线
/// </summary>
class ForegroundPolylineEditor : public QObject
{
	Q_OBJECT

public:
	ForegroundPolylineEditor(ccGLWindowInterface* glWindow);
	~ForegroundPolylineEditor();

	void setSelectCloudPtr(ccHObject** select_cloud);
	void setCallbackfunc(std::function<void()> callback);
	void closeLine();
	void startDraw();

	void onLeftButtonClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onDoubleLeftButtonClicked(int x, int y);
	void onDoubleRightButtonClicked(int x, int y);
	void onMouseWheelRotated(int delta);

	void onKeyPressEvent(QKeyEvent* event);

	void getPoints(std::vector<CCVector3d>& polyline);

	void setDraw(int max_points_num, bool isClosed);

	void resetDraw();
private:
	void updatePoly();
	void finishDraw(bool doAction);

	ccGLWindowInterface* m_glWindow;      // 用于显示的OpenGL窗口
	ccPointCloud* m_pointCloud;           // 用于绑定折线的点云对象(2D屏幕的坐标)
	ccPolyline* m_foregroundPolyline;     // 前景折线对象
	std::vector<CCVector3d> m_3DPoints;   // 用于保存点击位置

	// 备份标记
	ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
	ccGLWindowInterface::PICKING_MODE picking_mode_backup;

	ccHObject** pp_select_cloud = nullptr;  // 选择的点云对象指针
	std::function<void()> m_callback;       // 回调函数

	bool isFreezeUI = false;  // 是否冻结UI

	int max_points_num = INF;
	bool isClosed = true;

signals:
	void draw_start();  // 开始绘制
	void draw_finish(); // 绘制完成
	void update_tree(); // 更新树视图
};

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
	void contextMenuEvent(QContextMenuEvent* event) override;

	void getAllPointCloudsRecursive(ccHObject* object, std::vector<ccPointCloud*>& pointClouds); // 递归获取所有点云

	void loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem); // 加载树节点

private:
	ccHObject* root = nullptr;  // 根节点
	ccHObject** pp_select_cloud = nullptr;  // 选择的点云对象指针
	ccMainAppInterface* m_app = nullptr;  // 主程序接口
	ccGLWindowInterface* m_glWindow = nullptr;  // OpenGL 窗口接口

	ccGenericGLDisplay* originalDisplay;  // 原始显示对象


};
