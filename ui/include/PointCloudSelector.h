#pragma once

// Qt 相关
#include <QObject>
#include <QKeyEvent>
#include <QMouseEvent>

// 标准库
#include <functional>
#include <vector>

// CloudCompare Core 相关
#include <ccGLWindowInterface.h>     // 渲染窗口接口
#include <ccPointCloud.h>            // 点云对象
#include <ccPolyline.h>              // 折线对象
#include <ccHObject.h>               // HObject 基类

#include <limits>
inline constexpr int INF = std::numeric_limits<int>::max();

/// <summary>
/// PointCloudSelector 类用于编辑前景折线
/// </summary>
class PointCloudSelector : public QObject
{
	Q_OBJECT

public:
	PointCloudSelector(ccGLWindowInterface* glWindow);
	~PointCloudSelector();

	void setSelectCloudPtr(ccHObject** select_cloud);
	void setCallbackfunc(std::function<void()> callback);
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
