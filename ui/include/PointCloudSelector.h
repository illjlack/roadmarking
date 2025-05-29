#pragma once

// Qt ���
#include <QObject>
#include <QKeyEvent>
#include <QMouseEvent>

// ��׼��
#include <functional>
#include <vector>

// CloudCompare Core ���
#include <ccGLWindowInterface.h>     // ��Ⱦ���ڽӿ�
#include <ccPointCloud.h>            // ���ƶ���
#include <ccPolyline.h>              // ���߶���
#include <ccHObject.h>               // HObject ����

#include <limits>
inline constexpr int INF = std::numeric_limits<int>::max();

/// <summary>
/// PointCloudSelector �����ڱ༭ǰ������
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

	ccGLWindowInterface* m_glWindow;      // ������ʾ��OpenGL����
	ccPointCloud* m_pointCloud;           // ���ڰ����ߵĵ��ƶ���(2D��Ļ������)
	ccPolyline* m_foregroundPolyline;     // ǰ�����߶���
	std::vector<CCVector3d> m_3DPoints;   // ���ڱ�����λ��

	// ���ݱ��
	ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
	ccGLWindowInterface::PICKING_MODE picking_mode_backup;

	ccHObject** pp_select_cloud = nullptr;  // ѡ��ĵ��ƶ���ָ��
	std::function<void()> m_callback;       // �ص�����

	bool isFreezeUI = false;  // �Ƿ񶳽�UI

	int max_points_num = INF;
	bool isClosed = true;

signals:
	void draw_start();  // ��ʼ����
	void draw_finish(); // �������
	void update_tree(); // ��������ͼ
};
