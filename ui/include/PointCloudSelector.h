#pragma once

// Qt
#include <QObject>
#include <QKeyEvent>
#include <QMouseEvent>

// CloudCompare
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccHObject.h>

// STL
#include <functional>
#include <vector>
#include <limits>

namespace cc_interact
{
	inline constexpr int INF = std::numeric_limits<int>::max();

	// ========================
	// ����ģʽ����ͨ���ߡ��պ����ߡ����Σ�
	// ========================
	enum class DrawMode {
		PolylineOpen,
		PolylineClosed,
		Rectangle
	};

	// ========================
	// ���λ���״̬
	// ========================
	enum class RectangleState {
		WaitingFirstCorner,/*��һ���ε������������ͬ�ĵ�,һ���̶�,��һ�����������ƶ�*/ 
		WaitingSecondCorner, /*�̶��ƶ��ĵ�,���ٴ���ӵ�һ�����������ƶ�*/
		TrackingThirdCorner /*����ƽ���������ƾ���*/
	};

	// ========================
	// ���߻���״̬
	// ========================
	enum class PolyState {
		WaitingFirstPoint, /*��һ���ε������������ͬ�ĵ�,һ���̶�,��һ�����������ƶ�*/ 
		WaitingNewPoint /*�̶��ƶ��ĵ�,���ٴ���ӵ��������ƶ�*/
	};

	// ========================
	// ����״̬����¼��ǰ����/���ε�����
	// ========================
	struct DrawingState
	{
		std::vector<CCVector3d> ctrolPoints;/*�������ʵ�Ŀռ�λ�ã���ѡ�����ŵ�ʱ��ָ���������*/
		ccPointCloud* pointCloud = nullptr;/*���������ĵ���*/
		ccPolyline* polyline2D = nullptr;/*������ǰ��������*/

		inline void updateView(PolyState m_polyState, ccGLCameraParameters camera)
		{
			for (size_t i = 0; i < ctrolPoints.size(); ++i)
			{
				CCVector3d projectedPoint;
				camera.project(ctrolPoints[i], projectedPoint);
				CCVector3 pt2D(static_cast<PointCoordinateType>(projectedPoint.x),
					static_cast<PointCoordinateType>(projectedPoint.y),
					0);
				*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(i)) = pt2D;
			}
		}

		inline void updateTracking(PolyState m_polyState, CCVector3& newPoint)
		{
			if (m_polyState == PolyState::WaitingNewPoint)
			{
				*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(pointCloud->size() - 1)) = newPoint;
			}
		}
		inline void updateTracking(RectangleState m_rectangleState, CCVector3& newPoint)
		{
			if (m_rectangleState == RectangleState::TrackingThirdCorner)
			{
				const CCVector3& p1 = *pointCloud->getPoint(0);
				const CCVector3& p2 = *pointCloud->getPoint(1);

				CCVector3 dir = (p2 - p1);
				dir.normalize();
				CCVector3 ortho(-dir.y, dir.x, 0);

				double length = (newPoint - p2).dot(ortho);
				CCVector3 p3 = p1 + ortho * length;
				CCVector3 p4 = p2 + ortho * length;

				*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(3)) = p3;
				*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(2)) = p4;
			}
			else if (m_rectangleState == RectangleState::WaitingSecondCorner)
			{
				*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(1)) = newPoint;
			}
		}

		inline void updateView(RectangleState m_rectState, ccGLCameraParameters camera)
		{
			if (m_rectState == RectangleState::TrackingThirdCorner)
			{
				assert(pointCloud->size() == 4);
				CCVector3d projectedPoint;
				{
					camera.project(ctrolPoints[0], projectedPoint);
					CCVector3 pt2D(static_cast<PointCoordinateType>(projectedPoint.x),
						static_cast<PointCoordinateType>(projectedPoint.y),
						0);
					*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(0)) = pt2D;
				}
				{
					camera.project(ctrolPoints[1], projectedPoint);
					CCVector3 pt2D(static_cast<PointCoordinateType>(projectedPoint.x),
						static_cast<PointCoordinateType>(projectedPoint.y),
						0);
					*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(1)) = pt2D;
				}

				{
					const CCVector3& p1 = *pointCloud->getPoint(0);
					const CCVector3& p2 = *pointCloud->getPoint(1);

					CCVector3 dir = (p2 - p1);
					dir.normalize();
					CCVector3 ortho(-dir.y, dir.x, 0);

					double length = (*pointCloud->getPoint(2) - p2).dot(ortho);
					CCVector3 p3 = p1 + ortho * length;
					CCVector3 p4 = p2 + ortho * length;

					*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(3)) = p3;
					*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(2)) = p4;
				}
			}
			else
			{
				for (size_t i = 0; i < ctrolPoints.size(); ++i)
				{
					CCVector3d projectedPoint;
					camera.project(ctrolPoints[i], projectedPoint);
					CCVector3 pt2D(static_cast<PointCoordinateType>(projectedPoint.x),
						static_cast<PointCoordinateType>(projectedPoint.y),
						0);
					*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(i)) = pt2D;
				}
			}
		}

		inline void clear()
		{
			ctrolPoints.clear();
			if (pointCloud)
				pointCloud->clear();
			if (polyline2D)
				polyline2D->clear();
		}
	};

	// ========================
	// ���ࣺ���ƿ�ѡ��
	// ========================
	class PointCloudSelector : public QObject
	{
		Q_OBJECT
	public:
		explicit PointCloudSelector(ccGLWindowInterface* glWindow);
		~PointCloudSelector();

		// �ⲿ�ӿ�
		void setSelectCloudPtr(ccHObject** select_cloud);
		void setCallbackfunc(std::function<void()> callback);
		void setDraw(DrawMode mode, int max_points_num = INF);
		void startDraw();
		void resetDraw();
		void getPoints(std::vector<CCVector3d>& polyline);

		// �¼�����ת����
		void onLeftButtonClicked(int x, int y);
		void onMouseMoved(int x, int y, Qt::MouseButtons button);
		void onDoubleLeftButtonClicked(int x, int y);
		void onDoubleRightButtonClicked(int x, int y);
		void onMouseWheelRotated(int delta);
		void onKeyPressEvent(QKeyEvent* event);

	signals:
		void draw_start();   // ��ʼ����
		void draw_finish();  // �������
		void update_tree();  // ��������ͼ

	private:
		// ģʽ�ַ�
		void handlePolylineClick(int x, int y);
		void handleRectangleClick(int x, int y);
		void updateViewPoints();
		void finishDraw(bool doAction);

		// ��������
		ccGLWindowInterface* m_glWindow;
		ccHObject** pp_select_cloud = nullptr;
		std::function<void()> m_callback;

		bool isFreezeUI = false;/*�����������ƶ����ȴ������������*/
		int max_points_num = INF;
		DrawMode m_drawMode = DrawMode::PolylineOpen;

		// ״̬����
		DrawingState m_state;
		RectangleState m_rectState = RectangleState::WaitingFirstCorner;
		PolyState m_polyState = PolyState::WaitingFirstPoint;

		// ��������ָ�����
		ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
		ccGLWindowInterface::PICKING_MODE picking_mode_backup;
	};

} // namespace cc_interact
