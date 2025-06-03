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
	// 绘制模式（普通折线、闭合折线、矩形）
	// ========================
	enum class DrawMode {
		PolylineOpen,
		PolylineClosed,
		Rectangle
	};

	// ========================
	// 矩形绘制状态
	// ========================
	enum class RectangleState {
		WaitingFirstCorner,/*第一个次点击加入两个相同的点,一个固定,另一个用来跟踪移动*/ 
		WaitingSecondCorner, /*固定移动的点,并再次添加点一个点来跟踪移动*/
		TrackingThirdCorner /*计算平行线来绘制矩形*/
	};

	// ========================
	// 折线绘制状态
	// ========================
	enum class PolyState {
		WaitingFirstPoint, /*第一个次点击加入两个相同的点,一个固定,另一个用来跟踪移动*/ 
		WaitingNewPoint /*固定移动的点,并再次添加点来跟踪移动*/
	};

	// ========================
	// 绘制状态：记录当前折线/矩形的坐标
	// ========================
	struct DrawingState
	{
		std::vector<CCVector3d> ctrolPoints;/*保存的真实的空间位置，在选择、缩放的时候恢复绘制内容*/
		ccPointCloud* pointCloud = nullptr;/*折线依赖的点云*/
		ccPolyline* polyline2D = nullptr;/*绘制在前景的折线*/

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
	// 主类：点云框选器
	// ========================
	class PointCloudSelector : public QObject
	{
		Q_OBJECT
	public:
		explicit PointCloudSelector(ccGLWindowInterface* glWindow);
		~PointCloudSelector();

		// 外部接口
		void setSelectCloudPtr(ccHObject** select_cloud);
		void setCallbackfunc(std::function<void()> callback);
		void setDraw(DrawMode mode, int max_points_num = INF);
		void startDraw();
		void resetDraw();
		void getPoints(std::vector<CCVector3d>& polyline);

		// 事件处理（转发）
		void onLeftButtonClicked(int x, int y);
		void onMouseMoved(int x, int y, Qt::MouseButtons button);
		void onDoubleLeftButtonClicked(int x, int y);
		void onDoubleRightButtonClicked(int x, int y);
		void onMouseWheelRotated(int delta);
		void onKeyPressEvent(QKeyEvent* event);

	signals:
		void draw_start();   // 开始绘制
		void draw_finish();  // 绘制完成
		void update_tree();  // 更新树视图

	private:
		// 模式分发
		void handlePolylineClick(int x, int y);
		void handleRectangleClick(int x, int y);
		void updateViewPoints();
		void finishDraw(bool doAction);

		// 基础变量
		ccGLWindowInterface* m_glWindow;
		ccHObject** pp_select_cloud = nullptr;
		std::function<void()> m_callback;

		bool isFreezeUI = false;/*不允许点击、移动、等待其他操作完成*/
		int max_points_num = INF;
		DrawMode m_drawMode = DrawMode::PolylineOpen;

		// 状态保存
		DrawingState m_state;
		RectangleState m_rectState = RectangleState::WaitingFirstCorner;
		PolyState m_polyState = PolyState::WaitingFirstPoint;

		// 相机交互恢复备份
		ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
		ccGLWindowInterface::PICKING_MODE picking_mode_backup;
	};

} // namespace cc_interact
