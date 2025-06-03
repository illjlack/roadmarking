#pragma once

#include <QObject>
#include <QKeyEvent>
#include <QMouseEvent>
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccHObject.h>
#include <functional>
#include <vector>
#include <limits>

namespace cc_drawing
{
	inline constexpr int INF = std::numeric_limits<int>::max();

	enum class PolyState
	{
		WaitingFirstPoint,
		WaitingNewPoint
	};

	struct DrawingState
	{
		std::vector<CCVector3d> ctrolPoints;
		ccPointCloud* pointCloud = nullptr;
		std::vector<ccPolyline*> polyline2D;

		inline void updateView(const ccGLCameraParameters& camera)
		{
			if (!pointCloud) return;
			for (size_t i = 0; i < ctrolPoints.size(); ++i)
			{
				CCVector3d projected;
				camera.project(ctrolPoints[i], projected);
				CCVector3 pt2D(
					static_cast<PointCoordinateType>(projected.x),
					static_cast<PointCoordinateType>(projected.y),
					0
				);
				*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(i)) = pt2D;
			}
		}

		inline void updateTracking(PolyState m_polyState, const CCVector3& newPoint)
		{
			if (m_polyState == PolyState::WaitingNewPoint && pointCloud && pointCloud->size() > 0)
			{
				*const_cast<CCVector3*>(pointCloud->getPointPersistentPtr(pointCloud->size() - 1)) = newPoint;
			}
		}

		inline void clear()
		{
			ctrolPoints.clear();
			if (pointCloud) pointCloud->clear();
			for (auto p : polyline2D)
			{
				delete p;
			}
			polyline2D.clear();
		}

		inline ~DrawingState()
		{
			for (auto p : polyline2D)
			{
				delete p;
			}
			delete pointCloud;
		}
	};

	class PointCloudDrawer : public QObject
	{
		Q_OBJECT

	public:
		explicit PointCloudDrawer(ccGLWindowInterface* glWindow);
		~PointCloudDrawer();

		void setSelectCloudPtr(ccHObject** select_cloud);
		void setCallbackfunc(std::function<void()> callback);
		void startDraw();
		void resetDraw();
		void getPoints(std::vector<std::vector<CCVector3d>>& polylines) const;

		void onLeftButtonClicked(int x, int y);
		void onMouseMoved(int x, int y, Qt::MouseButtons button);
		void onDoubleLeftButtonClicked(int x, int y);
		void onDoubleRightButtonClicked(int x, int y);
		void onMouseWheelRotated(int delta);
		void onKeyPressEvent(QKeyEvent* event);

	signals:
		void draw_start();
		void draw_finish();
		void update_tree();

	private:
		void handlePolylineClick(int x, int y);
		void updateViewPoints();
		void finishSinglePolyline(bool isUsed);
		void finishDraw(bool doAction);

		ccGLWindowInterface* m_glWindow;
		ccHObject** pp_select_cloud = nullptr;
		std::function<void()> m_callback;
		bool isFreezeUI = false;
		DrawingState m_state;
		bool is_readyExit = false;
		int m_currentPolyIndex = -1;
		PolyState m_polyState = PolyState::WaitingFirstPoint;
		ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
		ccGLWindowInterface::PICKING_MODE picking_mode_backup;
	};
} // namespace cc_drawing
