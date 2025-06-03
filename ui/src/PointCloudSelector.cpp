#include "PointCloudSelector.h"
#include <ccColorTypes.h>
#include <QPointF>

using namespace cc_interact;

PointCloudSelector::PointCloudSelector(ccGLWindowInterface* glWindow)
	: m_glWindow(glWindow)
{
	m_state.pointCloud = new ccPointCloud();
	m_state.polyline2D = new ccPolyline(m_state.pointCloud);
	m_state.polyline2D->addChild(m_state.pointCloud);
	m_state.polyline2D->setColor(ccColor::green);
	m_state.polyline2D->showColors(true);
	m_state.polyline2D->set2DMode(true);
	m_state.polyline2D->setForeground(true);
	m_state.polyline2D->showVertices(true);
	m_state.polyline2D->setVertexMarkerWidth(2);
}

PointCloudSelector::~PointCloudSelector()
{
	if (m_glWindow)
		m_glWindow->removeFromOwnDB(m_state.polyline2D);

	delete m_state.pointCloud;
	delete m_state.polyline2D;
}

void PointCloudSelector::setSelectCloudPtr(ccHObject** select_cloud)
{
	pp_select_cloud = select_cloud;
}

void PointCloudSelector::setCallbackfunc(std::function<void()> callback)
{
	m_callback = std::move(callback);
}

void PointCloudSelector::setDraw(DrawMode mode, int max_points)
{
	max_points_num = max_points;
	m_drawMode = mode;
}

void PointCloudSelector::startDraw()
{
	interaction_flags_backup = m_glWindow->getInteractionMode();
	picking_mode_backup = m_glWindow->getPickingMode();
	emit draw_start();

	m_glWindow->setView(CC_TOP_VIEW);
	m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_PAN_ONLY | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
	m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);

	m_state.clear();
	m_glWindow->addToOwnDB(m_state.polyline2D);
	isFreezeUI = false;
}

void PointCloudSelector::resetDraw()
{
	max_points_num = INF;
	m_drawMode = DrawMode::PolylineOpen;
	m_state.clear();
	m_rectState = RectangleState::WaitingFirstCorner;
	m_polyState = PolyState::WaitingFirstPoint;
}

void PointCloudSelector::getPoints(std::vector<CCVector3d>& polyline)
{
	if (m_drawMode == DrawMode::PolylineOpen)
	{
		polyline = m_state.ctrolPoints;
	}
	else if (m_drawMode == DrawMode::PolylineClosed)
	{
		polyline = m_state.ctrolPoints;
		if (polyline.size())polyline.push_back(polyline.front());
	}
	else if (m_drawMode == DrawMode::Rectangle)
	{
		assert(m_state.ctrolPoints.size() == 3);
		auto& points = m_state.ctrolPoints;

		const CCVector3d& p1 = points[0];
		const CCVector3d& p2 = points[1];

		CCVector3d dir = (p2 - p1);
		dir.normalize();
		CCVector3d ortho(-dir.y, dir.x, 0);

		double length = (points[2] - p2).dot(ortho);
		CCVector3d p3 = p1 + ortho * length;
		CCVector3d p4 = p2 + ortho * length;

		polyline.push_back(p1);
		polyline.push_back(p2);
		polyline.push_back(p4);
		polyline.push_back(p3);
	}

}

void PointCloudSelector::onLeftButtonClicked(int x, int y)
{
	if (isFreezeUI) return;

	switch (m_drawMode)
	{
	case DrawMode::PolylineOpen:
	case DrawMode::PolylineClosed:
		handlePolylineClick(x, y); break;
	case DrawMode::Rectangle:
		handleRectangleClick(x, y); break;
	}
}

void PointCloudSelector::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (isFreezeUI) return;

	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 p3D(pos2D.x(), pos2D.y(), 0);

	if (m_drawMode == DrawMode::Rectangle)
	{
		m_state.updateTracking(m_rectState, p3D);
	}
	else if ((m_drawMode == DrawMode::PolylineOpen || m_drawMode == DrawMode::PolylineClosed))
	{
		m_state.updateTracking(m_polyState, p3D);
	}


	// 拖动更新2D屏幕上的点
	if (button == Qt::RightButton)
	{
		updateViewPoints();
	}
		
}

void PointCloudSelector::onDoubleLeftButtonClicked(int, int)
{
	if (!isFreezeUI && m_drawMode == DrawMode::PolylineClosed)finishDraw(true);
	else if (!isFreezeUI) finishDraw(false); /*目前其他的会自动终止*/
}

void PointCloudSelector::onDoubleRightButtonClicked(int, int)
{
	if (!isFreezeUI) finishDraw(false);
}

void PointCloudSelector::onMouseWheelRotated(int)
{
	if (!isFreezeUI)
		updateViewPoints();
}

void PointCloudSelector::onKeyPressEvent(QKeyEvent* event)
{
	if (isFreezeUI) return;
	if (event->key() == Qt::Key_F) finishDraw(true);
	if (event->key() == Qt::Key_G) finishDraw(false);
}

void PointCloudSelector::handlePolylineClick(int x, int y)
{
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	CCVector3d _3DPoint;
	camera.unproject(newPoint, _3DPoint);

	switch (m_polyState)
	{
	case PolyState::WaitingFirstPoint:
		m_state.pointCloud->addPoint(newPoint);
		m_state.pointCloud->addPoint(newPoint);
		m_state.polyline2D->addPointIndex(0);
		m_state.polyline2D->addPointIndex(1);
		m_state.ctrolPoints.push_back(_3DPoint);

		// 若是闭合模式，重新添加闭合的起点索引（0）
		if (m_drawMode == DrawMode::PolylineClosed)
		{
			m_state.polyline2D->addPointIndex(0);
		}
		m_polyState = PolyState::WaitingNewPoint;
		break;
	case PolyState::WaitingNewPoint:
		// 若是闭合模式，则先移除闭合的点索引（0）
		if (m_drawMode == DrawMode::PolylineClosed && m_state.polyline2D->size() >= 2)
		{
			// 移除尾部的闭合索引（0）
			m_state.polyline2D->resize(m_state.polyline2D->size() - 1);
		}

		m_state.pointCloud->addPoint(newPoint);
		m_state.polyline2D->addPointIndex(m_state.pointCloud->size() - 1);
		m_state.ctrolPoints.push_back(_3DPoint);

		// 若是闭合模式，重新添加闭合的起点索引（0）
		if (m_drawMode == DrawMode::PolylineClosed)
		{
			m_state.polyline2D->addPointIndex(0);
		}
		break;
	}
	if (m_state.ctrolPoints.size() == max_points_num)
		finishDraw(true);
	m_glWindow->redraw(true, false);
}

void PointCloudSelector::handleRectangleClick(int x, int y)
{
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	CCVector3d _3DPoint;
	camera.unproject(newPoint, _3DPoint);
	switch (m_rectState)
	{
	case RectangleState::WaitingFirstCorner:
		m_state.pointCloud->addPoint(newPoint);
		m_state.pointCloud->addPoint(newPoint);
		m_state.polyline2D->addPointIndex(0);
		m_state.polyline2D->addPointIndex(1);
		m_state.ctrolPoints.push_back(_3DPoint);
		m_rectState = RectangleState::WaitingSecondCorner;
		break;
	case RectangleState::WaitingSecondCorner:
		m_state.pointCloud->addPoint(newPoint);
		m_state.polyline2D->addPointIndex(m_state.pointCloud->size() - 1);
		m_state.pointCloud->addPoint(newPoint);
		m_state.polyline2D->addPointIndex(m_state.pointCloud->size() - 1);
		m_state.ctrolPoints.push_back(_3DPoint);
		m_state.polyline2D->addPointIndex(0);
		m_rectState = RectangleState::TrackingThirdCorner;
		break;
	case RectangleState::TrackingThirdCorner:
		m_state.ctrolPoints.push_back(_3DPoint);
		finishDraw(true);
		break;
	}
}

void PointCloudSelector::updateViewPoints()
{
	ccGLCameraParameters camera;
	// 缩放中心(viewport[0] + viewport[2]/2, viewport[1] + viewport[3]/2)在gl屏幕上是右上角， 需要移到中心
	// 不知道内部的过程为什么会产生这样的结果
	// 手动修正
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	if (m_drawMode == DrawMode::PolylineOpen || m_drawMode == DrawMode::PolylineClosed)
	{
		m_state.updateView(m_polyState, camera);
	}
	else if (m_drawMode == DrawMode::Rectangle)
	{
		m_state.updateView(m_rectState, camera);
	}
	m_glWindow->redraw(true, false);
}

void PointCloudSelector::finishDraw(bool doAction)
{
	isFreezeUI = true;
	if (doAction && m_callback)
		m_callback();
	m_glWindow->setInteractionMode(interaction_flags_backup);
	m_glWindow->setPickingMode(picking_mode_backup);
	emit draw_finish();
	m_glWindow->removeFromOwnDB(m_state.polyline2D);
	emit update_tree();
	m_glWindow->redraw(true, false);
	resetDraw();
}
