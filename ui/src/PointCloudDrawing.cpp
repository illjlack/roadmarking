#include "PointCloudDrawing.h"
#include <ccColorTypes.h>
#include <QPointF>

using namespace cc_drawing;

PointCloudDrawer::PointCloudDrawer(ccGLWindowInterface* glWindow)
	: m_glWindow(glWindow)
{
}

PointCloudDrawer::~PointCloudDrawer()
{
}

void PointCloudDrawer::setSelectCloudPtr(ccHObject** select_cloud)
{
	pp_select_cloud = select_cloud;
}

void PointCloudDrawer::setCallbackfunc(std::function<void()> callback)
{
	m_callback = std::move(callback);
}

void PointCloudDrawer::startDraw()
{
	// 备份当前相机交互和拾取模式
	interaction_flags_backup = m_glWindow->getInteractionMode();
	picking_mode_backup = m_glWindow->getPickingMode();

	// 发送绘制开始信号
	emit draw_start();

	// 切换到正俯视，并将交互模式限定为平移，禁止其他拾取
	m_glWindow->setView(CC_TOP_VIEW);
	m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_PAN_ONLY | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
	m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);

	m_state.clear();
	m_currentPolyIndex = -1;
	isFreezeUI = false;

	is_readyExit = false;


	m_state.pointCloud = new ccPointCloud();
	m_state.polyline2D.push_back(new ccPolyline(m_state.pointCloud));
	m_state.polyline2D.back()->setColor(ccColor::green);
	m_state.polyline2D.back()->showColors(true);
	m_state.polyline2D.back()->set2DMode(true);
	m_state.polyline2D.back()->setForeground(true);
	m_state.polyline2D.back()->showVertices(true);
	m_state.polyline2D.back()->setVertexMarkerWidth(2);

	m_currentPolyIndex = 0;
	m_polyState = PolyState::WaitingFirstPoint;

	// 将首条 polyline2D 添加到场景
	m_glWindow->addToOwnDB(m_state.polyline2D.back());
}

void PointCloudDrawer::resetDraw()
{
	// 恢复最大点数和冻结标志、状态
	isFreezeUI = false;
	m_polyState = PolyState::WaitingFirstPoint;
	is_readyExit = false;
	m_state.clear();
	m_currentPolyIndex = -1;
}

void PointCloudDrawer::getPoints(std::vector<std::vector<CCVector3d>>& polylines) const
{
	polylines.clear();
	for (auto polyline: m_state.polyline2D)
	{
		if (polyline == m_state.polyline2D.back())break;

		polylines.push_back({});
		unsigned count = polyline->size();

		for (unsigned i = 0; i < count; ++i)
		{
			// 获取第 i 个顶点在 3D 控制点数组中的索引
			unsigned pointIndex = polyline->getPointGlobalIndex(i);

			// 确保索引在 ctrolPoints 范围内
			if (pointIndex < m_state.ctrolPoints.size())
			{
				polylines.back().push_back(m_state.ctrolPoints[pointIndex]);
			}
		}
	}
}

void PointCloudDrawer::onLeftButtonClicked(int x, int y)
{
	if (isFreezeUI)
		return;

	handlePolylineClick(x, y);
}

void PointCloudDrawer::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (isFreezeUI || m_currentPolyIndex < 0)
		return;

	// 将 Qt 屏幕坐标转换为中心化的 GL 坐标
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 mouse2D(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	// 实时更新当前折线的最后一个点（跟踪鼠标位置）
	m_state.updateTracking(m_polyState, mouse2D);

	// 如果右键拖拽，则刷新所有折线在屏幕上的投影
	if (button == Qt::RightButton)
	{
		updateViewPoints();
	}
}

void PointCloudDrawer::onDoubleLeftButtonClicked(int /*x*/, int /*y*/)
{
	if (isFreezeUI || m_currentPolyIndex < 0)
		return;

	// 情况1：若处于“等待新点”状态，则结束当前折线，创建下一条空折线
	if (!is_readyExit)
	{
		finishSinglePolyline(true);
	}
	else
	{
		finishDraw(true);
	}
}

void PointCloudDrawer::onDoubleRightButtonClicked(int /*x*/, int /*y*/)
{
	if (isFreezeUI || m_currentPolyIndex < 0)
		return;

	if (!is_readyExit)
	{
		finishSinglePolyline(false);
	}
	else
	{
		finishDraw(false);
	}
}

void PointCloudDrawer::onMouseWheelRotated(int /*delta*/)
{
	if (isFreezeUI)
		return;

	updateViewPoints();
}

void PointCloudDrawer::onKeyPressEvent(QKeyEvent* event)
{
	if (isFreezeUI || m_currentPolyIndex < 0)
		return;

	if (event->key() == Qt::Key_F)
	{
		if (!is_readyExit)
		{
			finishSinglePolyline(true);
		}
		else
		{
			finishDraw(true);
		}
	}
	// 按 G 键结束当前折线但不执行回调 或 退出绘制
	else if (event->key() == Qt::Key_G)
	{
		if (!is_readyExit)
		{
			finishSinglePolyline(false);
		}
		else
		{
			finishDraw(false);
		}
	}
}

void PointCloudDrawer::handlePolylineClick(int x, int y)
{
	// 将鼠标点击的屏幕坐标转换为中心化 GL 坐标
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint2D(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	// 获取当前相机参数，并将 viewport 原点平移到中心
	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	// 反投影：从 2D 点得到对应的 3D 坐标
	CCVector3d newPoint3D;
	camera.unproject(newPoint2D, newPoint3D);

	switch (m_polyState)
	{
	case PolyState::WaitingFirstPoint:
		m_state.pointCloud->addPoint(newPoint2D);
		m_state.pointCloud->addPoint(newPoint2D);
		m_state.polyline2D.back()->addPointIndex(m_state.pointCloud->size() - 2);
		m_state.polyline2D.back()->addPointIndex(m_state.pointCloud->size() - 1);
		m_state.ctrolPoints.push_back(newPoint3D);
		m_polyState = PolyState::WaitingNewPoint;
		break;

	case PolyState::WaitingNewPoint:
		is_readyExit = false;
		m_state.pointCloud->addPoint(newPoint2D);
		int newIndex = static_cast<int>(m_state.pointCloud->size() - 1);
		m_state.polyline2D.back()->addPointIndex(newIndex);
		m_state.ctrolPoints.push_back(newPoint3D);
		break;
	}

	m_glWindow->redraw(true, false);
}

void PointCloudDrawer::updateViewPoints()
{
	// 获取当前相机参数并将 viewport 原点平移到中心
	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	m_state.updateView(camera);
	m_glWindow->redraw(true, false);
}

void PointCloudDrawer::finishSinglePolyline(bool isUsed)
{
	// 冻结 UI，防止在新折线创建时出现干扰
	isFreezeUI = true;

	if (isUsed)
	{
		m_state.pointCloud->resize(m_state.pointCloud->size() - 1);
		m_state.polyline2D.back()->resize(m_state.polyline2D.back()->size() - 1);

		// 创建下一条空折线，供用户继续绘制
		m_state.polyline2D.push_back(new ccPolyline(m_state.pointCloud));
		m_state.polyline2D.back()->setColor(ccColor::green);
		m_state.polyline2D.back()->showColors(true);
		m_state.polyline2D.back()->set2DMode(true);
		m_state.polyline2D.back()->setForeground(true);
		m_state.polyline2D.back()->showVertices(true);
		m_state.polyline2D.back()->setVertexMarkerWidth(2);

		m_glWindow->addToOwnDB(m_state.polyline2D.back());
	}
	else
	{
		m_state.pointCloud->resize(m_state.pointCloud->size() - 1);
		m_state.polyline2D.back()->clear();
	}

	m_currentPolyIndex = static_cast<int>(m_state.polyline2D.size()) - 1;
	m_polyState = PolyState::WaitingFirstPoint;
	is_readyExit = true;
	isFreezeUI = false;
}

void PointCloudDrawer::finishDraw(bool doAction)
{
	// 冻结 UI，防止在恢复交互过程中响应其他事件
	isFreezeUI = true;

	if (doAction && m_callback)
		m_callback();

	// 恢复窗口原交互模式与拾取模式
	m_glWindow->setInteractionMode(interaction_flags_backup);
	m_glWindow->setPickingMode(picking_mode_backup);

	// 触发“绘制流程彻底结束”信号
	emit draw_finish();

	for (auto poly : m_state.polyline2D)
	{
		m_glWindow->removeFromOwnDB(poly);
	}
	
	emit update_tree();

	resetDraw();
}

