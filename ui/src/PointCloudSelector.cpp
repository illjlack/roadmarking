#include "PointCloudSelector.h"
#include "ccGLWindowInterface.h"


PointCloudSelector::PointCloudSelector(ccGLWindowInterface* glWindow)
	: m_glWindow(glWindow), m_pointCloud(new ccPointCloud()), m_foregroundPolyline(new ccPolyline(m_pointCloud))
{
	m_foregroundPolyline->addChild(m_pointCloud);
	m_foregroundPolyline->setColor(ccColor::green);
	m_foregroundPolyline->showColors(true);
	m_foregroundPolyline->set2DMode(true);
	m_foregroundPolyline->setForeground(true);
	m_foregroundPolyline->showVertices(true);
	m_foregroundPolyline->setVertexMarkerWidth(2);
}

PointCloudSelector::~PointCloudSelector()
{
	m_glWindow->removeFromOwnDB(m_foregroundPolyline);

	if (m_pointCloud)
	{
		delete m_pointCloud;  // 删除内部创建的点云
		m_pointCloud = nullptr;
	}
	// 释放资源
	if (m_foregroundPolyline)
	{
		delete m_foregroundPolyline;
		m_foregroundPolyline = nullptr;
	}

}

void PointCloudSelector::setSelectCloudPtr(ccHObject** select_cloud)
{
	pp_select_cloud = select_cloud;
}

void PointCloudSelector::startDraw()
{
	// 窗口的准备
	{
		// 备份交互方式，结束时恢复，禁用其他按钮
		interaction_flags_backup = m_glWindow->getInteractionMode();
		picking_mode_backup = m_glWindow->getPickingMode();
		emit draw_start();

		// 固定为从上往下的正交视图，禁止相机旋转
		m_glWindow->setView(CC_TOP_VIEW);
		//if (pp_select_cloud && *pp_select_cloud)
		//{
		//	ccBBox bbox = (*pp_select_cloud)->getOwnBB();
		//	m_glWindow->updateConstellationCenterAndZoom(&bbox);
		//}
		m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_PAN_ONLY | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
		m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);
	}

	// 数据的准备
	{
		// 如果已有折线，则删除它
		if (m_foregroundPolyline->size())
		{
			m_foregroundPolyline->clear();
			m_pointCloud->clear();
			m_3DPoints.clear();
		}
		m_glWindow->addToOwnDB(m_foregroundPolyline);
	}
}

void PointCloudSelector::finishDraw(bool doAction)
{
	isFreezeUI = true;

	if (doAction && m_callback)
	{
		if (isClosed)m_3DPoints.push_back(m_3DPoints[0]);
		m_callback();
	}
	m_glWindow->setInteractionMode(interaction_flags_backup);
	m_glWindow->setPickingMode(picking_mode_backup);
	emit draw_finish();
	m_glWindow->removeFromOwnDB(m_foregroundPolyline);
	emit update_tree();
	if (m_foregroundPolyline->size())
	{
		m_foregroundPolyline->clear();
		m_pointCloud->clear();
		m_3DPoints.clear();
	}
	m_glWindow->redraw(true, false);
	isFreezeUI = false;
	resetDraw();
}

void PointCloudSelector::setCallbackfunc(std::function<void()> callback)
{
	// 设置回调函数
	m_callback = callback;
}

void PointCloudSelector::onLeftButtonClicked(int x, int y)
{
	if (isFreezeUI)return;
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	if (!m_pointCloud->size())
	{
		m_pointCloud->addPoint(newPoint);
		m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1)); // 临时
		m_foregroundPolyline->addPointIndex(0); // 闭合
	}

	// 上一个点固定
	{
		CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 2));
		*lastPoint = newPoint;
		ccGLCameraParameters camera;
		m_glWindow->getGLCameraParameters(camera);

		// 修正，同 PointCloudSelector::updatePoly()
		camera.viewport[0] = -camera.viewport[2] / 2;
		camera.viewport[1] = -camera.viewport[3] / 2;

		CCVector3d _3DPoint;
		camera.unproject(*lastPoint, _3DPoint);
		m_3DPoints.push_back(_3DPoint);

		if (m_3DPoints.size() == max_points_num)
		{
			finishDraw(true);
			return;
		}
	}

	m_pointCloud->addPoint(newPoint);
	m_foregroundPolyline->removePointGlobalIndex(m_foregroundPolyline->size() - 1);
	m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1));
	if (isClosed)m_foregroundPolyline->addPointIndex(0);
	else m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1));
	m_glWindow->redraw(true, false);
}

void PointCloudSelector::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (isFreezeUI)return;

	if (!m_pointCloud->size())
		return;

	// 拖动更新2D屏幕上的点
	if (button == Qt::RightButton)
	{
		updatePoly();
	}

	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);

	CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 2));
	*lastPoint = newPoint;
}

void PointCloudSelector::onDoubleLeftButtonClicked(int x, int y)
{
	if (isFreezeUI) return;
	finishDraw(true);
}

void PointCloudSelector::onDoubleRightButtonClicked(int x, int y)
{
	if (isFreezeUI) return;
	finishDraw(false);
}

void PointCloudSelector::onMouseWheelRotated(int delta)
{
	if (isFreezeUI)return;

	updatePoly();
}

void PointCloudSelector::onKeyPressEvent(QKeyEvent* event)
{
	if (isFreezeUI)return;

	if (event->key() == Qt::Key_F)
	{
		finishDraw(true);
	}

	if (event->key() == Qt::Key_G)
	{
		finishDraw(false);
	}
}

void PointCloudSelector::updatePoly()
{
	ccGLCameraParameters camera;

	// 缩放中心(viewport[0] + viewport[2]/2, viewport[1] + viewport[3]/2)在gl屏幕上是右上角， 需要移到中心
	// 不知道内部的过程为什么会产生这样的结果
	// 手动修正
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	for (size_t i = 0; i < m_3DPoints.size(); ++i)
	{
		CCVector3d projectedPoint;
		camera.project(m_3DPoints[i], projectedPoint);
		CCVector3* point = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(i));

		*point = CCVector3(projectedPoint.x
			, projectedPoint.y
			, 0);
	}
	m_glWindow->redraw(true, false);
}


void PointCloudSelector::getPoints(std::vector<CCVector3d>& polyline)
{
	polyline = m_3DPoints;
}

void PointCloudSelector::setDraw(int max_points_num, bool isClosed)
{
	this->max_points_num = max_points_num;
	this->isClosed = isClosed;
}

void PointCloudSelector::resetDraw()
{
	max_points_num = INF;
	isClosed = true;
}
