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
		delete m_pointCloud;  // ɾ���ڲ������ĵ���
		m_pointCloud = nullptr;
	}
	// �ͷ���Դ
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
	// ���ڵ�׼��
	{
		// ���ݽ�����ʽ������ʱ�ָ�������������ť
		interaction_flags_backup = m_glWindow->getInteractionMode();
		picking_mode_backup = m_glWindow->getPickingMode();
		emit draw_start();

		// �̶�Ϊ�������µ�������ͼ����ֹ�����ת
		m_glWindow->setView(CC_TOP_VIEW);
		//if (pp_select_cloud && *pp_select_cloud)
		//{
		//	ccBBox bbox = (*pp_select_cloud)->getOwnBB();
		//	m_glWindow->updateConstellationCenterAndZoom(&bbox);
		//}
		m_glWindow->setInteractionMode(ccGLWindowInterface::MODE_PAN_ONLY | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
		m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);
	}

	// ���ݵ�׼��
	{
		// ����������ߣ���ɾ����
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
	// ���ûص�����
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
		m_foregroundPolyline->addPointIndex(static_cast<unsigned>(m_pointCloud->size() - 1)); // ��ʱ
		m_foregroundPolyline->addPointIndex(0); // �պ�
	}

	// ��һ����̶�
	{
		CCVector3* lastPoint = const_cast<CCVector3*>(m_pointCloud->getPointPersistentPtr(m_foregroundPolyline->size() - 2));
		*lastPoint = newPoint;
		ccGLCameraParameters camera;
		m_glWindow->getGLCameraParameters(camera);

		// ������ͬ PointCloudSelector::updatePoly()
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

	// �϶�����2D��Ļ�ϵĵ�
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

	// ��������(viewport[0] + viewport[2]/2, viewport[1] + viewport[3]/2)��gl��Ļ�������Ͻǣ� ��Ҫ�Ƶ�����
	// ��֪���ڲ��Ĺ���Ϊʲô����������Ľ��
	// �ֶ�����
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
