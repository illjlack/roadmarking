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
	// ���ݵ�ǰ���������ʰȡģʽ
	interaction_flags_backup = m_glWindow->getInteractionMode();
	picking_mode_backup = m_glWindow->getPickingMode();

	// ���ͻ��ƿ�ʼ�ź�
	emit draw_start();

	// �л��������ӣ���������ģʽ�޶�Ϊƽ�ƣ���ֹ����ʰȡ
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

	// ������ polyline2D ��ӵ�����
	m_glWindow->addToOwnDB(m_state.polyline2D.back());
}

void PointCloudDrawer::resetDraw()
{
	// �ָ��������Ͷ����־��״̬
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
			// ��ȡ�� i �������� 3D ���Ƶ������е�����
			unsigned pointIndex = polyline->getPointGlobalIndex(i);

			// ȷ�������� ctrolPoints ��Χ��
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

	// �� Qt ��Ļ����ת��Ϊ���Ļ��� GL ����
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 mouse2D(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	// ʵʱ���µ�ǰ���ߵ����һ���㣨�������λ�ã�
	m_state.updateTracking(m_polyState, mouse2D);

	// ����Ҽ���ק����ˢ��������������Ļ�ϵ�ͶӰ
	if (button == Qt::RightButton)
	{
		updateViewPoints();
	}
}

void PointCloudDrawer::onDoubleLeftButtonClicked(int /*x*/, int /*y*/)
{
	if (isFreezeUI || m_currentPolyIndex < 0)
		return;

	// ���1�������ڡ��ȴ��µ㡱״̬���������ǰ���ߣ�������һ��������
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
	// �� G ��������ǰ���ߵ���ִ�лص� �� �˳�����
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
	// �����������Ļ����ת��Ϊ���Ļ� GL ����
	QPointF pos2D = m_glWindow->toCenteredGLCoordinates(x, y);
	CCVector3 newPoint2D(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	// ��ȡ��ǰ������������� viewport ԭ��ƽ�Ƶ�����
	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	// ��ͶӰ���� 2D ��õ���Ӧ�� 3D ����
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
	// ��ȡ��ǰ����������� viewport ԭ��ƽ�Ƶ�����
	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);
	camera.viewport[0] = -camera.viewport[2] / 2;
	camera.viewport[1] = -camera.viewport[3] / 2;

	m_state.updateView(camera);
	m_glWindow->redraw(true, false);
}

void PointCloudDrawer::finishSinglePolyline(bool isUsed)
{
	// ���� UI����ֹ�������ߴ���ʱ���ָ���
	isFreezeUI = true;

	if (isUsed)
	{
		m_state.pointCloud->resize(m_state.pointCloud->size() - 1);
		m_state.polyline2D.back()->resize(m_state.polyline2D.back()->size() - 1);

		// ������һ�������ߣ����û���������
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
	// ���� UI����ֹ�ڻָ�������������Ӧ�����¼�
	isFreezeUI = true;

	if (doAction && m_callback)
		m_callback();

	// �ָ�����ԭ����ģʽ��ʰȡģʽ
	m_glWindow->setInteractionMode(interaction_flags_backup);
	m_glWindow->setPickingMode(picking_mode_backup);

	// �������������̳��׽������ź�
	emit draw_finish();

	for (auto poly : m_state.polyline2D)
	{
		m_glWindow->removeFromOwnDB(poly);
	}
	
	emit update_tree();

	resetDraw();
}

