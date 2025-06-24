#include "qRoadMarking.h"

#include <QObject>
#include <QPoint>
#include <QMouseEvent>
#include <QCoreApplication>

#include "ccGLWindow.h"
#include "qSignExtractDlg.h"
#include "PointCloudIO.h"

using namespace roadmarking;

qRoadMarking::qRoadMarking(QObject* parent)
	: QObject(parent),
	ccStdPluginInterface(":/CC/plugin/qRoadMarking/info.json")
{
}

QIcon qRoadMarking::getIcon() const
{
	return QIcon("");
}

void qRoadMarking::onNewSelection(const ccHObject::Container& selectedEntities)
{
	// 判断是否选中了点云
	bool hasCloud = false;
	for (ccHObject* entity : selectedEntities)
	{
		if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
		{
			hasCloud = true;
			break;
		}
	}

	if (m_startDlg)
	{
		m_startDlg->setEnabled(hasCloud);
	}
}

QList<QAction*> qRoadMarking::getActions()
{
	if (!m_startDlg)
	{
		m_startDlg = new QAction(getName(), this);
		m_startDlg->setToolTip(getDescription());
		m_startDlg->setIcon(getIcon());
		connect(m_startDlg, &QAction::triggered, this, &qRoadMarking::doAction);
	}
	return QList<QAction*>{ m_startDlg };
}

void qRoadMarking::doAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	auto clouds = PointCloudIO::get_selected_clouds_from_DB(m_app);
	if (!clouds.size())
	{
		m_app->dispToConsole("未选择点云");
		return;
	}
	
	qSignExtractDlg dlg(m_app);

	//the widget should be visible before we add the cloud
	dlg.show();
	QCoreApplication::processEvents();

	//automatically deselect the input cloud
	for (auto cloud : clouds)
	{
		m_app->setSelectedInDB(cloud, false);

	}

	if (dlg.setCloud(clouds))
	{
		dlg.exec();
	}

	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}
