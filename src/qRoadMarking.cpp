#include "qRoadMarking.h"

#include <QObject>
#include <QPoint>
#include <QMouseEvent>
#include <QCoreApplication>

#include "ccGLWindow.h"
#include "qSignExtractDlg.h"

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


	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	if (!m_app->haveOneSelection() || !selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* cloud = selectedEntities.front();
	
	qSignExtractDlg dlg(m_app);

	//the widget should be visible before we add the cloud
	dlg.show();
	QCoreApplication::processEvents();

	//automatically deselect the input cloud
	m_app->setSelectedInDB(cloud, false);

	if (dlg.setCloud((ccPointCloud*)cloud))
	{
		dlg.exec();
	}

	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}
