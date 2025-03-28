#include "qRoadMarking.h"

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

	// 如果选择了点云
}

QList<QAction*> qRoadMarking::getActions()
{
	return QList<QAction*>{ nullptr };
}
