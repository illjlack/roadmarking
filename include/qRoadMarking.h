#pragma once

#include "ccStdPluginInterface.h"

/// <summary>
/// qRoadMarking�����cc�Ľӿ�
/// </summary>
class qRoadMarking : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)

	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qRoadMarking" FILE "../info.json")

public:
	explicit qRoadMarking(QObject* parent = nullptr);
	~qRoadMarking() override = default;

	QString getName() const override { return "RoadMarking"; }
	QString getDescription() const override { return "Extracts road markings from point clouds"; }
	QIcon getIcon() const override;

	void onNewSelection(const ccHObject::Container& selectedEntities) override;

	QList<QAction*> getActions() override;


protected:


private slots:
};
