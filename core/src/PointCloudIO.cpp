#include "PointCloudIO.h"
#include "comm.h"
#include "CloudProcess.h"

using namespace roadmarking;

ccCloudPtr PointCloudIO::get_selected_cloud_from_DB(ccMainAppInterface* p_app)
{
	const ccHObject::Container& selectedEntities = p_app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		p_app->dispToConsole("请先选择至少一个点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return nullptr;
	}
	for (ccHObject* ent : selectedEntities)
	{
		if (ent->isA(CC_TYPES::POINT_CLOUD))
		{
			// 这里不管理当前点云，使用空删除器
			return ccCloudPtr(static_cast<ccPointCloud*>(ent), [](ccPointCloud*) {});
		}
	}
	p_app->dispToConsole("选择的实体不是点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	return nullptr;
}

ccCloudPtr PointCloudIO::convert_to_ccCloudPtr(ccPointCloud* p_cloud)
{
	// 这里不管理当前点云，使用空删除器
	return ccCloudPtr(static_cast<ccPointCloud*>(p_cloud), [](ccPointCloud*) {});
}

std::vector<ccHObject*> PointCloudIO::get_selected_clouds_from_DB(ccMainAppInterface* app)
{
	const ccHObject::Container& selectedEntities = app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		app->dispToConsole("请先选择至少一个点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return {};
	}
	return selectedEntities;
}

ccCloudPtr PointCloudIO::convert_to_ccCloudPtr(ccHObject* ob, ccMainAppInterface* app)
{
	if (ob == nullptr)return nullptr;
	if (ob->isA(CC_TYPES::POINT_CLOUD))
	{
		return ccCloudPtr(static_cast<ccPointCloud*>(ob), [](ccPointCloud*) {});
	}
	if(app)app->dispToConsole("选择的实体不是点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	return nullptr;
}

void PointCloudIO::save_ccCloudPtr_to_DB(ccMainAppInterface* app, ccCloudPtr cloud)
{
	// 完全转移点云所有权，交由主界面管理
	ccPointCloud* cccloud = cloud.release();

	app->addToDB(cccloud);
	app->refreshAll();
}

PCLCloudPtr PointCloudIO::convert_to_PCLCloudPtr(ccPointCloud* cloud)
{
	PCLCloudPtr pclCloud(new PCLCloud);
	size_t numPoints = cloud->size();
	pclCloud->resize(numPoints);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		const CCVector3& point = *cloud->getPoint(i);
		pclCloud->points[i] = pcl::PointXYZ(point.x, point.y, point.z);
	}

	return pclCloud;
}


PCLCloudPtr PointCloudIO::convert_to_PCLCloudPtr(ccCloudPtr cloud)
{
	return convert_to_PCLCloudPtr(cloud.get());
}


PCLCloudPtr PointCloudIO::convert_to_PCLCloudPtr(PCLCloudXYZIPtr cloud)
{
	PCLCloudPtr pclCloud(new PCLCloud);
	size_t numPoints = cloud->size();
	pclCloud->resize(numPoints);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		const PCLPointXYZI& point = (*cloud)[i];
		pclCloud->points[i] = pcl::PointXYZ(point.x, point.y, point.z);
	}

	return pclCloud;
}


ccCloudPtr PointCloudIO::convert_to_ccCloudPtr(PCLCloudPtr pclCloud)
{
	ccCloudPtr ccCloud(new ccPointCloud());
	size_t numPoints = pclCloud->size();
	ccCloud->reserve(numPoints);

	std::vector<CCVector3> tempPoints(numPoints);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		tempPoints[i] = CCVector3(pclCloud->points[i].x, pclCloud->points[i].y, pclCloud->points[i].z);
	}

	for (size_t i = 0; i < numPoints; ++i)
	{
		ccCloud->addPoint(tempPoints[i]);
	}
	return ccCloud;
}

PCLCloudXYZIPtr PointCloudIO::convert_to_PCLCloudPtr_with_XYZI(ccCloudPtr cloud)
{
	PCLCloudXYZIPtr pclCloud(new PCLCloudXYZI);
	size_t numPoints = cloud->size();
	pclCloud->resize(numPoints);

	int intensitySFIndex = PointCloudIO::get_intensity_idx(cloud);
	ccScalarField* intensitySF = (intensitySFIndex >= 0) ? static_cast<ccScalarField*>(cloud->getScalarField(intensitySFIndex)) : nullptr;
	float defaultIntensity = 0.0f;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		const CCVector3* point = cloud->getPoint(i);
		pcl::PointXYZI& pclPoint = pclCloud->points[i];
		pclPoint.x = point->x;
		pclPoint.y = point->y;
		pclPoint.z = point->z;
		pclPoint.intensity = (intensitySF) ? intensitySF->getValue(i) : defaultIntensity;
	}

	return pclCloud;
}


ccCloudPtr PointCloudIO::convert_to_ccCloudPtr_with_XYZI(PCLCloudXYZIPtr pclCloud)
{
	ccCloudPtr ccCloud(new ccPointCloud());

	ccScalarField* intensitySF = nullptr;
	intensitySF = new ccScalarField("Intensity");
	ccCloud->addScalarField(intensitySF);

	for (const auto& pclPoint : pclCloud->points)
	{
		CCVector3 ccPoint(pclPoint.x, pclPoint.y, pclPoint.z);
		ccCloud->addPoint(ccPoint);
		intensitySF->addElement(static_cast<float>(pclPoint.intensity));
	}
	intensitySF->computeMinAndMax();
	return ccCloud;
}

int PointCloudIO::get_intensity_idx(ccCloudPtr cloud)
{
	return get_intensity_idx(cloud.get());
}

int PointCloudIO::get_intensity_idx(ccPointCloud* cloud)
{
	if (!cloud)
		return -1;

	int sfIdx = -1;
	const int sfCount = cloud->getNumberOfScalarFields();
	for (int i = 0; i < sfCount; ++i)
	{
		if (QString::fromStdString(cloud->getScalarField(i)->getName()).contains("intensity", Qt::CaseInsensitive))
		{
			sfIdx = i;
			break;
		}
	}
	return sfIdx;
}



ccPointCloud* PointCloudIO::get_ground_cloud(ccPointCloud* cloud)
{
	QString name = cloud->getName() + "_ground_part";
	for (int i = 0; i < cloud->getChildrenNumber(); i++)
	{
		if(cloud->getChild(i)->getName() == name)return static_cast<ccPointCloud*>(cloud->getChild(i));
	}
	ccCloudPtr ground = CloudProcess::apply_csf_ground_extraction(PointCloudIO::convert_to_ccCloudPtr(cloud));
	cloud->addChild(ground.release());
	ground->setName(name);
	return ground.get();
}
