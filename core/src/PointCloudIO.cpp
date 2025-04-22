#include "PointCloudIO.h"
#include "comm.h"

using namespace roadmarking;

ccCloudPtr PointCloudIO::getSelectedCloud(ccMainAppInterface* app)
{
	const ccHObject::Container& selectedEntities = app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		app->dispToConsole("请先选择至少一个点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return nullptr;
	}
	for (ccHObject* ent : selectedEntities)
	{
		if (ent->isA(CC_TYPES::POINT_CLOUD))
		{
			// 生命周期归前端管理, 使用空的析构函数
			return ccCloudPtr(static_cast<ccPointCloud*>(ent), [](ccPointCloud*) {});
		}
	}
	app->dispToConsole("选择的实体不是点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	return nullptr;
}

std::vector<ccHObject*> PointCloudIO::getSelectedClouds(ccMainAppInterface* app)
{
	const ccHObject::Container& selectedEntities = app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		app->dispToConsole("请先选择至少一个点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return {};
	}

	return selectedEntities;
}

ccCloudPtr PointCloudIO::convertToCCCloud(ccHObject* ob, ccMainAppInterface* app)
{
	if (ob == nullptr)return nullptr;
	if (ob->isA(CC_TYPES::POINT_CLOUD))
	{
		return ccCloudPtr(static_cast<ccPointCloud*>(ob), [](ccPointCloud*) {});
	}
	if(app)app->dispToConsole("选择的实体不是点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	return nullptr;
}

void PointCloudIO::saveCloudToDB(ccMainAppInterface* app, ccCloudPtr cloud)
{
	// 完全获得点云的所有权,生命周期交给前端控制
	ccPointCloud* cccloud = cloud.release();

	app->addToDB(cccloud);
	app->refreshAll();
}

#include <omp.h>
#include "..\include\PointCloudIO.h"

PCLCloudPtr PointCloudIO::convertToPCLCloud(ccCloudPtr cloud)
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


PCLCloudPtr PointCloudIO::convertToPCLCloud(PCLCloudXYZIPtr cloud)
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


ccCloudPtr PointCloudIO::convertToCCCloud(PCLCloudPtr pclCloud)
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

PCLCloudXYZIPtr PointCloudIO::convertToPCLCloudXYZI(ccCloudPtr cloud)
{
	PCLCloudXYZIPtr pclCloud(new PCLCloudXYZI);
	size_t numPoints = cloud->size();
	pclCloud->resize(numPoints);

	int intensitySFIndex = PointCloudIO::getIntensityIdx(cloud);
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


ccCloudPtr PointCloudIO::convertToCCCloudXYZI(PCLCloudXYZIPtr pclCloud)
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

int PointCloudIO::getIntensityIdx(ccCloudPtr cloud)
{
	return getIntensityIdx(cloud.get());
}

int PointCloudIO::getIntensityIdx(ccPointCloud* cloud)
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

