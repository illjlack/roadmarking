#include "RoadMarkingExtract.h"
#include <QMap>
#include <QVariant>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QDebug>
#include <cmath>

#include "CloudProcess.h"

using namespace roadmarking;


ccCloudPtr RoadMarkingExtract::applyVoxelGridFiltering(ccCloudPtr inputCloud, float voxelSize, ccMainAppInterface* m_app)
{
	Timer timer("体素滤波");
	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("未选择点云");
		return nullptr;
	}

	ccCloudPtr filteredCloud = CloudProcess::applyVoxelGridFiltering(inputCloud, voxelSize);
	filteredCloud->setName("downsampledCloud");

	if (m_app)
		timer.elapsed(m_app, "体素滤波完成");
	return filteredCloud;
}

ccCloudPtr RoadMarkingExtract::applyCSFGroundExtraction(ccCloudPtr inputCloud, ccMainAppInterface* m_app)
{
	Timer timer("CSF 提取地面点云");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("未选择点云");
		return nullptr;
	}

	ccCloudPtr filteredCloud = CloudProcess::applyCSFGroundExtraction(inputCloud);
	if (!filteredCloud)
	{
		if (m_app) m_app->dispToConsole("CSF 处理失败");
		return nullptr;
	}

	filteredCloud->setName("groundCloud");

	if (m_app)
		timer.elapsed(m_app, "CSF 处理完成");
	return filteredCloud;
}

ccCloudPtr RoadMarkingExtract::extractLargestComponent(ccCloudPtr inputCloud, float clusterRadius, ccMainAppInterface* m_app)
{
	Timer timer("欧式聚类提取最大连通块");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("未选择点云");
		return nullptr;
	}

	PCLCloudPtr pclCloud = PointCloudIO::convertToPCLCloud(inputCloud);
	PCLCloudPtr filteredCloud = CloudProcess::extractMaxCloudByEuclideanCluster(pclCloud, clusterRadius);
	if (!filteredCloud)
	{
		if (m_app) m_app->dispToConsole("找不到最大联通块");
		return nullptr;
	}

	ccCloudPtr ccFilteredCloud = CloudProcess::CropBySparseCloud(inputCloud, filteredCloud);
	ccFilteredCloud->setName("LargestRoadComponent");

	if (m_app)
		timer.elapsed(m_app, "找到最大连通块");
	return ccFilteredCloud;
}

void RoadMarkingExtract::automaticExtraction(ccCloudPtr inputCloud,
	ccMainAppInterface* m_app, float voxelSize,
	float clusterRadius, float curvatureThreshold,
	float angleThreshold, float searchRadius
	)
{
	Timer timer("全自动提取");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("未选择点云");
		return;
	}

	PCLCloudPtr pclCloud = PointCloudIO::convertToPCLCloud(inputCloud);
	if (m_app) timer.restart(m_app, "转换pcl点云");

	// 2.================================体素滤波
	PCLCloudPtr voxelCloud = CloudProcess::applyVoxelGridFiltering(pclCloud, voxelSize);
	if (m_app) timer.restart(m_app, "体素滤波成功");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convertToCCCloud(voxelCloud);
		bugCloud->setName("VoxelGridFiltering");
		applyDefaultIntensityDisplay(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	// 3.================================csf
	ccCloudPtr groundCloud = CloudProcess::applyCSFGroundExtraction(PointCloudIO::convertToCCCloud(voxelCloud));
	if (!groundCloud)
	{
		if (m_app) timer.elapsed(m_app, "CSF 处理失败");
		return;
	}
	if (m_app) timer.restart(m_app, "CSF 处理完成，提取出地面点云");

#ifdef DEBUG
	{
		if (groundCloud)
		{
			groundCloud->setName("groundCloud");
			applyDefaultIntensityDisplay(groundCloud);
			inputCloud->addChild(groundCloud.release());
		}
	}
#endif // DEBUG

	// 4.=====================================欧式聚类，提取最大联通块
	PCLCloudPtr largestClusterCloud = CloudProcess::extractMaxCloudByEuclideanCluster(PointCloudIO::convertToPCLCloud(groundCloud), clusterRadius);
	if (!largestClusterCloud)
	{
		if (m_app) timer.elapsed(m_app, "找不到最大联通块");
		return;
	}
	if (m_app) timer.restart(m_app, "欧式聚类，找到最大联通块");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convertToCCCloud(largestClusterCloud);
		bugCloud->setName("euclideanClusterCloud");
		applyDefaultIntensityDisplay(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	// 5.====================================生长算法提取道路点云
	PCLCloudPtr roadCloud = CloudProcess::extractRoadPoints(largestClusterCloud, searchRadius, angleThreshold, curvatureThreshold);
	if (m_app) timer.restart(m_app, "道路点云提取完成");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convertToCCCloud(roadCloud);
		bugCloud->setName("roadCloud");
		applyDefaultIntensityDisplay(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	ccCloudPtr oRoadCloud = CloudProcess::CropBySparseCloud(inputCloud, roadCloud);
	if (m_app) timer.restart(m_app, "密集道路点云提取完成");

#ifdef DEBUG
	{
		oRoadCloud->setName("oroadCloud");
		applyDefaultIntensityDisplay(oRoadCloud);
		inputCloud->addChild(oRoadCloud.release());
	}
#endif // DEBUG

	auto markingClouds = CloudProcess::extractRoadMarking(oRoadCloud, 0.03);
	if (m_app) timer.restart(m_app, "路标点云提取成功");

#ifdef DEBUG
	{
		ccHObject* parentNode = new ccHObject("Clusters_Parent");
		size_t clusterId = 0;
		for (auto& cluster : markingClouds)
		{
			ccCloudPtr markingCloud = PointCloudIO::convertToCCCloudXYZI(cluster);
			markingCloud->setName(QString("Cluster_%1").arg(clusterId++));
			applyDefaultIntensityDisplay(markingCloud);
			parentNode->addChild(markingCloud.release());
		}
		parentNode->setVisible(true);
		inputCloud->addChild(parentNode);
	}
#endif // DEBUG


	std::vector<PCLCloudPtr> pclClouds;
	for (auto markingCloud : markingClouds)
	{
		pclClouds.push_back(PointCloudIO::convertToPCLCloud(markingCloud));
	}

	auto markings = CloudProcess::applyVectorization(pclClouds);
	markings->setName("roadmarking");
	inputCloud->addChild(markings);

	if (m_app) timer.elapsed(m_app, "处理完成,一共耗时");
}

void RoadMarkingExtract::applyDefaultIntensityDisplay(ccCloudPtr cloud)
{
	if (!cloud)
		return;

	// 查找名为“intensity”的标量字段
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

	// 如果找到了强度标量字段
	if (sfIdx >= 0)
	{
		// 设置该标量字段作为颜色显示
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);  // 显示标量字段
		cloud->showColors(true);  // 启用颜色显示
	}
	else
	{
		// 如果没有强度标量字段，保持默认行为
		cloud->showSF(false);
		cloud->showColors(false);
	}
	cloud->setVisible(true);
}
