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

	ccCloudPtr filteredCloud = CloudProcess::apply_voxel_grid_filtering(inputCloud, voxelSize);
	filteredCloud->setName("downsampledCloud");

	if (m_app)
		timer.elapsed(m_app, "体素滤波完成");
	return filteredCloud;
}

ccCloudPtr RoadMarkingExtract::applyCSFGroundExtraction(ccCloudPtr inputCloud, ccMainAppInterface* m_app)
{
	Timer timer("CSF 地面提取处理");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("未选择点云");
		return nullptr;
	}

	ccCloudPtr filteredCloud = CloudProcess::apply_csf_ground_extraction(inputCloud);
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
	Timer timer("欧式聚类提取最大连通");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("未选择点云");
		return nullptr;
	}

	PCLCloudPtr pclCloud = PointCloudIO::convert_to_PCLCloudPtr(inputCloud);
	PCLCloudPtr filteredCloud = CloudProcess::extract_max_cloud_by_euclidean_cluster(pclCloud, clusterRadius);
	if (!filteredCloud)
	{
		if (m_app) m_app->dispToConsole("找不到最大连通");
		return nullptr;
	}

	ccCloudPtr ccFilteredCloud = CloudProcess::crop_raw_by_sparse_cloud(inputCloud, filteredCloud);
	ccFilteredCloud->setName("LargestRoadComponent");

	if (m_app)
		timer.elapsed(m_app, "找到最大连通");
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

	PCLCloudPtr pclCloud = PointCloudIO::convert_to_PCLCloudPtr(inputCloud);
	if (m_app) timer.restart(m_app, "转换pcl数据");

	// 2.================================体素滤波
	PCLCloudPtr voxelCloud = CloudProcess::apply_voxel_grid_filtering(pclCloud, voxelSize);
	if (m_app) timer.restart(m_app, "体素滤波成功");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convert_to_ccCloudPtr(voxelCloud);
		bugCloud->setName("VoxelGridFiltering");
		CloudProcess::apply_default_intensity_and_visible(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	// 3.================================csf
	ccCloudPtr groundCloud = CloudProcess::apply_csf_ground_extraction(PointCloudIO::convert_to_ccCloudPtr(voxelCloud));
	if (!groundCloud)
	{
		if (m_app) timer.elapsed(m_app, "CSF 处理失败");
		return;
	}
	if (m_app) timer.restart(m_app, "CSF 处理完成，提取地面成功");

#ifdef DEBUG
	{
		if (groundCloud)
		{
			groundCloud->setName("groundCloud");
			CloudProcess::apply_default_intensity_and_visible(groundCloud);
			inputCloud->addChild(groundCloud.release());
		}
	}
#endif // DEBUG

	// 4.=====================================欧式聚类，提取最大连通
	PCLCloudPtr largestClusterCloud = CloudProcess::extract_max_cloud_by_euclidean_cluster(PointCloudIO::convert_to_PCLCloudPtr(groundCloud), clusterRadius);
	if (!largestClusterCloud)
	{
		if (m_app) timer.elapsed(m_app, "找不到最大连通");
		return;
	}
	if (m_app) timer.restart(m_app, "欧式聚类，找到最大连通");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convert_to_ccCloudPtr(largestClusterCloud);
		bugCloud->setName("euclideanClusterCloud");
		CloudProcess::apply_default_intensity_and_visible(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	// 5.====================================区域生长算法提取道路点
	PCLCloudPtr roadCloud = CloudProcess::extract_road_points(largestClusterCloud, searchRadius, angleThreshold, curvatureThreshold);
	if (m_app) timer.restart(m_app, "道路点提取完成");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convert_to_ccCloudPtr(roadCloud);
		bugCloud->setName("roadCloud");
		CloudProcess::apply_default_intensity_and_visible(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	ccCloudPtr oRoadCloud = CloudProcess::crop_raw_by_sparse_cloud(inputCloud, roadCloud);
	if (m_app) timer.restart(m_app, "密集化道路点提取完成");

#ifdef DEBUG
	{
		oRoadCloud->setName("oroadCloud");
		CloudProcess::apply_default_intensity_and_visible(oRoadCloud);
		inputCloud->addChild(oRoadCloud.release());
	}
#endif // DEBUG

	auto markingClouds = CloudProcess::extract_roadmarking(oRoadCloud, 0.03);
	if (m_app) timer.restart(m_app, "路标提取成功");

#ifdef DEBUG
	{
		ccHObject* parentNode = new ccHObject("Clusters_Parent");
		size_t clusterId = 0;
		for (auto& cluster : markingClouds)
		{
			ccCloudPtr markingCloud = PointCloudIO::convert_to_ccCloudPtr_with_XYZI(cluster);
			markingCloud->setName(QString("Cluster_%1").arg(clusterId++));
			CloudProcess::apply_default_intensity_and_visible(markingCloud);
			parentNode->addChild(markingCloud.release());
		}
		parentNode->setVisible(true);
		inputCloud->addChild(parentNode);
	}
#endif // DEBUG


	std::vector<PCLCloudPtr> pclClouds;
	for (auto markingCloud : markingClouds)
	{
		pclClouds.push_back(PointCloudIO::convert_to_PCLCloudPtr(markingCloud));
	}

	auto markings = CloudProcess::apply_roadmarking_vectorization(pclClouds);
	markings->setName("roadmarking");
	inputCloud->addChild(markings);

	if (m_app) timer.elapsed(m_app, "处理完成,一共用时");
}
