#include "CloudProcess.h"
#include <CSF.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <queue>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include "GridProcess.h"
#include "TemplateMatcher.h"
#include <ccPolyline.h>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <QFileDialog>
#include <QMessageBox>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_hull.h>
#include "ccProgressDialog.h"      // CloudCompare 的 Qt 进度对话框封装
#include <GenericProgressCallback.h>  // CCLib 通用回调接口
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace roadmarking;

/// <summary>
/// 使用射线法判断点是否在多边形内
/// 算法原理：从点向右发射一条射线，统计与多边形边界的交点数
/// 如果交点数为奇数，则点在多边形内；如果为偶数，则点在多边形外
/// </summary>
/// <param name="polygon">多边形顶点集合</param>
/// <param name="point">待判断的点</param>
/// <returns>true表示点在多边形内，false表示在多边形外</returns>
inline bool pointInPolygonRaycast(const std::vector<CCVector3d>& polygon, const CCVector3d& point)
{
	if (polygon.size() < 3)
		return false;

	int crossings = 0;
	const size_t n = polygon.size();

	for (size_t i = 0; i < n; i++)
	{
		const CCVector3d& p1 = polygon[i];
		const CCVector3d& p2 = polygon[(i + 1) % n];

		// 检查射线是否与边相交
		// 射线条件：p1.y <= point.y < p2.y 或 p2.y <= point.y < p1.y
		if (((p1.y <= point.y) && (p2.y > point.y)) ||
			((p1.y > point.y) && (p2.y <= point.y)))
		{
			// 计算交点的x坐标
			double xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
			
			// 如果交点在射线上（point.x <= xinters），则计数加1
			if (point.x <= xinters)
				crossings++;
		}
	}

	// 交点数为奇数表示在多边形内
	return (crossings % 2) == 1;
}

PCLOctreePtr CloudProcess::build_octree(PCLCloudPtr pclCloud, float targetVoxelSize)
{
	PCLOctreePtr pclOctree(new PCLOctree(targetVoxelSize));
	pclOctree->setInputCloud(pclCloud);
	pclOctree->addPointsFromInputCloud();
	return pclOctree;
}

ccCloudPtr CloudProcess::crop_raw_by_sparse_cloud(ccCloudPtr ccCloud, PCLCloudPtr pclCloud, PCLOctreePtr octree)
{
	ccPointCloud* croppedCloud = new ccPointCloud();
	std::unordered_set<int> unique_indices;

	std::vector<int> scalarFieldIndices(ccCloud->getNumberOfScalarFields());
	std::iota(scalarFieldIndices.begin(), scalarFieldIndices.end(), 0);

	if (!octree)
	{
		octree = build_octree(PointCloudIO::convert_to_PCLCloudPtr(ccCloud), 0.2);
	}

	size_t targetSize = pclCloud->size();
	croppedCloud->reserve(targetSize);

	std::vector<ccScalarField*> croppedScalarFields;
	for (int sfIdx : scalarFieldIndices)
	{
		ccScalarField* newField = new ccScalarField(ccCloud->getScalarField(sfIdx)->getName());
		newField->reserve(pclCloud->size());
		croppedCloud->addScalarField(newField);
		croppedScalarFields.push_back(newField);
	}

	for (int i = 0; i < pclCloud->size(); ++i)
	{
		const pcl::PointXYZ& p = pclCloud->points[i];
		std::vector<int> voxel_indices;
		if (octree->voxelSearch(p, voxel_indices))
		{
			for (const int idx : voxel_indices)
			{
				if (unique_indices.insert(idx).second)
				{
					const CCVector3* point = ccCloud->getPoint(idx);
					croppedCloud->addPoint(*point);

					for (size_t j = 0; j < scalarFieldIndices.size(); ++j)
					{
						int sfIdx = scalarFieldIndices[j];
						ScalarType val = ccCloud->getScalarField(sfIdx)->getValue(idx);
						croppedScalarFields[j]->addElement(val);
					}
				}
			}
		}
	}

	// **计算标量字段的最小最大值**
	for (ccScalarField* field : croppedScalarFields)
	{
		if (field)
			field->computeMinAndMax();
	}

	return ccCloudPtr(croppedCloud);
}


#include <ccPointCloud.h>
#include <unordered_set>
#include <vector>
#include <omp.h> // OpenMP 进行并行加速
#include <comm.h>

ccCloudPtr CloudProcess::apply_voxel_grid_filtering(ccCloudPtr ccCloud, float targetVoxelSize, PCLOctreePtr octree)
{
	// 创建新的点云
	ccPointCloud* croppedCloud = new ccPointCloud();

	// 记录已经添加的点索引，使用 vector<bool> 替代 unordered_set 以提高查找效率
	std::vector<bool> unique_indices(ccCloud->size(), false);

	// 获取所有标量字段
	std::vector<int> scalarFieldIndices(ccCloud->getNumberOfScalarFields());
	std::iota(scalarFieldIndices.begin(), scalarFieldIndices.end(), 0);

	// 获取点云数据指针
	const CCVector3* points = ccCloud->getPoint(0);

	if (!octree)
	{
		octree = build_octree(PointCloudIO::convert_to_PCLCloudPtr(ccCloud), 0.2);
	}

	// 转换为 PCL 格式点云
	PCLCloudPtr pclCloud = PointCloudIO::convert_to_PCLCloudPtr(ccCloud);
	// 体素网格滤波
	PCLCloudPtr filteredCloud = apply_voxel_grid_filtering(pclCloud, targetVoxelSize);

	croppedCloud->reserve(filteredCloud->points.size()); // 预分配，减少动态扩容
	for (int sfIdx : scalarFieldIndices)
	{
		ccScalarField* newField = new ccScalarField(ccCloud->getScalarField(sfIdx)->getName());
		newField->reserve(filteredCloud->points.size());
		croppedCloud->addScalarField(newField);
	}

	// 使用 OpenMP 并行处理
#pragma omp parallel for
	for (int i = 0; i < filteredCloud->points.size(); i++)
	{
		const auto& filteredPoint = filteredCloud->points[i];
		std::vector<int> voxel_indices;
		std::vector<float> distances;

		// 直接获取体素中心，避免最近邻搜索
		if (octree->nearestKSearch(filteredPoint, 1, voxel_indices, distances))
		{
			int idx = voxel_indices[0];

			// 确保线程安全更新唯一索引
#pragma omp critical
			{
				if (!unique_indices[idx])
				{
					unique_indices[idx] = true;
					croppedCloud->addPoint(points[idx]);

					for (int sfIdx : scalarFieldIndices)
					{
						ScalarType val = ccCloud->getScalarField(sfIdx)->getValue(idx);
						croppedCloud->getScalarField(sfIdx)->addElement(val);
					}
				}
			}
		}
	}

	for (int sfIdx : scalarFieldIndices)
	{
		croppedCloud->getScalarField(sfIdx)->computeMinAndMax();
	}

	return ccCloudPtr(croppedCloud);
}

PCLCloudPtr CloudProcess::apply_voxel_grid_filtering(PCLCloudPtr pclCloud, float targetVoxelSize)
{
	pcl::octree::OctreePointCloudVoxelCentroid<PCLPoint> octree(targetVoxelSize);
	octree.setInputCloud(pclCloud);
	octree.addPointsFromInputCloud();

	std::vector<PCLPoint, Eigen::aligned_allocator<PCLPoint>> voxelCentroids;
	octree.getVoxelCentroids(voxelCentroids);

	PCLCloudPtr filteredCloud(new PCLCloud);
	filteredCloud->reserve(voxelCentroids.size());
	for (const auto& p : voxelCentroids)
	{
		filteredCloud->push_back({ p.x,p.y,p.z });
	}
	return filteredCloud;
}

ccCloudPtr CloudProcess::apply_csf_ground_extraction(ccCloudPtr ccCloud)
{
	CSF::Parameters csfParams;
	csfParams.cloth_resolution = 2.0;
	csfParams.class_threshold = 0.3;
	csfParams.rigidness = 2;
	csfParams.iterations = 500;
	csfParams.smoothSlope = true;

	// **执行 CSF**
	ccPointCloud* groundCloud = nullptr;
	ccPointCloud* offGroundCloud = nullptr;
	ccMesh* clothMesh = nullptr;

	bool result = CSF::Apply(ccCloud.get(), csfParams, groundCloud, offGroundCloud, false, clothMesh);

	if (offGroundCloud) delete offGroundCloud;
	//  clothMesh 未开启

	if (result)
	{
		return ccCloudPtr(groundCloud);
	}
	if (groundCloud) delete groundCloud;
	return nullptr;
}

PCLCloudPtr CloudProcess::apply_csf_ground_extraction(PCLCloudPtr pclCloud)
{
	ccCloudPtr ccCloud = PointCloudIO::convert_to_ccCloudPtr(pclCloud);
	ccCloudPtr groundCloud = apply_csf_ground_extraction(ccCloud);
	PCLCloudPtr pclGroundCloud = PointCloudIO::convert_to_PCLCloudPtr(groundCloud);
	return pclGroundCloud;
}

PCLCloudPtr CloudProcess::extract_max_cloud_by_euclidean_cluster(PCLCloudPtr groundCloud, float euclideanClusterRadius)
{
	std::vector<PCLCloudPtr> clouds;
	extract_euclidean_clusters<PCLPoint>(groundCloud, clouds, euclideanClusterRadius);
	if (clouds.size() == 0)
	{
		return nullptr;
	}
	auto largestCluster = std::max_element(clouds.begin(), clouds.end(),
		[](const auto& a, const auto& b) { return a->size() < b->size(); });
	return *largestCluster;
}

PCLCloudPtr CloudProcess::extract_road_points(PCLCloudPtr groundCloud,
	float searchRadius,
	float angleThreshold,
	float curvatureBaseThreshold,
	PCLPoint* pickSeedPoint)
{
	angleThreshold = cos(angleThreshold * M_PI / 180.0);
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	std::vector<float> curvatures;

	PCLPoint seedPoint;
	if (pickSeedPoint)
	{
		seedPoint = *pickSeedPoint;
	}
	else
	{
		pcl::computeCentroid(*groundCloud, seedPoint);
	}

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne.setNumberOfThreads(8);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(searchRadius);
	ne.setInputCloud(groundCloud);
	normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*normals);

	curvatures.resize(groundCloud->size());
	for (size_t i = 0; i < normals->size(); ++i)
	{
		curvatures[i] = normals->points[i].curvature;
	}

	std::vector<int> pointIdxVec;
	std::vector<float> distances;

	{// 算完释放空间
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> seedOctree(1.0);
		seedOctree.setInputCloud(groundCloud);
		seedOctree.addPointsFromInputCloud();
		seedOctree.nearestKSearch(seedPoint, 10, pointIdxVec, distances);
	}

	std::queue<int> seedQueue;
	std::vector<bool> processed(groundCloud->size(), false);
	std::vector<int> roadIndices;
	tree->setInputCloud(groundCloud);
	for (auto& pIdx : pointIdxVec)
	{
		seedQueue.push(pIdx);
	}

	while (!seedQueue.empty())
	{
		int currIdx = seedQueue.front();
		seedQueue.pop();
		roadIndices.push_back(currIdx);

		std::vector<int> neighborIndices;
		std::vector<float> neighborDistances;
		tree->radiusSearch(groundCloud->points[currIdx], searchRadius, neighborIndices, neighborDistances);

		for (int neighborIdx : neighborIndices)
		{
			if (processed[neighborIdx]) continue;

			Eigen::Vector3f n1(normals->points[currIdx].normal_x, normals->points[currIdx].normal_y, normals->points[currIdx].normal_z);
			Eigen::Vector3f n2(normals->points[neighborIdx].normal_x, normals->points[neighborIdx].normal_y, normals->points[neighborIdx].normal_z);
			float dotProduct = fabs(n1.dot(n2));

			if (curvatures[neighborIdx] > curvatureBaseThreshold)
				continue;

			if (dotProduct > angleThreshold)
			{
				seedQueue.push(neighborIdx);
				processed[neighborIdx] = true;
			}
		}
	}

	PCLCloudPtr roadCloud(new PCLCloud);
	for (int idx : roadIndices)
	{
		roadCloud->points.push_back(groundCloud->points[idx]);
	}
	return roadCloud;
}

std::vector<PCLCloudXYZIPtr> CloudProcess::extract_roadmarking(ccCloudPtr roadCloud,
	float resolution,
	double euclideanClusterRadius,
	int minNum)
{
	GridProcess gridProcess(resolution);
	gridProcess.perform_orthogonal_grid_mapping(roadCloud);
	gridProcess.div_by_adaptive_intensity_threshold();
	PCLCloudXYZIPtr markingCloud(new PCLCloudXYZI);
	gridProcess.restore_from_grid_to_cloud(markingCloud);

	std::vector<PCLCloudXYZIPtr> clouds;
	extract_euclidean_clusters<PCLPointXYZI>(markingCloud, clouds, euclideanClusterRadius, minNum);

	return clouds;
}

PCLCloudPtr CloudProcess::match_roadmarking(PCLCloudPtr pclCloud)
{
	//TemplateMatcher matcher;
	//matcher.setScenePointCloud(pclCloud);
	//matcher.matchTemplates();
	//return matcher.getBestMatchCloud();
	return nullptr;
}

ccHObject* CloudProcess::apply_roadmarking_vectorization(ccPointCloud* cloud)
{
	return apply_roadmarking_vectorization(PointCloudIO::convert_to_ccCloudPtr(cloud));
}

ccHObject* CloudProcess::apply_roadmarking_vectorization(ccCloudPtr cloud)
{
	std::vector<PCLCloudPtr> clouds;
	extract_euclidean_clusters<PCLPoint>(PointCloudIO::convert_to_PCLCloudPtr(cloud), clouds, 0.2);
	return apply_roadmarking_vectorization(clouds);
}

ccHObject* visualizeRoadMarkings(const RoadMarkings& roadmarkings)
{
	ccHObject* allLinesContainer = new ccHObject();
	allLinesContainer->setName("RoadMarkings");

	// 遍历分类结果中的每个 roadmarking
	for (const auto& roadmarking : roadmarkings)
	{
		// 跳过非折线类型（假设只有 polylines 里有内容才处理）
		if (roadmarking.polylines.empty())
			continue;

		MetaRoadmarking* meta = new MetaRoadmarking;
		meta->setName(QString::fromStdString(roadmarking.name + " 准确率（重叠率）:" + std::to_string(roadmarking.accuracy * 100) + "%"));

		// 对 roadmarking.polylines 中的每一条折线都创建一个 ccPolyline
		for (const auto& singlePolyline : roadmarking.polylines)
		{
			if (singlePolyline.size() < 2)
				continue; // 至少要两个点才算"折线"

			// 创建一个 ccPointCloud 来存储这一条折线的点
			ccCloudPtr polylineCloud(new ccPointCloud);

			// 将 singlePolyline 中的点添加到 polylineCloud
			for (const auto& pt : singlePolyline)
			{
				polylineCloud->addPoint({ pt.x, pt.y, pt.z });
			}

			// 创建 ccPolyline，并将所有点索引加入
			ccPolyline* polylineObj = new ccPolyline(polylineCloud.release());

			// 把 ccPointCloud 注册为 ccPolyline 的子节点，以便在场景里一起删除
			polylineObj->addChild(polylineCloud.release());

			// 为折线预留索引空间
			polylineObj->reserve(static_cast<unsigned>(singlePolyline.size()));

			// 将每个顶点的索引依次添加
			for (unsigned idx = 0; idx < singlePolyline.size(); ++idx)
			{
				polylineObj->addPointIndex(idx);
			}

			// 设置可见性和颜色
			polylineObj->setVisible(true);
			polylineObj->setColor(ccColor::orangeRGB);
			polylineObj->showColors(true);

			// 将折线加入到 meta 中
			meta->addChild(polylineObj);
		}
		// 将 meta 添加到 allLinesContainer
		allLinesContainer->addChild(meta);
	}

	allLinesContainer->setVisible(true);
	return allLinesContainer;
}

ccHObject* CloudProcess::apply_roadmarking_vectorization(std::vector<PCLCloudPtr> pclClouds)
{
	// 创建分类器对象
	RoadMarkingClassifier classifier;

	// 用于保存分类结果
	RoadMarkings roadmarkings;

	// 目前硬编码了 model.json 路径
	QString model_path = "F:\\RoadMarking\\install\\CloudCompare_debug\\model\\model.json";

	// 调用分类函数，得到 roadmarkings（其中每个 RoadMarking 现在包含 polylines 而非 polyline）
	classifier.ClassifyRoadMarkings(pclClouds, roadmarkings, model_path.toStdString());

	// 可视化分类结果
	ccHObject* allLinesContainer = visualizeRoadMarkings(roadmarkings);

	return allLinesContainer;
}

template <typename PointT>
void CloudProcess::extract_euclidean_clusters(
	typename pcl::PointCloud<PointT>::Ptr inputCloud,
	std::vector<typename pcl::PointCloud<PointT>::Ptr>& outputClusters,
	double euclideanClusterRadius,
	int minNum)
{
	// 创建KdTree搜索结构
	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(inputCloud);

	// 创建EuclideanClusterExtraction对象
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(euclideanClusterRadius);  // 设置聚类容忍度
	ec.setMinClusterSize(minNum);                    // 设置最小聚类大小
	ec.setSearchMethod(tree);                         // 设置搜索方法
	ec.setInputCloud(inputCloud);                     // 设置输入点云

	// 提取聚类
	std::vector<pcl::PointIndices> clusterIndices;
	ec.extract(clusterIndices);

	// 遍历每个聚类
	for (const auto& indices : clusterIndices)
	{
		if (indices.indices.size() < ec.getMinClusterSize()) continue;

		// 创建一个新的聚类点云
		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
		cluster->reserve(indices.indices.size());

		// 将聚类的点复制到新的点云中
		pcl::copyPointCloud(*inputCloud, indices, *cluster);

		// 使用统计滤波器移除离群点
		typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud(cluster);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*cloud_filtered);

		// 将过滤后的聚类点云加入输出
		outputClusters.push_back(cluster);
	}
}

PCLCloudPtr CloudProcess::extract_outline(const PCLCloudPtr& inputCloud, float alpha)
{
	if (!inputCloud || inputCloud->empty())
	{
		return nullptr;
	}

	PCLCloudPtr hullCloud(new PCLCloud);

	// 计算 Concave Hull（凹包）
	pcl::ConcaveHull<PCLPoint> concaveHull;
	concaveHull.setInputCloud(inputCloud);
	concaveHull.setAlpha(alpha); // α 值越小，轮廓越紧
	concaveHull.reconstruct(*hullCloud);

	// 如果点数不足，尝试使用 Convex Hull（凸包）
	if (hullCloud->size() < 3)
	{
		pcl::ConvexHull<PCLPoint> convexHull;
		convexHull.setInputCloud(inputCloud);
		convexHull.reconstruct(*hullCloud);
	}

	// 最终检查，确保轮廓有效
	if (hullCloud->size() < 3)
	{
		return nullptr; // 无效轮廓，返回空指针
	}

	return hullCloud;
}


//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/convex_hull_2.h>
//#include <CGAL/Polygon_2_algorithms.h>
//#include <vector>
//
//typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
//typedef K::Point_2 Point_2;
//typedef std::vector<Point_2> Polygon_2;
//
//void CloudProcess::cropPointCloudWithFineSelection(
//	const std::vector<ccPointCloud*>& clouds,            // 输入点云
//	const std::vector<CCVector3d>& polygon_points,     // 自定义的裁剪区域（多边形）
//	ccPointCloud* cloud_cropped                       // 输出裁剪后的点云
//)
//{
//	if (clouds.empty() || !cloud_cropped)
//	{
//		return;
//	}
//
//	// 使用CGAL表示多边形
//	Polygon_2 polygon_cgal;
//	for (const auto& pt : polygon_points)
//	{
//		polygon_cgal.push_back(Point_2(pt.x, pt.y));  // 将点云中的点转换为CGAL的Point_2
//	}
//
//	cloud_cropped->addScalarField("intensity");
//	auto _sf = cloud_cropped->getScalarField(cloud_cropped->getScalarFieldIndexByName("intensity"));
//
//	for (auto cloud : clouds)
//	{
//		int sfIdx = PointCloudIO::getIntensityIdx(cloud);
//		if (sfIdx > -1)
//		{
//			auto sf = cloud->getScalarField(sfIdx);
//			for (size_t i = 0; i < cloud->size(); ++i)
//			{
//				const auto& point = cloud->getPoint(i);
//				Point_2 pt_2d(point->x, point->y); // 将点云中的点转换为CGAL的Point_2
//
//				// 使用CGAL的bounded_side_2判断点是否在多边形内
//				if (CGAL::bounded_side_2(polygon_cgal.begin(), polygon_cgal.end(), pt_2d, K()) == CGAL::ON_BOUNDED_SIDE)
//				{
//					if (sf) {
//						cloud_cropped->addPoint(*point);
//						_sf->addElement(sf->getValue(i));
//					}
//				}
//			}
//		}
//	}
//}

void CloudProcess::apply_default_intensity_and_visible(ccCloudPtr cloud)
{
	apply_default_intensity_and_visible(cloud.get());
}

void CloudProcess::apply_default_intensity_and_visible(ccPointCloud* cloud)
{
	if (!cloud)
		return;

	int sfIdx = PointCloudIO::get_intensity_idx(cloud);

	// 如果找到了强度标量字段
	if (sfIdx >= 0)
	{
		// 设置该标量字段作为颜色显示
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);  // 显示标量字段
	}
	else
	{
		// 如果没有强度标量字段，保持默认行为
		cloud->showSF(false);
	}
	cloud->setVisible(true);
}


void CloudProcess::filter_cloud_by_intensity(
	ccPointCloud* inCloud,   // 输入点云列表
	double lowerThreshold,                      // 强度下限
	double upperThreshold,                      // 强度上限
	ccPointCloud* cloud_filtered                // 输出：阈值过滤后的点云
)
{
	if (!inCloud || !cloud_filtered)
		return;

	// 为输出点云创建一个"intensity"标量场
	cloud_filtered->addScalarField("intensity");
	int outSfIdx = cloud_filtered->getScalarFieldIndexByName("intensity");
	auto sfOut = cloud_filtered->getScalarField(outSfIdx);

	// 找到输入点云的"intensity"标量场索引
	int inSfIdx = PointCloudIO::get_intensity_idx(inCloud);
	if (inSfIdx < 0)return;

	auto sfIn = inCloud->getScalarField(inSfIdx);
	if (!sfIn)return;

	// 对每个点，判断其强度是否在阈值区间内
	for (size_t i = 0; i < inCloud->size(); ++i)
	{
		const CCVector3* P = inCloud->getPoint(i);
		double intensity = sfIn->getValue(i);

		if (intensity >= lowerThreshold && intensity <= upperThreshold)
		{
			// 符合条件：添加到输出点云，并同步强度值
			cloud_filtered->addPoint(*P);
			sfOut->addElement(intensity);
		}
	}

	// 重新计算输出标量场的最小/最大，用于后续显示
	sfOut->computeMinAndMax();

	// 设置当前显示标量场并应用默认显示
	cloud_filtered->setCurrentDisplayedScalarField(outSfIdx);
	CloudProcess::apply_default_intensity_and_visible(cloud_filtered);

	// 请求重绘（如果有可视化窗口）
	if (cloud_filtered->getDisplay())
		cloud_filtered->getDisplay()->redraw(false, true);
}

void CloudProcess::filter_cloud_by_z(
	ccPointCloud* inCloud,   // 输入点云列表
	double lowerThreshold,                      // 强度下限
	double upperThreshold,                      // 强度上限
	ccPointCloud* cloud_filtered                // 输出：阈值过滤后的点云
)
{
	if (!inCloud || !cloud_filtered)
		return;

	// 为输出点云创建一个"intensity"标量场
	cloud_filtered->addScalarField("intensity");
	int outSfIdx = cloud_filtered->getScalarFieldIndexByName("intensity");
	auto sfOut = cloud_filtered->getScalarField(outSfIdx);

	// 找到输入点云的"intensity"标量场索引
	int inSfIdx = PointCloudIO::get_intensity_idx(inCloud);
	if (inSfIdx < 0)return;

	auto sfIn = inCloud->getScalarField(inSfIdx);
	if (!sfIn)return;

	for (size_t i = 0; i < inCloud->size(); ++i)
	{
		const CCVector3* P = inCloud->getPoint(i);
		double intensity = sfIn->getValue(i);

		if (P->z >= lowerThreshold && P->z <= upperThreshold)
		{
			// 符合条件：添加到输出点云，并同步强度值
			cloud_filtered->addPoint(*P);
			sfOut->addElement(intensity);
		}
	}

	// 重新计算输出标量场的最小/最大，用于后续显示
	sfOut->computeMinAndMax();

	// 设置当前显示标量场并应用默认显示
	cloud_filtered->setCurrentDisplayedScalarField(outSfIdx);
	CloudProcess::apply_default_intensity_and_visible(cloud_filtered);

	// 请求重绘（如果有可视化窗口）
	if (cloud_filtered->getDisplay())
		cloud_filtered->getDisplay()->redraw(false, true);
}


// 简单版：用AABB盒子筛选点
void getPointsInBox(ccPointCloud* cloud, const ccBBox& aabb, std::vector<unsigned>& indices)
{
	if (!cloud)
		return;

	unsigned n = cloud->size();
	indices.clear();
	indices.reserve(n / 10); // 预留一点空间，避免多次扩容

	for (unsigned i = 0; i < n; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		if (P->x >= aabb.minCorner().x && P->x <= aabb.maxCorner().x &&
			P->y >= aabb.minCorner().y && P->y <= aabb.maxCorner().y 
			/*&&P->z >= aabb.minCorner().z && P->z <= aabb.maxCorner().z*/)
		{
			indices.push_back(i);
		}
	}
}


void getPointsCentroidAndAxis(ccPointCloud* ccCloud, std::vector<unsigned>& candIndices,  Eigen::Vector4f& centroid, Eigen::Vector3f& axis)
{
	Eigen::Matrix3f covariance;
	PCLCloudPtr cloud(new PCLCloud);
	for (auto idx : candIndices)
	{
		auto& point = *ccCloud->getPoint(idx);
		cloud->push_back({ point.x, point.y,point.z });
	}

	pcl::computeMeanAndCovarianceMatrix(*cloud, covariance, centroid);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> model_solver(covariance);
	Eigen::Matrix3f eigenvectors = model_solver.eigenvectors();

	axis = eigenvectors.col(2);
}

void getPointsCentroidAndAxis(std::vector<CCVector3>& points, Eigen::Vector4f& centroid, Eigen::Vector3f& axis)
{
	Eigen::Matrix3f covariance;
	PCLCloudPtr cloud(new PCLCloud);
	for (auto point : points)
	{
		cloud->push_back({ point.x, point.y,point.z });
	}

	pcl::computeMeanAndCovarianceMatrix(*cloud, covariance, centroid);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> model_solver(covariance);
	Eigen::Matrix3f eigenvectors = model_solver.eigenvectors();

	axis = eigenvectors.col(2);
}


// p: 三维点；C: 矩形中心；dir: 矩形长边方向；halfL/halfW: 长/宽半轴

/// <summary>
/// 判断点是否在矩形内
/// </summary>
/// <param name="p">判断的点</param>
/// <param name="C">矩形的中心点</param>
/// <param name="dir">矩形长边的方向（要单位向量化！）</param>
/// <param name="halfL">一半的长</param>
/// <param name="halfW">一半的宽</param>
/// <returns></returns>
bool pointInOrientedRect(const CCVector3& p,
	const CCVector3& C,
	const CCVector3& dir,
	double halfL,
	double halfW)
{
	double dx = p.x - C.x;
	double dy = p.y - C.y;

	CCVector3 u = dir;         // 长边单位向量
	CCVector3 v(-u.y, u.x, 0); // 矩形短边方向
	v.normalize();

	// 投影长度
	double a = dx * u.x + dy * u.y; // = d · u
	double b = dx * v.x + dy * v.y; // = d · v

	// 范围判断
	return std::abs(a) <= halfL && std::abs(b) <= halfW;
}


/// <summary>
/// 获得点云中，在矩形内的点
/// （判断每个点是否在矩形内）
/// </summary>
/// <param name="cloud">点云</param>
/// <param name="C">矩形中心</param>
/// <param name="dir">矩形长边方向</param>
/// <param name="halfL">半长</param>
/// <param name="halfW">半宽</param>
/// <param name="indices">输出参数：在矩形内点的索引</param>
void getPointsInBox(ccPointCloud* cloud,
	const CCVector3& C,
	const CCVector3& dir,
	double halfL,
	double halfW,
	std::vector<unsigned>& indices)
{
	if (!cloud)
		return;

	unsigned n = cloud->size();
	indices.clear();
	indices.reserve(n / 10); // 预留一点空间，避免多次扩容

	for (unsigned i = 0; i < n; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		if (pointInOrientedRect(*P, C, dir, halfL, halfW))
		{
			indices.push_back(i);
		}
	}
}

/// <summary>
/// 获得点云中，在矩形内的点
/// （使用八叉树）
/// </summary>
/// <param name="cloud">点云</param>
/// <param name="C">矩形中心</param>
/// <param name="dir">矩形长边方向</param>
/// <param name="L">长</param>
/// <param name="W">宽</param>
/// <param name="indices">输出参数：在矩形内的点</param>
void getPointsInBox(ccPointCloud* cloud,
	const CCVector3& C,
	const CCVector3& dir,
	float L,
	float W,
	std::vector<CCVector3>& points)
{
	if (!cloud)
		return;

	unsigned n = cloud->size();
	points.clear();

	ccOctree::Shared octree = PointCloudIO::get_octree(cloud);
	if (!octree)
	{
		return;
	}

	CCCoreLib::DgmOctree::BoxNeighbourhood boxParams;
	boxParams.center = C;  // 中心点
	boxParams.dimensions = { L , W, /*INFINITY*/ 100};  // 长宽高
	boxParams.axes = new CCVector3[3];
	{
		boxParams.axes[0] = { dir.x, dir.y, 0 };
		boxParams.axes[0].normalize();
		boxParams.axes[1] = { -dir.y, dir.x, 0 };
		boxParams.axes[1].normalize();
		boxParams.axes[2] = {0,0,1};
	}
	boxParams.level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(std::min(L,W));
	octree->getPointsInBoxNeighbourhood(boxParams);

	points.clear();
	for (auto p : boxParams.neighbours)
	{
		points.push_back(*p.point);
	}
}


/// <summary>
/// 从矩形中筛选出等腰三角形内的点
/// 三角形的顶点为 startPt，高为 height，底边长为 baseLen，dir 为高的方向（单位向量）
/// </summary>
void getPointsInIsoscelesTriangleRegion(
	ccPointCloud* cloud,
	const CCVector3& startPt,
	const CCVector3& dir,         // 高的方向（单位向量）
	float height,                 // 高
	float baseLen,                // 底边长度
	std::vector<CCVector3>& trianglePoints)
{
	// Step 1: 算出底边中心点
	CCVector3 centerBase = startPt + dir * height;

	// Step 2: 算出高的中点作为 box 的中心
	CCVector3 boxCenter = (startPt + centerBase) / 2.0f;

	// Step 3: 用 getPointsInBox 得到初步候选点
	std::vector<CCVector3> boxPoints;
	getPointsInBox(cloud, boxCenter, dir, height, baseLen, boxPoints);

	// Step 4: 构造底边方向（垂直于高）
	CCVector3 perp(-dir.y, dir.x, 0);  // 顺时针旋转90度
	perp.normalize();

	// Step 5: 构造三角形三点
	CCVector3 A = startPt;
	CCVector3 B = centerBase + perp * (baseLen / 2.0f);
	CCVector3 C = centerBase - perp * (baseLen / 2.0f);

	trianglePoints.clear();
	// Step 6: 使用重心坐标法判断点是否在三角形 ABC 内
	for (const CCVector3& P : boxPoints)
	{
		CCVector3 v0 = C - A;
		CCVector3 v1 = B - A;
		CCVector3 v2 = P - A;

		float dot00 = v0.dot(v0);
		float dot01 = v0.dot(v1);
		float dot02 = v0.dot(v2);
		float dot11 = v1.dot(v1);
		float dot12 = v1.dot(v2);

		float denom = dot00 * dot11 - dot01 * dot01;
		if (fabs(denom) < 1e-6)
			continue;

		float u = (dot11 * dot02 - dot01 * dot12) / denom;
		float v = (dot00 * dot12 - dot01 * dot02) / denom;

		if (u >= 0 && v >= 0 && (u + v <= 1))
		{
			trianglePoints.push_back(P);
		}
	}
}



PCLCloudPtr CloudProcess::rotate_cloud(PCLCloudPtr pclCloud, const Eigen::Vector3f& now_v, const Eigen::Vector3f& new_v)
{
	// 旋转角度
	double theta = acos(now_v.dot(new_v) / (now_v.norm() * new_v.norm()));

	// 旋转轴：now_v和new_v轴的叉积
	Eigen::Vector3f axis = now_v.cross(new_v);
	axis.normalize();

	// 使用 Eigen 计算旋转矩阵
	Eigen::AngleAxisf rotation(theta, axis);
	Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();
	Eigen::Matrix4f rotationMatrix4 = Eigen::Matrix4f::Identity();
	rotationMatrix4.block<3, 3>(0, 0) = rotationMatrix;

	// Apply rotation to PCL cloud
	PCLCloudPtr cloud(new PCLCloud);
	pcl::transformPointCloud(*pclCloud, *cloud, rotationMatrix4);

	return cloud;
}


ccPointCloud* CloudProcess::rotate_cloud(ccPointCloud* P, const CCVector3& now_v, const CCVector3& new_v)
{
	// 将 ccPointCloud 转换为 PCLCloudPtr
	PCLCloudPtr pclCloud = PointCloudIO::convert_to_PCLCloudPtr(P);

	// 调用第二个版本的 rotate_cloud，传入 PCLCloudPtr 类型的云点数据和旋转向量
	PCLCloudPtr rotatedCloud = rotate_cloud(pclCloud, Eigen::Vector3f(now_v.x, now_v.y, now_v.z), Eigen::Vector3f(new_v.x, new_v.y, new_v.z));

	// 将旋转后的 PCLCloudPtr 转换回 ccPointCloud
	return PointCloudIO::convert_to_ccCloudPtr(rotatedCloud).release();
}

/// <summary>
/// 使用 RANSAC 方法在输入点云上拟合一条直线，提取距离模型内点的阈值范围内的点云。
/// 并可选地计算该直线的单位方向向量及沿该方向的两个最远端点。
/// </summary>
/// <param name="cloud">输入点云的指针，包含待分割的点数据。</param>
/// <param name="distance_threshold">距离阈值，点到拟合直线的最大允许距离。</param>
/// <param name="cloud_filtered">输出点云的指针，提取出的直线内点将存入此点云。</param>
/// <param name="direction">
/// 可选。若不为 nullptr，则赋值为拟合直线的单位方向向量；
/// 传入 nullptr 可跳过方向计算。
/// </param>
/// <param name="endpoint_min">
/// 可选。若不为 nullptr，则赋值为沿方向投影最小的端点坐标；
/// 传入 nullptr 可跳过此端点计算。
/// </param>
/// <param name="endpoint_max">
/// 可选。若不为 nullptr，则赋值为沿方向投影最大的端点坐标；
/// 传入 nullptr 可跳过此端点计算。
/// </param>
void fitLineByRANSAC(
	PCLCloudPtr cloud,
	float distance_threshold,
	PCLCloudPtr cloud_filtered,
	Eigen::Vector3f* direction = nullptr,
	Eigen::Vector3f* endpoint_min = nullptr,
	Eigen::Vector3f* endpoint_max = nullptr)
{
	// 1. 分割直线模型
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setNumberOfThreads(0);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.empty())
		return;

	// 2. 提取直线内点
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	// 如果调用者需要方向或端点，再继续下面的计算
	if (direction || endpoint_min || endpoint_max)
	{
		// 3. 读取直线参数
		Eigen::Vector3f p0(coefficients->values[0],
			coefficients->values[1],
			coefficients->values[2]);
		Eigen::Vector3f dir(coefficients->values[3],
			coefficients->values[4],
			coefficients->values[5]);
		dir.normalize();

		// 4. 在 cloud_filtered 中找投影 t 的最小/最大值
		float t_min = std::numeric_limits<float>::max();
		float t_max = -std::numeric_limits<float>::max();
		for (const auto& pt : cloud_filtered->points) {
			Eigen::Vector3f p(pt.x, pt.y, pt.z);
			float t = (p - p0).dot(dir);
			t_min = std::min(t_min, t);
			t_max = std::max(t_max, t);
		}

		// 5. 根据调用者需求写回结果
		if (direction) {
			*direction = dir;
		}
		if (endpoint_min) {
			*endpoint_min = p0 + t_min * dir;
		}
		if (endpoint_max) {
			*endpoint_max = p0 + t_max * dir;
		}
	}
}

/// <summary>
/// 从固定起点 start_point 沿 dir 大致方向，用"矩形走廊"方法搜索 ±30° 扫描，
/// 选出最大 inliers 对应的精确方向。
/// </summary>
void fitLineByBox(
	PCLCloudPtr                    cloud,
	float                          distance_threshold,
	const Eigen::Vector3f& start_point,
	const Eigen::Vector3f& dir,
	PCLCloudPtr                    cloud_filtered,
	Eigen::Vector3f* direction = nullptr,
	Eigen::Vector3f* endpoint_min = nullptr,
	Eigen::Vector3f* endpoint_max = nullptr)
{
	if (!cloud || !cloud_filtered) return;

	// 1. 归一化中心方向
	Eigen::Vector3f d0 = dir.normalized();
	float half_w = distance_threshold;

	// 2. 准备"上下浮动"范围：±30°
	constexpr float maxAngleRad = 30.0f * static_cast<float>(M_PI) / 180.0f;
	const int   numSteps = 61;  // 扫描粒度：每度一次

	// 3. 确定"摆动轴"：取 d0 与全局"上"（0,0,1）做叉乘，如果平行再换 (1,0,0)
	Eigen::Vector3f up(0, 0, 1);
	if (std::fabs(d0.dot(up)) > 0.99f) up = Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f swingAxis = d0.cross(up).normalized();

	// 4. 对每个候选角度投票
	size_t bestCount = 0;
	float  bestMaxProj = 0.0f;
	float  bestAngle = 0.0f;

	for (int i = 0; i < numSteps; ++i)
	{
		// 线性插值角度
		float angle = -maxAngleRad + (2 * maxAngleRad) * (float(i) / (numSteps - 1));
		Eigen::Vector3f dcand = Eigen::AngleAxisf(angle, swingAxis) * d0;

		// 统计 inliers 并记录最大投影长度
		size_t count = 0;
		float  maxProj = -std::numeric_limits<float>::infinity();
		for (const auto& pt : cloud->points)
		{
			Eigen::Vector3f p(pt.x, pt.y, pt.z);
			Eigen::Vector3f v = p - start_point;
			float proj = v.dot(dcand);
			float perp = (v - dcand * proj).norm();
			if (proj >= 0.0f && perp <= half_w)
			{
				++count;
				if (proj > maxProj) maxProj = proj;
			}
		}

		if (count > bestCount)
		{
			bestCount = count;
			bestMaxProj = maxProj;
			bestAngle = angle;
		}
	}

	// 5. 选出最佳方向，重新填充滤波点云和端点
	Eigen::Vector3f bestDir = Eigen::AngleAxisf(bestAngle, swingAxis) * d0;
	cloud_filtered->clear();
	float max_proj = -std::numeric_limits<float>::infinity();
	for (const auto& pt : cloud->points)
	{
		Eigen::Vector3f p(pt.x, pt.y, pt.z);
		Eigen::Vector3f v = p - start_point;
		float proj = v.dot(bestDir);
		float perp = (v - bestDir * proj).norm();
		if (proj >= 0.0f && perp <= half_w)
		{
			cloud_filtered->push_back(pt);
			if (proj > max_proj) max_proj = proj;
		}
	}

	// 输出生长方向
	if (direction)    *direction = bestDir;
	// 最小端点始终是起点
	if (endpoint_min) *endpoint_min = start_point;
	// 最大端点
	if (endpoint_max)
	{
		if (bestCount > 0)
			*endpoint_max = start_point + bestDir * max_proj;
		else
			*endpoint_max = start_point;
	}
}

/// <summary>
/// 在 RANSAC 提取的 inliers 点云上，
/// 按全局方向 dir 排序后截取末端 tail_ratio 比例的点集，
/// 对这段点集做 PCA，仅计算局部末端的主方向（延伸方向）。
/// </summary>
/// <param name="cloud_filtered">
/// RANSAC 提取的 inliers 点云（已去除大弯曲与噪声）
/// </param>
/// <param name="p0">
/// RANSAC 拟合直线模型上一点，用于计算投影参数
/// </param>
/// <param name="dir">
/// RANSAC 拟合直线的单位方向向量
/// </param>
/// <param name="tail_ratio">
/// 截取末端点云的比例，(0,1]，例如 0.3 表示保留最后 30% 的点
/// </param>
/// <param name="end_dir">
/// 输出参数：计算得到的局部末端延伸方向（单位向量）
/// </param>
/// <summary>
/// 对 RANSAC 提取的 inliers 点云进行局部 PCA，
/// 按全局方向 dir 排序后截取末端 tail_ratio 比例的点集，
/// 计算该段的主方向（延伸方向）。
/// </summary>
/// <param name="cloud_filtered">
///   RANSAC 提取的 inliers 点云
/// </param>
/// <param name="dir">
///   RANSAC 拟合直线的单位方向向量
/// </param>
/// <param name="tail_ratio">
///   截取末端点云的比例 (0,1]
/// </param>
/// <param name="end_dir">
///   输出：局部末端延伸方向（单位向量）
/// </param>
void computeEndDirectionPCA(
	PCLCloudPtr cloud_filtered,
	const Eigen::Vector3f& dir,
	float tail_ratio,
	Eigen::Vector3f& end_dir)
{
	struct TP { float t; Eigen::Vector3f p; };
	std::vector<TP> proj_pts;
	proj_pts.reserve(cloud_filtered->size());

	// 1. 直接用 p·dir 排序
	for (auto& pt : cloud_filtered->points) {
		Eigen::Vector3f p(pt.x, pt.y, pt.z);
		proj_pts.push_back({ p.dot(dir), p });
	}
	std::sort(proj_pts.begin(), proj_pts.end(),
		[](auto& a, auto& b) { return a.t < b.t; });

	// 2. 截取末端 tail_ratio 部分
	size_t N = proj_pts.size();
	size_t start = static_cast<size_t>(N * (1.0f - tail_ratio));
	if (start >= N) start = 0;

	std::vector<Eigen::Vector3f> tail_pts;
	tail_pts.reserve(N - start);
	for (size_t i = start; i < N; ++i)
		tail_pts.push_back(proj_pts[i].p);

	// 3. 点太少时退回全局方向
	if (tail_pts.size() < 2) {
		end_dir = dir;
		return;
	}

	// 4. 计算尾段质心
	Eigen::Vector3f mean = Eigen::Vector3f::Zero();
	for (auto& p : tail_pts) mean += p;
	mean /= tail_pts.size();

	// 5. 协方差矩阵
	Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
	for (auto& p : tail_pts) {
		Eigen::Vector3f d = p - mean;
		C += d * d.transpose();
	}
	C /= tail_pts.size();

	// 6. 特征分解，取主方向
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(C);
	end_dir = es.eigenvectors().col(2).normalized();

	if (dir.dot(end_dir) < 0)
	{
		end_dir = -end_dir;
	}
}

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

std::vector<CCVector3> clusterFromSeed(
	const CCVector3& seed,
	const std::vector<CCVector3>& input_pts,
	float cluster_tolerance = 0.2f,
	int min_cluster_size = 10,
	int max_cluster_size = 10000)
{
	// Step 1: 转换为 PCL 点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (const auto& pt : input_pts)
		cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));

	// Step 2: 构建 KD 树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	// Step 3: 欧几里得聚类
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(cluster_tolerance);
	ec.setMinClusterSize(min_cluster_size);
	ec.setMaxClusterSize(max_cluster_size);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	// Step 4: 找到 seed 所在的点索引（找最接近点）
	int seed_idx = -1;
	float min_dist = 1e9;
	for (int i = 0; i < input_pts.size(); ++i)
	{
		float dist = (input_pts[i] - seed).norm();
		if (dist < min_dist)
		{
			min_dist = dist;
			seed_idx = i;
		}
	}
	if (seed_idx == -1) return {}; // 没有找到种子点

	// Step 5: 返回包含该点的 cluster
	for (const auto& cluster : cluster_indices)
	{
		for (int idx : cluster.indices)
		{
			if (idx == seed_idx)
			{
				std::vector<CCVector3> result;
				for (int i : cluster.indices)
				{
					const auto& p = cloud->points[i];
					result.emplace_back(p.x, p.y, p.z);
				}
				return result;
			}
		}
	}

	return {}; // 没有找到包含 seed 的聚类
}

CCVector3 fitPCADirection(const std::vector<CCVector3>& pts)
{
	if (pts.size() < 3)
		return CCVector3(1, 0, 0); // 默认方向

	Eigen::MatrixXf mat(pts.size(), 3);
	for (int i = 0; i < pts.size(); ++i)
	{
		mat(i, 0) = pts[i].x;
		mat(i, 1) = pts[i].y;
		mat(i, 2) = pts[i].z;
	}

	Eigen::Vector3f mean = mat.colwise().mean();
	Eigen::MatrixXf centered = mat.rowwise() - mean.transpose();
	Eigen::Matrix3f cov = centered.transpose() * centered / float(mat.rows() - 1);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
	Eigen::Vector3f dir = eig.eigenvectors().col(2); // 最大特征值对应的方向（Z列）

	return CCVector3(dir.x(), dir.y(), dir.z());
}


void CloudProcess::grow_line_from_seed(
	ccPointCloud* P,
	const CCVector3& p0,
	const CCVector3& v0,
	ccPointCloud* select_points,
	std::vector<CCVector3>& result,
	ccGLWindowInterface* m_glWindow,
	bool                     isGetGround,
	bool                     doFitLine,       // 新增：是否在每步用 RANSAC 拟合直线
	bool                     useDynamicRect,  // 新增：是否在生长时动态平移矩形
	double                   W,
	double                   L,
	unsigned                 Nmin,
	double                   theta_max,
	unsigned                 Kmax
)
{
	// 1. 输入校验
	if (!P || !select_points) return;

	// 2. 选择地面或原始点云
	ccPointCloud* ground = isGetGround ? PointCloudIO::get_ground_cloud(P) : P;

	// 3. 当前点与方向
	CCVector3 curr_pt = p0;
	CCVector3 curr_dir = v0;
	curr_dir.normalize();

	// 4. 初步估计高程
	if (ground->size()) {
		curr_pt.z = ground->getPoint(0)->z;
		std::vector<CCVector3> tmp;
		getPointsInBox(ground, curr_pt, curr_dir, L, W, tmp);
		if (!tmp.empty()) curr_pt.z = tmp[0].z;
	}

	// 5. 调试可视化
	ccHObject* debug = nullptr;
	if (m_glWindow) {
		debug = new ccHObject("debug");
		m_glWindow->addToOwnDB(debug);
	}

	// 6. 预分配 PCL 容器，循环内重复利用，避免额外构造
	auto raw = std::make_shared<PCLCloud>();
	auto filtered = std::make_shared<PCLCloud>();
	auto cloud = std::make_shared<PCLCloud>();
	auto inliers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	unsigned jumpCount = 0;
	int frameCount = 0;

	// 7. 主循环：不断延伸直线
	while (true) {
		frameCount++;

		// 7.1 寻找最大有效点集
		double len = L;
		CCVector3 next_pt;
		int last_size = 0;
		std::vector<CCVector3> pts;
		bool is_illegality = false;
		bool is_confirm_start = false;
		while (true)
		{
			next_pt = curr_pt + curr_dir * len;

			if(!is_confirm_start)getPointsInBox(ground, (curr_pt + next_pt) * 0.5, curr_dir, len, W, pts);
			else getPointsInIsoscelesTriangleRegion(ground, curr_pt, curr_dir, len, W, pts);


			if (doFitLine)
			{
				raw->clear();
				for (auto& p : pts)
					raw->push_back(pcl::PointXYZ(p.x, p.y, p.z));

				filtered->clear();
				//fitLineByRANSAC(raw, 0.1f, filtered);
				if (!is_confirm_start) //第一次使用fitLineByRANSAC来确定起点
				{
					Eigen::Vector3f eigenDir, epMin, epMax;
					fitLineByRANSAC(raw, 0.2f, filtered, &eigenDir, &epMin, &epMax);
					if (filtered->size() > Nmin)
					{
						if (curr_dir.dot(CCVector3(eigenDir.x(), eigenDir.y(), eigenDir.z())) < 0)
						{
							eigenDir = -eigenDir;
							std::swap(epMax, epMin);
						}

						Eigen::Vector3f diff = epMax - epMin;
						float segmentLength = diff.norm();
						if (segmentLength > 1.0f && useDynamicRect)
						{
							is_confirm_start = true;
							curr_pt = { epMin.x(), epMin.y(), epMin.z() };
							result.push_back(curr_pt);
							float angle = std::acos(std::clamp(
								float(curr_dir.dot(CCVector3(eigenDir.x(), eigenDir.y(), eigenDir.z()))), -1.0f, 1.0f));
							if (angle > theta_max)
							{
								is_illegality = true;
								break;
							}
							curr_dir = { eigenDir.x(), eigenDir.y(), eigenDir.z() };
						}
					}
				}
				else
				{
					fitLineByRANSAC(raw, 0.2f, filtered);
				}
				pts.clear();
				pts.reserve(filtered->size());
				for (auto& q : *filtered)
					pts.emplace_back(q.x, q.y, q.z);
			}
			if (is_illegality)break;


			if (pts.size() > Nmin)
			{
				// 1. 计算点云质心和矩形中心
				CCVector3 pts_centroid(0, 0, 0);
				for (auto& bp : pts) pts_centroid += bp;
				pts_centroid /= pts.size();
				CCVector3 rect_center = (curr_pt + next_pt) * 0.5;

				// 2. 计算总偏移向量
				CCVector3 offset = pts_centroid - rect_center;

				// 3. 只保留与 curr_dir 垂直的分量：perp_offset = offset - (offset·dir_n)·dir_n
				CCVector3 dir_n = curr_dir;
				dir_n.normalize();
				float proj_len = offset.dot(dir_n);              // 平行分量长度
				CCVector3 parallel = dir_n * proj_len;            // 平行分量
				CCVector3 perp_offset = offset - parallel;        // 垂直分量

				// 4. 仅沿垂直方向平移框
				curr_pt += perp_offset;
				next_pt += perp_offset;
			}

			if (useDynamicRect && pts.size() >= last_size + Nmin)
			{
				last_size = pts.size();
				len += L;
			}
			else
			{
				break;
			}
		}

		// 7.2 点数不足：跳跃或结束
		if (pts.size() < Nmin) {
			if (++jumpCount > Kmax) break;

			if (debug) {
				CCVector3 dv(-curr_dir.y, curr_dir.x, 0);
				dv.normalize();
				ccPointCloud* dbgCloud = new ccPointCloud;
				dbgCloud->addPoint(curr_pt + dv * W * 0.5);
				dbgCloud->addPoint(curr_pt - dv * W * 0.5);
				dbgCloud->addPoint(next_pt - dv * W * 0.5);
				dbgCloud->addPoint(next_pt + dv * W * 0.5);
				ccPolyline* poly = new ccPolyline(dbgCloud);
				for (int i = 0; i < 4; ++i) poly->addPointIndex(i);
				poly->addPointIndex(0);
				poly->setName(QString("jumpCount %1 pts:%2").arg(jumpCount).arg(pts.size()));
				poly->setColor(ccColor::blue);
				poly->showColors(true);
				debug->addChild(poly);
			}
			curr_pt = next_pt;
			continue;
		}

		// 7.3 全局 RANSAC 拟合直线并提取 inliers
		cloud->clear();
		for (auto& p : pts)
			cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));

		inliers->clear();
		Eigen::Vector3f eigenDir, epMin, epMax;
		//fitLineByRANSAC(cloud, 0.1f, inliers, &eigenDir, &epMin, &epMax);
		fitLineByBox(cloud, 0.1f, { curr_pt.x, curr_pt.y, curr_pt.z }, { curr_dir.x, curr_dir.y, curr_dir.z }, inliers, &eigenDir, &epMin, &epMax);

		// 7.4 方向一致性修正
		if (curr_dir.dot(CCVector3(eigenDir.x(), eigenDir.y(), eigenDir.z())) < 0)
		{
			eigenDir = -eigenDir;
			std::swap(epMax, epMin);
		}

		// 7.5 局部 PCA 计算末端方向
		Eigen::Vector3f localDir;
		Eigen::Vector3f diff = epMax - epMin;
		float segmentLength = diff.norm();
		float tailRatio = (segmentLength > 1.0f)
			? 1.0f / segmentLength
			: 1.0f;
		if (inliers->size() < 10) tailRatio = 1.0f;

		computeEndDirectionPCA(inliers, eigenDir, tailRatio, localDir);

		// 7.6 角度约束
		float angle = std::acos(std::clamp(
			float(curr_dir.dot(CCVector3(localDir.x(), localDir.y(), localDir.z()))), -1.0f, 1.0f));
		if (angle > theta_max) {
			break;
			//if (++jumpCount > Kmax) break;
			//curr_pt = CCVector3(epMax.x(), epMax.y(), epMax.z());
			//continue;
		}

		jumpCount = 0;

		// debug可视化
		if (debug) {
			CCVector3 dv(-curr_dir.y, curr_dir.x, 0);
			dv.normalize();
			ccPointCloud* dbgCloud = new ccPointCloud;
			dbgCloud->addPoint(curr_pt + dv * W * 0.5);
			dbgCloud->addPoint(curr_pt - dv * W * 0.5);
			dbgCloud->addPoint(next_pt - dv * W * 0.5);
			dbgCloud->addPoint(next_pt + dv * W * 0.5);
			ccPolyline* poly = new ccPolyline(dbgCloud);
			for (int i = 0; i < 4; ++i) poly->addPointIndex(i);
			poly->addPointIndex(0);
			poly->setName(QString("frame %1 pts:%2").arg(frameCount).arg(pts.size()));

			ccPointCloud* dbgSelectCloud = new ccPointCloud;
			for (auto& p : pts)
				dbgSelectCloud->addPoint(p);
			dbgSelectCloud->setName(QString("frame %1 select cloud").arg(frameCount));
			debug->addChild(poly);
			debug->addChild(dbgSelectCloud);

			// 标注末端方向
			{
				// arrowLen 为可视化长度，可按需要调节
				const float arrowLen = static_cast<float>(L);
				CCVector3 dirCC(localDir.x(), localDir.y(), localDir.z());
				dirCC.normalize();
				CCVector3 arrowStart = { epMax.x(), epMax.y(), epMax.z() };
				ccPointCloud* arrowCloud = new ccPointCloud;
				arrowCloud->addPoint(arrowStart);
				arrowCloud->addPoint(arrowStart + dirCC * arrowLen);
				ccPolyline* arrowLine = new ccPolyline(arrowCloud);
				arrowLine->addPointIndex(0);
				arrowLine->addPointIndex(1);
				arrowLine->setName(QString("Dir frame %1").arg(frameCount));
				arrowLine->setColor(ccColor::redRGB);
				arrowLine->showColors(true);
				debug->addChild(arrowLine);
			}

			// 标注最小端点 epMin
			{
				CCVector3 pMin(epMin.x(), epMin.y(), epMin.z());
				ccPointCloud* epMinCloud = new ccPointCloud;
				epMinCloud->addPoint(pMin);
				epMinCloud->setName(QString("epMin frame %1").arg(frameCount));
				// 设置点的显示大小和颜色（假设 ccPointCloud 支持 setPointSize 和 setTempColor）
				epMinCloud->setPointSize(5);
				epMinCloud->setTempColor(ccColor::red);
				debug->addChild(epMinCloud);
			}

			// 标注最大端点 epMax
			{
				CCVector3 pMax(epMax.x(), epMax.y(), epMax.z());
				ccPointCloud* epMaxCloud = new ccPointCloud;
				epMaxCloud->addPoint(pMax);
				epMaxCloud->setName(QString("epMax frame %1").arg(frameCount));
				epMaxCloud->setPointSize(5);
				epMaxCloud->setTempColor(ccColor::green);
				debug->addChild(epMaxCloud);
			}
		}

		// 7.7 更新当前点和方向
		curr_pt = CCVector3(epMax.x(), epMax.y(), epMax.z());
		curr_dir = CCVector3(
			localDir.x(), localDir.y(), localDir.z()
		);
		curr_dir.normalize();


		if (result.size() >= 2)
		{
			const CCVector3& prev2 = result[result.size() - 2]; // Pₙ₋₂
			const CCVector3& prev1 = result[result.size() - 1]; // Pₙ₋₁

			CCVector3 dir_prev = (prev1 - prev2);
			dir_prev.normalize();
			CCVector3 dir_curr = (curr_pt - prev1);
			dir_curr.normalize();

			float angle = std::acos(std::clamp(float(dir_prev.dot(dir_curr)), -1.0f, 1.0f));

			if (angle > theta_max)
			{
				// 角度变化太大，不保存 curr_pt，终止或跳过
				break;  // 或 continue;
			}
		}
		// 7.8 保存结果
		result.push_back(curr_pt);
		for (auto& p : pts)
			select_points->addPoint(p);
	}
}

/*void CloudProcess::grow_line_from_seed(
	ccPointCloud* P,
	const CCVector3& p0,
	const CCVector3& v0,
	ccPointCloud* select_points,
	std::vector<CCVector3>& result,
	ccGLWindowInterface* m_glWindow,
	bool                     isGetGround,
	bool                     doFitLine,       // 新增：是否在每步用 RANSAC 拟合直线
	bool                     useDynamicRect,  // 新增：是否在生长时动态平移矩形
	double                   W,
	double                   L,
	unsigned                 Nmin,
	double                   theta_max,
	unsigned                 Kmax
)
{
	if (!P || !select_points) return;
	ccPointCloud* ground = isGetGround ? PointCloudIO::get_ground_cloud(P) : P;


	CCVector3 curr_pt = p0;
	CCVector3 curr_dir = v0;
	curr_dir.normalize();

	if (ground->size())
	{
		curr_pt.z = ground->getPoint(0)->z;
		std::vector<CCVector3> tmp;
		getPointsInBox(ground, curr_pt, curr_dir, W, W, tmp);
		if (!tmp.empty()) curr_pt.z = tmp[0].z;
	}
	result.push_back(curr_pt);

	ccHObject* debug = nullptr;
	if (m_glWindow) {
		debug = new ccHObject("debug");
		m_glWindow->addToOwnDB(debug);
	}

	auto raw = std::make_shared<PCLCloud>();
	auto filtered = std::make_shared<PCLCloud>();
	auto cloud = std::make_shared<PCLCloud>();
	auto inliers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	unsigned jumpCount = 0;
	int frameCount = 0;

	
	auto seekLine = [&](std::vector<CCVector3>& pts, CCVector3& curr_pt, CCVector3& curr_dir)
	{
		const float probeHeight = L;
		const float probeBase = W;
		const std::vector<float> angleList = { -15, -10, -5, 0, 5, 10, 15 }; // 角度尝试范围（单位：度）
		float best_score = -1;
		CCVector3 best_dir;
		std::vector<CCVector3> best_pts;
		CCVector3 next_pt;

		for (float angle_deg : angleList)
		{
			// Step 1: 旋转当前方向
			float angle_rad = angle_deg * M_PI / 180.0f;
			CCVector3 rot_dir(
				curr_dir.x * std::cos(angle_rad) - curr_dir.y * std::sin(angle_rad),
				curr_dir.x * std::sin(angle_rad) + curr_dir.y * std::cos(angle_rad),
				0);
			rot_dir.normalize();

			// Step 2: 提取当前等腰三角区域内点
			std::vector<CCVector3> tri_pts;
			getPointsInIsoscelesTriangleRegion(ground, curr_pt, rot_dir, probeHeight, probeBase, tri_pts);
			if (tri_pts.size() < 5) continue;

			// Step 3: 区域生长
			std::vector<CCVector3> region_pts = clusterFromSeed(curr_pt, tri_pts);
			PCLCloudPtr pcl_pts = 
			for (auto p : region_pts)
			{

			}
			if (region_pts.size() < 5) continue;

			// Step 4: 使用 RANSAC 拟合直线
			Eigen::Vector3f dir;
			Eigen::Vector3f epMin, epMax;
			fitLineByRANSAC(region_pts, 0.1f, nullptr, &dir, &epMin, &epMax);

			// 转换为 CCVector3
			CCVector3 line_dir(dir[0], dir[1], dir[2]);
			line_dir.normalize();
			CCVector3 end_min(epMin[0], epMin[1], epMin[2]);
			CCVector3 end_max(epMax[0], epMax[1], epMax[2]);

			// Step 5: 确认 line 起点是否靠近 curr_pt（方向选择一致）
			CCVector3 to_min = end_min - curr_pt;
			CCVector3 to_max = end_max - curr_pt;
			float dist_min = to_min.norm();
			float dist_max = to_max.norm();

			CCVector3 chosen_dir;
			CCVector3 chosen_next_pt;

			if (to_max.dot(rot_dir) > to_min.dot(rot_dir)) // max 更符合方向
			{
				chosen_dir = (end_max - end_min).normalized();
				chosen_next_pt = end_max;
			}
			else
			{
				chosen_dir = (end_min - end_max).normalized();
				chosen_next_pt = end_min;
			}

			float angle_cos = chosen_dir.dot(rot_dir);
			float score = angle_cos * region_pts.size(); // 相似方向 + 点数作为评分

			if (score > best_score)
			{
				best_score = score;
				best_dir = chosen_dir;
				best_pts = region_pts;
				next_pt = chosen_next_pt;
			}
		}

		// Step 6: 更新方向和当前位置
		if (best_score > 0)
		{
			curr_dir = best_dir;
			curr_pt = next_pt;
			pts = best_pts;
		}
		else
		{
			pts.clear(); // 无有效结果
		}
	};

	while (true) {
		frameCount++;
		double len = L;
		CCVector3 next_pt;
		int last_size = 0;
		std::vector<CCVector3> total_pts;
		std::vector<CCVector3> pts;
		CCVector3 curr_start = p0;
		CCVector3 curr_end = p0;
		while (true)
		{
			next_pt = curr_pt + curr_dir * len;
			
			if (len == L)getPointsInBox(ground, (curr_start + next_pt) * 0.5, curr_dir, len, W, pts);
			else getPointsInIsoscelesTriangleRegion(ground, curr_start, curr_dir, len, W, pts);


			if (doFitLine)
			{
				raw->clear();
				for(auto& p: total_pts)
					raw->push_back(pcl::PointXYZ(p.x, p.y, p.z));
				for (auto& p : pts)
					raw->push_back(pcl::PointXYZ(p.x, p.y, p.z));

				filtered->clear();
				Eigen::Vector3f eigenDir, epMin, epMax;
				fitLineByRANSAC(raw, 0.1f, filtered, &eigenDir, &epMin, &epMax);
				if (filtered->size() > Nmin)
				{
					if (curr_dir.dot(CCVector3(eigenDir.x(), eigenDir.y(), eigenDir.z())) < 0)
					{
						eigenDir = -eigenDir;
						std::swap(epMax, epMin);
					}
					curr_pt = { epMin.x(), epMin.y(), epMin.z() };
					curr_dir = { eigenDir.x(), eigenDir.y(), eigenDir.z() };
					curr_end = { epMax.x(), epMax.y(), epMax.z() };
				}

				total_pts.clear();
				total_pts.reserve(filtered->size());
				for (auto& q : *filtered)
					total_pts.emplace_back(q.x, q.y, q.z);
			}

			if (useDynamicRect && pts.size() >= Nmin)
			{
				last_size = pts.size();
				curr_start = curr_pt + curr_dir * len;
				len += L;
			}
			else
			{
				break;
			}
		}

		// 7.2 点数不足：跳跃或结束
		if (pts.size() < Nmin) {
			if (++jumpCount > Kmax) break;
			
			if (debug) {
				CCVector3 dv(-curr_dir.y, curr_dir.x, 0);
				dv.normalize();
				ccPointCloud* dbgCloud = new ccPointCloud;
				dbgCloud->addPoint(curr_pt + dv * W * 0.5);
				dbgCloud->addPoint(curr_pt - dv * W * 0.5);
				dbgCloud->addPoint(next_pt - dv * W * 0.5);
				dbgCloud->addPoint(next_pt + dv * W * 0.5);
				ccPolyline* poly = new ccPolyline(dbgCloud);
				for (int i = 0; i < 4; ++i) poly->addPointIndex(i);
				poly->addPointIndex(0);
				poly->setName(QString("jumpCount %1 pts:%2").arg(jumpCount).arg(pts.size()));
				poly->setColor(ccColor::blue);
				poly->showColors(true);
				debug->addChild(poly);
			}
			curr_pt = next_pt;
			continue;
		}

		// 7.3 全局 RANSAC 拟合直线并提取 inliers
		cloud->clear();
		for (auto& p : pts)
			cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));

		inliers->clear();
		Eigen::Vector3f eigenDir, epMin, epMax;
		//fitLineByRANSAC(cloud, 0.1f, inliers, &eigenDir, &epMin, &epMax);
		fitLineByBox(cloud, 0.1f, { curr_pt.x, curr_pt.y, curr_pt.z }, {curr_dir.x, curr_dir.y, curr_dir.z}, inliers, &eigenDir, &epMin, &epMax);

		// 7.4 方向一致性修正
		if (curr_dir.dot(CCVector3(eigenDir.x(), eigenDir.y(), eigenDir.z())) < 0)
		{
			eigenDir = -eigenDir;
			std::swap(epMax, epMin);
		}

		// 7.5 局部 PCA 计算末端方向
		Eigen::Vector3f localDir;
		Eigen::Vector3f diff = epMax - epMin;
		float segmentLength = diff.norm();
		float tailRatio = (segmentLength > 0.7f)
			? 0.7f/segmentLength
			: 1.0f;
		if (inliers->size() < 10) tailRatio = 1.0f;

		computeEndDirectionPCA(inliers, eigenDir, tailRatio, localDir);

		// 7.6 角度约束
		float angle = std::acos(std::clamp(
			float(curr_dir.dot(CCVector3(localDir.x(), localDir.y(), localDir.z()))), -1.0f, 1.0f));
		if (angle > theta_max) {
			break;
			//if (++jumpCount > Kmax) break;
			//curr_pt = CCVector3(epMax.x(), epMax.y(), epMax.z());
			//continue;
		}

		jumpCount = 0;

		// debug可视化
		if (debug) {
			CCVector3 dv(-curr_dir.y, curr_dir.x, 0);
			dv.normalize();
			ccPointCloud* dbgCloud = new ccPointCloud;
			dbgCloud->addPoint(curr_pt + dv * W * 0.5);
			dbgCloud->addPoint(curr_pt - dv * W * 0.5);
			dbgCloud->addPoint(next_pt - dv * W * 0.5);
			dbgCloud->addPoint(next_pt + dv * W * 0.5);
			ccPolyline* poly = new ccPolyline(dbgCloud);
			for (int i = 0; i < 4; ++i) poly->addPointIndex(i);
			poly->addPointIndex(0);
			poly->setName(QString("frame %1 pts:%2").arg(frameCount).arg(pts.size()));

			ccPointCloud* dbgSelectCloud = new ccPointCloud;
			for (auto& p : pts)
				dbgSelectCloud->addPoint(p);
			dbgSelectCloud->setName(QString("frame %1 select cloud").arg(frameCount));
			debug->addChild(poly);
			debug->addChild(dbgSelectCloud);

			// 标注末端方向
			{
				// arrowLen 为可视化长度，可按需要调节
				const float arrowLen = static_cast<float>(L);
				CCVector3 dirCC(localDir.x(), localDir.y(), localDir.z());
				dirCC.normalize();
				CCVector3 arrowStart= {epMax.x(), epMax.y(), epMax.z()};
				ccPointCloud* arrowCloud = new ccPointCloud;
				arrowCloud->addPoint(arrowStart);
				arrowCloud->addPoint(arrowStart+ dirCC * arrowLen);
				ccPolyline* arrowLine = new ccPolyline(arrowCloud);
				arrowLine->addPointIndex(0);
				arrowLine->addPointIndex(1);
				arrowLine->setName(QString("Dir frame %1").arg(frameCount));
				arrowLine->setColor(ccColor::redRGB);
				arrowLine->showColors(true);
				debug->addChild(arrowLine);
			}

			// 标注最小端点 epMin
			{
				CCVector3 pMin(epMin.x(), epMin.y(), epMin.z());
				ccPointCloud* epMinCloud = new ccPointCloud;
				epMinCloud->addPoint(pMin);
				epMinCloud->setName(QString("epMin frame %1").arg(frameCount));
				// 设置点的显示大小和颜色（假设 ccPointCloud 支持 setPointSize 和 setTempColor）
				epMinCloud->setPointSize(5);
				epMinCloud->setTempColor(ccColor::red);
				debug->addChild(epMinCloud);
			}

			// 标注最大端点 epMax
			{
				CCVector3 pMax(epMax.x(), epMax.y(), epMax.z());
				ccPointCloud* epMaxCloud = new ccPointCloud;
				epMaxCloud->addPoint(pMax);
				epMaxCloud->setName(QString("epMax frame %1").arg(frameCount));
				epMaxCloud->setPointSize(5);
				epMaxCloud->setTempColor(ccColor::green);
				debug->addChild(epMaxCloud);
			}
		}

		// 7.7 更新当前点和方向
		curr_pt = CCVector3(epMax.x(), epMax.y(), epMax.z());
		curr_dir = CCVector3(
			localDir.x(), localDir.y(), localDir.z()
		);
		curr_dir.normalize();

		// 7.8 保存结果
		result.push_back(curr_pt);
		for (auto& p : pts)
			select_points->addPoint(p);
	}
}
*/

// 生成一维模板
std::vector<int> generate_stripe_template(double stripeLength, double gapLength, int stripeCount,
	double binSize, int totalBins)
{
	std::vector<int> templateMask(totalBins, 0);
	int stripeBins = static_cast<int>(stripeLength / binSize);
	int gapBins = static_cast<int>(gapLength / binSize);
	int offset = (totalBins - (stripeBins * stripeCount + gapBins * (stripeCount - 1))) / 2;

	int cursor = std::max(offset, 0);
	for (int i = 0; i < stripeCount && cursor + stripeBins <= totalBins; ++i)
	{
		for (int j = 0; j < stripeBins; ++j)
			templateMask[cursor + j] = 1;
		cursor += stripeBins + gapBins;
	}
	return templateMask;
}

// 匹配 bins 数据（假设是投影后的 binId => 点数）
int match_stripe_template(const std::map<int, std::vector<int>>& bins, const std::vector<int>& templateMask, int& bestOffset)
{
	// step 1: 创建直方图
	if (bins.empty()) return 0;

	int minBin = bins.begin()->first;
	int maxBin = bins.rbegin()->first;
	int totalBins = maxBin - minBin + 1;

	std::vector<int> histogram(totalBins, 0);
	for (const auto& [binId, indices] : bins)
	{
		histogram[binId - minBin] = static_cast<int>(indices.size());
	}

	// step 2: 滑动模板匹配
	int maxScore = -1;
	bestOffset = 0;
	int tmplLen = static_cast<int>(templateMask.size());

	for (int offset = 0; offset <= totalBins - tmplLen; ++offset)
	{
		int score = 0;
		for (int j = 0; j < tmplLen; ++j)
		{
			if (templateMask[j] == 1)
				score += histogram[offset + j];
		}
		if (score > maxScore)
		{
			maxScore = score;
			bestOffset = offset;
		}
	}

	return maxScore;
}

// 自动寻找最佳模板参数
void auto_match_best_stripe(const std::map<int, std::vector<int>>& bins, double binSize,
	double stripeLen, double gapLen, int minStripes, int maxStripes)
{
	if (bins.empty()) return;

	int minBin = bins.begin()->first;
	int maxBin = bins.rbegin()->first;
	int totalBins = maxBin - minBin + 1;

	int bestGlobalScore = -1;
	int bestOffset = 0;
	int bestStripeCount = 0;

	for (int stripeCount = minStripes; stripeCount <= maxStripes; ++stripeCount)
	{
		auto mask = generate_stripe_template(stripeLen, gapLen, stripeCount, binSize, totalBins);
		int offset = 0;
		int score = match_stripe_template(bins, mask, offset);

		if (score > bestGlobalScore)
		{
			bestGlobalScore = score;
			bestOffset = offset;
			bestStripeCount = stripeCount;
		}
	}

	std::cout << "Best Stripe Count: " << bestStripeCount << "\n";
	std::cout << "Best Offset: " << bestOffset << "\n";
	std::cout << "Best Score: " << bestGlobalScore << "\n";
}

// 制作斑马线模板（avgWidth是斑马线的宽，medianStripeLength，medianGapLength分别是白条，间隔的宽）
// 制作5~20个模板（分别这么多白条数）
// 0,0是起点，轮廓的点云是0.1一个点，把白条围起来
// raw点云（用来匹配重叠率，每0.1*0.1一个点，把白条铺满）
// 用折线把白条围起来

//class Model

//std::vector<std::vector<PCLPoint>> vectorized_polylines;  // 向量化点的列表
//std::string name;
//PCLCloudPtr outline_point_cloud;
//PCLCloudPtr raw_point_cloud;

// 辅助函数：在两点之间生成并填充点
void fillEdgeWithPoints(const Eigen::Vector3f& start, const Eigen::Vector3f& end, PCLCloudPtr& cloud)
{
	// 计算边的长度
	float length = (end - start).norm();

	// 计算需要的点数
	int numPoints = static_cast<int>(length / 0.1f);  // 以0.1为间隔生成点

	// 计算边的方向向量
	Eigen::Vector3f direction = (end - start).normalized();

	// 在边上按间隔填充点
	for (int i = 0; i <= numPoints; ++i)
	{
		Eigen::Vector3f point = start + direction * (i * 0.1f);
		cloud->push_back(PCLPoint(point.x(), point.y(), point.z()));
	}
}

void createModelPointCloud(roadmarking::Model& model, float avgWidth, float medianStripeLength, float medianGapLength, int numStripes)
{
	// 清空现有点云
	model.outline_point_cloud->clear();
	model.raw_point_cloud->clear();

	// 从原点(0, 0)开始生成斑马线模板
	float currentX = 0.0f;  // X轴起点
	for (int i = 0; i < numStripes; ++i)
	{
		// 每条白条的长度和间隔
		float stripeLength = medianStripeLength;// + (rand() % 2 ? 1 : -1) * 0.1f;  // 随机略微变化(多个可能有效果，代价大了)
		float gapLength = medianGapLength;// + (rand() % 2 ? 1 : -1) * 0.1f;      // 随机略微变化

		// 白条的坐标：通过X轴方向和宽度生成矩形
		// 包含周围0.1的间隔
		Eigen::Vector3f topLeft(currentX - 0.1f, avgWidth / 2.0f + 0.1f, 0.0f);   // 左上点（加上0.1的间隔）
		Eigen::Vector3f topRight(currentX + stripeLength + 0.1f, avgWidth / 2.0f + 0.1f, 0.0f);  // 右上点
		Eigen::Vector3f bottomRight(currentX + stripeLength + 0.1f, -avgWidth / 2.0f - 0.1f, 0.0f); // 右下点
		Eigen::Vector3f bottomLeft(currentX - 0.1f, -avgWidth / 2.0f - 0.1f, 0.0f);  // 左下点

		// 填充每条边上的点
		fillEdgeWithPoints(topLeft, topRight, model.outline_point_cloud);
		fillEdgeWithPoints(topRight, bottomRight, model.outline_point_cloud);
		fillEdgeWithPoints(bottomRight, bottomLeft, model.outline_point_cloud);
		fillEdgeWithPoints(bottomLeft, topLeft, model.outline_point_cloud);

		// 生成原始点云（每个0.1*0.1小点填充）
		for (float x = topLeft.x(); x < topRight.x(); x += 0.1f) {
			for (float y = bottomLeft.y(); y < topLeft.y(); y += 0.1f) {
				model.raw_point_cloud->push_back(PCLPoint(x, y, 0.0f));  // 添加点到原始点云
			}
		}

		// 创建折线包围每个白条
		std::vector<PCLPoint> stripePolyline = {
			PCLPoint(topLeft.x(), topLeft.y(), topLeft.z()),        // 左上点
			PCLPoint(topRight.x(), topRight.y(), topRight.z()),    // 右上点
			PCLPoint(bottomRight.x(), bottomRight.y(), bottomRight.z()),  // 右下点
			PCLPoint(bottomLeft.x(), bottomLeft.y(), bottomLeft.z()),    // 左下点
			PCLPoint(topLeft.x(), topLeft.y(), topLeft.z())        // 闭合折线
		};

		model.vectorized_polylines.push_back(stripePolyline);

		// 更新下一个白条的起始位置
		currentX += stripeLength + gapLength;
	}
}

void CloudProcess::extract_zebra_by_struct(ccPointCloud* inputCloud, ccGLWindowInterface* m_glWindow)
{
	if (!inputCloud)
		return;

	const float binWidth = 0.1f;

	ccHObject* debug = nullptr;
	if (m_glWindow)
	{
		debug = new ccHObject("zebra_debug");
		m_glWindow->addToOwnDB(debug);
	}

	// 1.计算主轴并把点映射到主轴上
	std::vector<unsigned> allIndices(inputCloud->size());
	std::iota(allIndices.begin(), allIndices.end(), 0);
	Eigen::Vector4f centroid;
	Eigen::Vector3f axis;
	getPointsCentroidAndAxis(inputCloud, allIndices, centroid, axis);

	std::map<int, std::vector<int>> bins;
	std::map<int, float> binPosMap;
	float min_proj = std::numeric_limits<float>::max();
	float max_proj = std::numeric_limits<float>::min();

	for (unsigned i = 0; i < inputCloud->size(); ++i)
	{
		Eigen::Vector3f p(inputCloud->getPoint(i)->x, inputCloud->getPoint(i)->y, inputCloud->getPoint(i)->z);
		float proj = (p - Eigen::Vector3f(centroid[0], centroid[1], centroid[2])).dot(axis);
		min_proj = std::min(min_proj, proj);
		max_proj = std::max(max_proj, proj);
		int binId = static_cast<int>(proj / binWidth);
		bins[binId].push_back(i);
		binPosMap[binId] = proj;
	}

	// 2.滑动窗口计算阈值， 3.计算线区域
	std::vector<std::vector<int>> stripes;
	double medianStripeLength = 0;
	double medianGapLength = 0;
	{
		int windowSize = 17;
		std::vector<int> currentStripe;
		std::vector<int> sortedBinIds;
		for (const auto& [binId, _] : bins)
			sortedBinIds.push_back(binId);
		std::sort(sortedBinIds.begin(), sortedBinIds.end());

		for (size_t i = 0; i < sortedBinIds.size(); ++i)
		{
			int binId = sortedBinIds[i];

			// 计算滑动窗口内的平均密度
			int count = 0;
			int sum = 0;
			for (int offset = -windowSize / 2; offset <= windowSize / 2; ++offset)
			{
				int neighborId = binId + offset;
				auto it = bins.find(neighborId);
				if (it != bins.end())
				{
					sum += static_cast<int>(it->second.size());
				}
				if (neighborId >= sortedBinIds.front() && neighborId < sortedBinIds.back())
				{
					count++;
				}
			}
			double avgDensity = (count > 0) ? (double)sum / count : 0.0;

			// 判断是否是 stripe 区域
			if (bins[binId].size() > avgDensity)
			{
				if (currentStripe.empty() || binId == currentStripe.back() + 1)
				{
					currentStripe.push_back(binId);
				}
				else
				{
					if (!currentStripe.empty())
						stripes.push_back(currentStripe);
					currentStripe = { binId };
				}
			}
			else
			{
				if (!currentStripe.empty())
				{
					stripes.push_back(currentStripe);
					currentStripe.clear();
				}
			}
		}
		if (!currentStripe.empty())
			stripes.push_back(currentStripe);


		// ======================== 计算stripe、gap宽度中位数
		std::vector<int> stripeLengths;
		std::vector<int> gapLengths;
		for (int i = 0; i < stripes.size(); i++)
		{
			stripeLengths.push_back(stripes[i].back() - stripes[i].front());
			if(i)
			{
				gapLengths.push_back(stripes[i].front() - stripes[i - 1].back());
			}
		}

		auto median = [](std::vector<int>& vec) -> int {
			if (vec.empty()) return 0;
			std::sort(vec.begin(), vec.end());
			size_t mid = vec.size() / 2;
			if (vec.size() % 2 == 0)
				return (vec[mid - 1] + vec[mid]) / 2;
			else
				return vec[mid];
		};

		medianStripeLength = median(stripeLengths)* binWidth;
		medianGapLength = median(gapLengths)* binWidth;
	}

	double avgWidth = 0.0f;
	{
		float totalLength = 0.0f;
		float totalWidth = 0.0f;
		size_t validStripes = 0;

		// 遍历每个stripe
		for (size_t i = 0; i < stripes.size(); ++i)
		{
			std::vector<Eigen::Vector3f> stripePoints;

			// 收集当前stripe的所有点
			for (int binId : stripes[i])
			{
				for (int ptIdx : bins[binId])
				{
					const CCVector3* pt = inputCloud->getPoint(ptIdx);
					stripePoints.emplace_back(pt->x, pt->y, pt->z);
				}
			}

			if (stripePoints.size() < 10)
				continue; // 点数太少，跳过

			// 使用已知主轴 axis 和垂直方向构造斑马线
			Eigen::Vector3f stripeDir = Eigen::Vector3f(-axis.y(), axis.x(), 0);  // 条纹方向，垂直于axis
			Eigen::Vector3f stripePerp = axis;  // 沿着主轴的方向用于计算宽度

			// 投影到 stripeDir，计算长度
			std::vector<float> projAlong;
			for (const auto& pt : stripePoints)
			{
				Eigen::Vector3f d = pt;
				projAlong.push_back(d.dot(stripeDir));
			}

			auto minmaxAlong = std::minmax_element(projAlong.begin(), projAlong.end());
			float stripeLength = (*minmaxAlong.second - *minmaxAlong.first);

			// 累加长度和宽度
			totalLength += stripeLength;
			++validStripes;
		}

		// 计算平均长宽
		if (validStripes > 0) {
			avgWidth = totalLength / validStripes;
		}
	}


	{
		// 生成模板
		std::vector<Model>models;
		int estimation_num = (max_proj - min_proj)/(medianStripeLength + medianGapLength);

		for (int i = estimation_num -1; i <= estimation_num + 1; i++)
		{
			models.push_back({});
			createModelPointCloud(models.back(), avgWidth, medianStripeLength,medianGapLength, i);
		}

		// 匹配
		PCLCloudPtr points(new PCLCloud);
		// 遍历每个stripe
		for (size_t i = 0; i < stripes.size(); ++i)
		{
			// 收集当前stripe的所有点
			for (int binId : stripes[i])
			{
				for (int ptIdx : bins[binId])
				{
					const CCVector3* pt = inputCloud->getPoint(ptIdx);
					points->push_back({ pt->x, pt->y, pt->z });
				}
			}
		}
		const std::vector<PCLCloudPtr>clouds{points};
		RoadMarkings roadMarkings;
		RoadMarkingClassifier classifier;
		classifier.ClassifyRoadMarkings(clouds, roadMarkings, models);


		ccHObject* allLinesContainer = visualizeRoadMarkings(roadMarkings);

		debug->addChild(allLinesContainer);
	}

	
	
	// =================================================================== debug
	if (debug)
	{
		float max_dis = 0, min_dis = 0;
		float _max_dis = 0, _min_dis = 0;
		for (unsigned i = 0; i < inputCloud->size(); ++i)
		{
			Eigen::Vector3f p(inputCloud->getPoint(i)->x, inputCloud->getPoint(i)->y, inputCloud->getPoint(i)->z);
			float proj = (p - Eigen::Vector3f(centroid[0], centroid[1], centroid[2])).dot(axis);
			max_dis = std::max(proj, max_dis);
			min_dis = std::min(proj, min_dis);
			float _proj = (p - Eigen::Vector3f(centroid[0], centroid[1], centroid[2])).dot(Eigen::Vector3f{ -axis.y() , axis.x(), 0 });
			_max_dis = std::max(_proj, _max_dis);
			_min_dis = std::min(_proj, _min_dis);
		}
		ccPointCloud* axisLineCloud = new ccPointCloud;
		CCVector3 centroid3f(centroid[0], centroid[1], centroid[2]);
		CCVector3 axis3f(axis[0], axis[1], 0);
		CCVector3 _axis3f(-axis[1], axis[0], 0);
		axisLineCloud->addPoint(centroid3f + axis3f * min_dis + _axis3f * _max_dis);
		axisLineCloud->addPoint(centroid3f + axis3f * min_dis + _axis3f * _min_dis);
		axisLineCloud->addPoint(centroid3f + axis3f * max_dis + _axis3f * _min_dis);
		axisLineCloud->addPoint(centroid3f + axis3f * max_dis + _axis3f * _max_dis);
		ccPolyline* axisLine = new ccPolyline(axisLineCloud);
		axisLine->addPointIndex(0);
		axisLine->addPointIndex(1);
		axisLine->addPointIndex(2);
		axisLine->addPointIndex(3);
		axisLine->addPointIndex(0);
		axisLine->setColor(ccColor::red);
		axisLine->showColors(true);
		axisLine->setName("主方向线段");
		debug->addChild(axisLine);


		// ============================每块stripe，用主轴方向生成矩形=========================
		for (size_t i = 0; i < stripes.size(); ++i)
		{
			std::vector<Eigen::Vector3f> stripePoints;

			// 收集当前stripe的所有点
			for (int binId : stripes[i])
			{
				for (int ptIdx : bins[binId])
				{
					const CCVector3* pt = inputCloud->getPoint(ptIdx);
					stripePoints.emplace_back(pt->x, pt->y, pt->z);
				}
			}

			if (stripePoints.size() < 10)
				continue; // 点数太少，跳过拟合

			// 使用已知主轴 axis 和垂直方向构造矩形
			Eigen::Vector3f stripeDir = Eigen::Vector3f(-axis.y(), axis.x(), 0);  // 条纹方向，垂直于axis
			Eigen::Vector3f stripePerp = axis;  // 沿着主轴的方向用于计算长度

			// 点中心
			Eigen::Vector3f mean(0.0f, 0.0f, 0.0f);
			for (const auto& pt : stripePoints)
			{
				mean += pt;
			}
			mean /= static_cast<float>(stripePoints.size());


			// 投影到 stripeDir / stripePerp，找边界
			std::vector<float> projAlong, projPerp;
			for (const auto& pt : stripePoints)
			{
				Eigen::Vector3f d = pt - mean;
				projAlong.push_back(d.dot(stripeDir));
				projPerp.push_back(d.dot(stripePerp));
			}

			auto minmaxAlong = std::minmax_element(projAlong.begin(), projAlong.end());
			auto minmaxPerp = std::minmax_element(projPerp.begin(), projPerp.end());

			float halfLen = (*minmaxAlong.second - *minmaxAlong.first) / 2.0f;
			float halfWid = (*minmaxPerp.second - *minmaxPerp.first) / 2.0f;
			Eigen::Vector3f center = mean + stripeDir * ((*minmaxAlong.second + *minmaxAlong.first) / 2.0f)
				+ stripePerp * ((*minmaxPerp.second + *minmaxPerp.first) / 2.0f);

			// 构造矩形四个角点
			std::vector<Eigen::Vector3f> corners;
			corners.push_back(center + stripeDir * halfLen + stripePerp * halfWid);
			corners.push_back(center - stripeDir * halfLen + stripePerp * halfWid);
			corners.push_back(center - stripeDir * halfLen - stripePerp * halfWid);
			corners.push_back(center + stripeDir * halfLen - stripePerp * halfWid);

			// 添加 ctrolPoints 到 debug 中
			ccPointCloud* rectCloud = new ccPointCloud;
			for (const auto& pt : corners)
				rectCloud->addPoint(CCVector3(pt.x(), pt.y(), pt.z()));
			rectCloud->addPoint(CCVector3(corners[0].x(), corners[0].y(), corners[0].z())); // 闭环

			ccPolyline* rect = new ccPolyline(rectCloud);
			for (unsigned j = 0; j < 5; ++j)
				rect->addPointIndex(j);
			rect->setColor(ccColor::redRGB);
			rect->showColors(true);
			rect->setName(QString("Stripe_Rect_%1").arg(i));
			debug->addChild(rect);
		}
	}
}

#include <ccOctree.h>  // CloudCompare 中的 Octree 头文件
#include <stack>

void CloudProcess::cluster_points_around_pos(ccPointCloud* select_cloud, unsigned idx,
	float radius, ccPointCloud& clustered_cloud)
{
	ccOctree::Shared octree = PointCloudIO::get_octree(select_cloud);
	if (!octree)
	{
		return;
	}

	// 创建一个访问标志和存储连通区域点的索引
	std::vector<bool> visited(select_cloud->size(), false);
	std::vector<int> cluster_indices;

	// 使用 DFS 查找邻域点并进行区域生长

	int level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
	auto dfs = [&](int start_idx)
	{
		// 创建一个栈来实现 DFS
		std::stack<int> stack;
		stack.push(start_idx);

		while (!stack.empty())
		{
			int current_idx = stack.top();
			stack.pop();

			if (visited[current_idx]) continue;

			// 标记该点为已访问
			visited[current_idx] = true;
			cluster_indices.push_back(current_idx);

			CCCoreLib::DgmOctree::NeighboursSet neighbours;
			
			octree->getPointsInSphericalNeighbourhood(*select_cloud->getPoint(current_idx), radius, neighbours, level);

			for (auto x : neighbours)
			{
				if (!visited[x.pointIndex])
				{
					stack.push(x.pointIndex);
				}
			}
		}
	};

	// 启动 DFS 区域生长
	dfs(idx);

	// 将所有聚类点加入 clustered_cloud
	for (int cluster_idx : cluster_indices)
	{
		clustered_cloud.addPoint(*select_cloud->getPoint(cluster_idx));
	}
}

// // 计算局部二维法向量
// void CloudProcess::computeLocalNormal2D(const PCLCloudPtr& cloud, float radius, pcl::PointCloud<pcl::Normal>::Ptr normals)
// {
//     // 创建KD树
//     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//     kdtree.setInputCloud(cloud);

//     normals->resize(cloud->size());

//     #pragma omp parallel for schedule(dynamic, 1)
//     for (int i = 0; i < cloud->size(); i++)
//     {
//         const auto& point = cloud->points[i];
        
//         // 查找邻域点
//         std::vector<int> pointIdxRadiusSearch;
//         std::vector<float> pointRadiusSquaredDistance;
//         kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

//         // 如果邻域点太少，跳过
//         if (pointIdxRadiusSearch.size() < 3)
//         {
//             normals->points[i].normal_x = 0;
//             normals->points[i].normal_y = 0;
//             normals->points[i].normal_z = 1;  // 默认Z轴向上
//             normals->points[i].curvature = 0;
//             continue;
//         }

//         // 计算邻域点的质心
//         float center_x = 0, center_y = 0;
//         for (int idx : pointIdxRadiusSearch)
//         {
//             center_x += cloud->points[idx].x;
//             center_y += cloud->points[idx].y;
//         }
//         center_x /= pointIdxRadiusSearch.size();
//         center_y /= pointIdxRadiusSearch.size();

//         // 计算协方差矩阵
//         float cov_xx = 0, cov_xy = 0, cov_yy = 0;
//         for (int idx : pointIdxRadiusSearch)
//         {
//             float dx = cloud->points[idx].x - center_x;
//             float dy = cloud->points[idx].y - center_y;
//             cov_xx += dx * dx;
//             cov_xy += dx * dy;
//             cov_yy += dy * dy;
//         }
//         cov_xx /= pointIdxRadiusSearch.size();
//         cov_xy /= pointIdxRadiusSearch.size();
//         cov_yy /= pointIdxRadiusSearch.size();

//         // 计算协方差矩阵的特征值和特征向量
//         float trace = cov_xx + cov_yy;
//         float det = cov_xx * cov_yy - cov_xy * cov_xy;
//         float delta = std::sqrt(trace * trace - 4 * det);
        
//         // 计算较小的特征值对应的特征向量（法向量）
//         float lambda = (trace - delta) / 2;
//         float a = cov_xx - lambda;
//         float b = cov_xy;
        
//         // 归一化法向量
//         float norm = std::sqrt(a * a + b * b);
//         if (norm > 1e-6)
//         {
//             normals->points[i].normal_x = a / norm;
//             normals->points[i].normal_y = b / norm;
//             normals->points[i].normal_z = 0;  // XY平面的法向量，Z分量为0
//             normals->points[i].curvature = lambda / (lambda + (trace - lambda));  // 计算曲率
//         }
//         else
//         {
//             normals->points[i].normal_x = 0;
//             normals->points[i].normal_y = 0;
//             normals->points[i].normal_z = 1;  // 默认Z轴向上
//             normals->points[i].curvature = 0;
//         }
//     }
// }


// 计算局部二维法向量
void CloudProcess::computeLocalNormal2D(const PCLCloudPtr& cloud, float radius, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    // 创建KD树
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    normals->resize(cloud->size());

    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < cloud->size(); i++)
    {
        const auto& point = cloud->points[i];
        
        // 查找最近的两个点（包括当前点）
        std::vector<int> pointIdxNKNSearch(2);
        std::vector<float> pointNKNSquaredDistance(2);
        kdtree.nearestKSearch(point, 2, pointIdxNKNSearch, pointNKNSquaredDistance);

        // 如果找不到足够的点，跳过
        if (pointIdxNKNSearch.size() < 2)
        {
            normals->points[i].normal_x = 0;
            normals->points[i].normal_y = 0;
            normals->points[i].normal_z = 1;  // 默认Z轴向上
            normals->points[i].curvature = 0;
            continue;
        }

        // 获取最近的点（跳过当前点）
        int nearest_idx = (pointIdxNKNSearch[0] == i) ? pointIdxNKNSearch[1] : pointIdxNKNSearch[0];

        // 计算连线方向
        float dir_x = cloud->points[nearest_idx].x - point.x;
        float dir_y = cloud->points[nearest_idx].y - point.y;
        float dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y);

        if (dir_norm > 1e-6)
        {
            // 计算法向量（垂直于连线）
            float normal_x = -dir_y / dir_norm;  // 旋转90度并归一化
            float normal_y = dir_x / dir_norm;

            // 确保法向量指向线的外部
            // 计算质心
            float center_x = (point.x + cloud->points[nearest_idx].x) / 2;
            float center_y = (point.y + cloud->points[nearest_idx].y) / 2;

            // 计算质心到当前点的向量
            float to_point_x = point.x - center_x;
            float to_point_y = point.y - center_y;

            // 如果法向量与质心到点的向量方向相反，则翻转法向量
            if (normal_x * to_point_x + normal_y * to_point_y < 0)
            {
                normal_x = -normal_x;
                normal_y = -normal_y;
            }

            normals->points[i].normal_x = normal_x;
            normals->points[i].normal_y = normal_y;
            normals->points[i].normal_z = 0;  // XY平面的法向量，Z分量为0
            normals->points[i].curvature = 0;  // 暂时不计算曲率
        }
        else
        {
            normals->points[i].normal_x = 0;
            normals->points[i].normal_y = 0;
            normals->points[i].normal_z = 1;
            normals->points[i].curvature = 0;
        }
    }
}
