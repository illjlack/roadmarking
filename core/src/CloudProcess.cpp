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
#include <opencv2/opencv.hpp>

using namespace roadmarking;

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

ccHObject* CloudProcess::apply_roadmarking_vectorization(ccCloudPtr cloud)
{
	ccHObject* polylineContainer(new ccHObject);
	GridProcess gridProcess(0.05);
	gridProcess.perform_orthogonal_grid_mapping(cloud);
	gridProcess.process_grid_to_polylines(polylineContainer);
	return polylineContainer;
}

ccHObject* CloudProcess::apply_roadmarking_vectorization(std::vector<PCLCloudPtr> pclClouds)
{
	// 创建分类器对象
	RoadMarkingClassifier classifier;

	// 用于保存分类结果
	RoadMarkings roadmarkings;

	ccHObject* allLinesContainer = new ccHObject();

	// 提示用户选择一个 JSON 文件
	/*QString model_path = QFileDialog::getOpenFileName(nullptr, "选择 JSON 文件", "", "JSON Files (*.json);;All Files (*)");
	if (model_path.isEmpty()) {
		QMessageBox::information(nullptr, "提示", "未选择文件");
		return allLinesContainer;
	}*/

	QString model_path = "F:\\RoadMarking\\CloudCompare\\plugins\\core\\Standard\\qRoadMarking\\model\\model.json";

	// 调用分类函数
	classifier.ClassifyRoadMarkings(pclClouds, roadmarkings, model_path.toStdString());

	// 遍历分类结果中的每个 roadmarking
	for (const auto& roadmarking : roadmarkings)
	{
		// 获取分类中的折线（polyline）
		const std::vector<pcl::PointXYZ>& polyline = roadmarking.polyline;

		// 创建 ccPointCloud 对象，用来存储折线的点
		ccCloudPtr polylineCloud(new ccPointCloud);

		// 将 ctrolPoints 中的点添加到 polylineCloud 中
		for (const auto& point : polyline)
		{
			polylineCloud->addPoint({ point.x ,point.y, point.z });
		}

		// 创建 ccPolyline 对象并将 polylineCloud 作为输入点云
		ccPolyline* polylineObj = new ccPolyline(polylineCloud.release());

		polylineObj->addChild(polylineCloud.get()); // 一起删除

		// 预留空间用于折线点索引
		polylineObj->reserve(static_cast<unsigned>(polyline.size()));

		// 将折线中的每个点添加到 ccPolyline 中
		for (size_t i = 0; i < polyline.size(); ++i)
		{
			polylineObj->addPointIndex(static_cast<unsigned>(i));
		}

		// 将创建的 ccPolyline 添加到 allLinesContainer 中
		polylineObj->setVisible(true);
		polylineObj->setColor(ccColor::redRGB);
		polylineObj->showColors(true);
		allLinesContainer->addChild(polylineObj);
	}
	allLinesContainer->setVisible(true);
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

std::vector<PCLPoint> CloudProcess::draw_polyline_on_cloud_by_pcl_view(const PCLCloudPtr& inputCloud) {
	// 创建可视化器并设置窗口名称
	pcl::visualization::PCLVisualizer viewer("Point Cloud Polyline Drawer");

	// 计算点云中心并居中显示
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*inputCloud, centroid);
	PCLCloudPtr centeredCloud(new pcl::PointCloud<PCLPoint>);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() = -centroid.head(3);  // 设置平移向量
	pcl::transformPointCloud(*inputCloud, *centeredCloud, transform);  // 使用变换矩阵

	// 添加点云并设置渲染属性
	viewer.addPointCloud(centeredCloud, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer.addCoordinateSystem(0.5);  // 添加坐标系参考

	// 存储绘制数据
	std::vector<PCLPoint> polylinePoints;
	bool exit_flag = false;
	bool draw_mode = false;  // 绘制模式标志

	// 定义点拾取回调函数
	auto point_picking_cb = [&](const pcl::visualization::PointPickingEvent& event) {
		if (event.getPointIndex() == -1) return;

		// 获取点击点的坐标（基于居中后的坐标系）
		PCLPoint clickedPoint;
		event.getPoint(clickedPoint.x, clickedPoint.y, clickedPoint.z);

		// 转换回原始坐标系
		clickedPoint.x += centroid.x();
		clickedPoint.y += centroid.y();
		clickedPoint.z += centroid.z();

		// 如果已有点且新点与上一个点相同，则不添加
		if (!polylinePoints.empty()) {
			const auto& lastPoint = polylinePoints.back();
			if (clickedPoint.getVector3fMap() == lastPoint.getVector3fMap()) {
				return;
			}
		}

		// 添加到折线点集
		polylinePoints.push_back(clickedPoint);

		// 绘制线段（当有至少两个点时）
		if (polylinePoints.size() > 1) {
			std::string line_id = "line_" + std::to_string(polylinePoints.size() - 1);
			const auto& pt1 = polylinePoints[polylinePoints.size() - 2];
			const auto& pt2 = polylinePoints.back();

			// 将点转换回居中坐标系用于显示
			PCLPoint display_pt1, display_pt2;
			display_pt1.getVector3fMap() = pt1.getVector3fMap() - centroid.head<3>();
			display_pt2.getVector3fMap() = pt2.getVector3fMap() - centroid.head<3>();

			viewer.addLine<PCLPoint>(display_pt1, display_pt2, 0, 1, 0, line_id);
		}
	};

	// 撤销最后一条线段
	auto undo_last_action = [&]() {
		if (polylinePoints.size() < 2) return;  // 至少需要两个点才能绘制线段

		// 删除最后添加的点和线段
		polylinePoints.pop_back();
		viewer.removeShape("line_" + std::to_string(polylinePoints.size()));

		// 如果还有点，则删除最后一条线段
		if (polylinePoints.size() > 1) {
			const auto& pt1 = polylinePoints[polylinePoints.size() - 2];
			const auto& pt2 = polylinePoints.back();

			// 将点转换回居中坐标系用于显示
			PCLPoint display_pt1, display_pt2;
			display_pt1.getVector3fMap() = pt1.getVector3fMap() - centroid.head<3>();
			display_pt2.getVector3fMap() = pt2.getVector3fMap() - centroid.head<3>();

			std::string line_id = "line_" + std::to_string(polylinePoints.size() - 1);
			viewer.addLine<PCLPoint>(display_pt1, display_pt2, 0, 1, 0, line_id);
		}
	};

	// 注册点选取回调函数
	viewer.registerPointPickingCallback(point_picking_cb);

	// 注册键盘事件回调，用来触发撤销操作
	viewer.registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent& event) {
		if (event.getKeySym() == "z" && event.keyDown() && event.isCtrlPressed()) {
			undo_last_action();  // 按下 Ctrl + Z 撤销
		}
		if (event.getKeySym() == "BackSpace" && event.keyDown()) {
			undo_last_action();  // 按下 BackSpace 键撤销
		}
		});

	while (!viewer.wasStopped() && !exit_flag) {
		viewer.spinOnce(100); // 必须调用以处理事件
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	// 清除所有图形对象
	viewer.removeAllShapes();
	return polylinePoints;
}

void CloudProcess::crop_cloud_with_polygon(
	const std::vector<ccPointCloud*>& clouds,
	const std::vector<CCVector3d>& polygon_points,
	ccPointCloud* cloud_cropped
)
{
	if (clouds.empty() || !cloud_cropped || polygon_points.size() < 3)
		return;

	// ==================== 1. 构建 polygon + AABB ====================
	std::vector<cv::Point2f> polygon_cv;
	double minX = DBL_MAX, maxX = -DBL_MAX;
	double minY = DBL_MAX, maxY = -DBL_MAX;

	for (const auto& pt : polygon_points)
	{
		polygon_cv.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
		minX = std::min(minX, pt.x);
		maxX = std::max(maxX, pt.x);
		minY = std::min(minY, pt.y);
		maxY = std::max(maxY, pt.y);
	}

	// ==================== 2. 准备输出 scalar field ====================
	cloud_cropped->addScalarField("intensity");
	auto _sf = cloud_cropped->getScalarField(cloud_cropped->getScalarFieldIndexByName("intensity"));

	// ==================== 3. 并行裁剪所有点云 ====================
	for (auto cloud : clouds)
	{
		int sfIdx = PointCloudIO::get_intensity_idx(cloud);
		if (sfIdx < 0) continue;

		auto sf = cloud->getScalarField(sfIdx);
		if (!sf) continue;

		const size_t pointCount = cloud->size();
		std::vector<CCVector3> local_points_all;
		std::vector<ScalarType> local_scalars_all;

		// ========== 3.1 对大规模点云内部进行并行裁剪 ==========
#pragma omp parallel
		{
			std::vector<CCVector3> local_points;
			std::vector<ScalarType> local_scalars;
#pragma omp for schedule(static)
			for (int i = 0; i < static_cast<int>(pointCount); ++i)
			{
				const CCVector3* pt = cloud->getPoint(i);
				float x = pt->x;
				float y = pt->y;

				// AABB 快速排除
				if (x < minX || x > maxX || y < minY || y > maxY)
					continue;

				// 射线法精细判断
				if (cv::pointPolygonTest(polygon_cv, cv::Point2f(x, y), false) > 0)
				{
					local_points.push_back(*pt);
					local_scalars.push_back(sf->getValue(i));
				}
			}

			// ========== 3.2 合并当前线程结果 ==========
#pragma omp critical
			{
				local_points_all.insert(local_points_all.end(), local_points.begin(), local_points.end());
				local_scalars_all.insert(local_scalars_all.end(), local_scalars.begin(), local_scalars.end());
			}
		}

		// ========== 3.3 将合并结果写入输出点云 ==========
		for (size_t j = 0; j < local_points_all.size(); ++j)
		{
			cloud_cropped->addPoint(local_points_all[j]);
			_sf->addElement(local_scalars_all[j]);
		}
	}

	// ==================== 4. 完善输出字段 & 可视化设置 ====================
	_sf->computeMinAndMax();
	CloudProcess::apply_default_intensity_and_visible(cloud_cropped);
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

	// 为输出点云创建一个“intensity”标量场
	cloud_filtered->addScalarField("intensity");
	int outSfIdx = cloud_filtered->getScalarFieldIndexByName("intensity");
	auto sfOut = cloud_filtered->getScalarField(outSfIdx);

	// 找到输入点云的“intensity”标量场索引
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

	// 为输出点云创建一个“intensity”标量场
	cloud_filtered->addScalarField("intensity");
	int outSfIdx = cloud_filtered->getScalarFieldIndexByName("intensity");
	auto sfOut = cloud_filtered->getScalarField(outSfIdx);

	// 找到输入点云的“intensity”标量场索引
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

	if (!cloud->getOctree())
	{
		ccProgressDialog progressDlg(false);
		progressDlg.setWindowTitle("构建八叉树");
		progressDlg.setAutoClose(true);
		if (!cloud->computeOctree(&progressDlg))
		{
			// 构建失败
			QMessageBox::critical(nullptr,
				QStringLiteral("错误"),
				QStringLiteral("八叉树构建失败！"));
			return;
		}
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
	boxParams.level = cloud->getOctree()->findBestLevelForAGivenNeighbourhoodSizeExtraction(std::min(L,W));
	cloud->getOctree()->getPointsInBoxNeighbourhood(boxParams);

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
/// 从固定起点 start_point 沿 dir 大致方向，用“矩形走廊”方法搜索 ±30° 扫描，
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

	// 2. 准备“上下浮动”范围：±30°
	constexpr float maxAngleRad = 30.0f * static_cast<float>(M_PI) / 180.0f;
	const int   numSteps = 61;  // 扫描粒度：每度一次

	// 3. 确定“摆动轴”：取 d0 与全局“上”（0,0,1）做叉乘，如果平行再换 (1,0,0)
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
	
	for (unsigned i = 0; i < inputCloud->size(); ++i)
	{
		Eigen::Vector3f p(inputCloud->getPoint(i)->x, inputCloud->getPoint(i)->y, inputCloud->getPoint(i)->z);
		float proj = (p - Eigen::Vector3f(centroid[0], centroid[1], centroid[2])).dot(axis);
		int binId = static_cast<int>(proj / binWidth);
		bins[binId].push_back(i);
		binPosMap[binId] = proj;
	}

	// 2.简单确定阈值，是一个中位数 / 平均值 / 最大类间方差
	int baseThreshold;
	{
		// 统计所有bin中的点数作为密度
		std::vector<int> densityValues;
		for (const auto& [binId, pts] : bins)
			densityValues.push_back(static_cast<int>(pts.size()));

		// --------------------------
		// 方法一：中位数阈值
		// --------------------------
		
		//std::nth_element(densityValues.begin(), densityValues.begin() + densityValues.size() / 2, densityValues.end());
		//baseThreshold = densityValues[densityValues.size() / 2];
		

		// --------------------------
		// 方法二：平均值阈值
		// --------------------------

		int sum = std::accumulate(densityValues.begin(), densityValues.end(), 0);
		baseThreshold = static_cast<int>(sum / densityValues.size());
		

		// --------------------------
		// 方法三：最大类间方差(阈值大了)
		// --------------------------
		/*
		const int maxValue = *std::max_element(densityValues.begin(), densityValues.end());
		std::vector<int> histogram(maxValue + 1, 0);
		for (int v : densityValues)
			histogram[v]++;

		int totalPoints = static_cast<int>(densityValues.size());
		double sumAll = 0;
		for (int t = 0; t <= maxValue; ++t)
			sumAll += t * histogram[t];

		int threshold = 0;
		double maxVariance = -1.0;
		int wB = 0;
		double sumB = 0;

		for (int t = 0; t <= maxValue; ++t) {
			wB += histogram[t];
			if (wB == 0) continue;

			int wF = totalPoints - wB;
			if (wF == 0) break;

			sumB += t * histogram[t];
			double mB = sumB / wB;
			double mF = (sumAll - sumB) / wF;

			double betweenVar = static_cast<double>(wB) * wF * (mB - mF) * (mB - mF);
			if (betweenVar > maxVariance) {
				maxVariance = betweenVar;
				threshold = t;
			}
		}
		baseThreshold = threshold;
		*/
	}


	// 3.计算线区域
	std::vector<std::vector<int>> stripes;
	int medianStripeLength = 0;
	int medianGapLength = 0;
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

		medianStripeLength = median(stripeLengths);
		medianGapLength = median(gapLengths);

		// ========================= 修复stripes
		// (首尾两段的stripe往两边补切)
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
			rect->setColor(ccColor::yellow);
			rect->showColors(true);
			rect->setName(QString("Stripe_Rect_%1").arg(i));
			debug->addChild(rect);
		}

	}
}


void CloudProcess::extract_zebra_by_projection(
	ccPointCloud* inputCloud,
	float binWidth,
	int densityThreshold,
	float minStripeLength,
	ccGLWindowInterface* m_glWindow,
	ccPointCloud* outputCloud,
	std::vector<CCVector3>& centers
) {
	std::vector<CCVector3> dirs;
	std::vector<float> lengths;
	if (!inputCloud || !outputCloud)
		return;

	ccHObject* debug = nullptr;
	if (m_glWindow)
	{
		auto debugCloud = new ccPointCloud("debug_centers");
		debug = new ccHObject("zebra_debug");
		debug->addChild(debugCloud);
		m_glWindow->addToOwnDB(debug);
	}

	std::vector<unsigned> allIndices(inputCloud->size());
	for (unsigned i = 0; i < inputCloud->size(); ++i)
		allIndices[i] = i;

	Eigen::Vector4f centroid;
	Eigen::Vector3f axis;
	getPointsCentroidAndAxis(inputCloud, allIndices, centroid, axis);

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
	}

	std::map<int, std::vector<int>> bins;
	std::map<int, float> binPosMap;
	for (unsigned i = 0; i < inputCloud->size(); ++i)
	{
		Eigen::Vector3f p(inputCloud->getPoint(i)->x, inputCloud->getPoint(i)->y, inputCloud->getPoint(i)->z);
		float proj = (p - Eigen::Vector3f(centroid[0], centroid[1], centroid[2])).dot(axis);
		int binId = static_cast<int>(proj / binWidth);
		bins[binId].push_back(i);
		binPosMap[binId] = proj;
	}

	std::vector<int> densityValues;
	for (const auto& [binId, pts] : bins)
		densityValues.push_back(static_cast<int>(pts.size()));

	std::nth_element(densityValues.begin(), densityValues.begin() + densityValues.size() / 2, densityValues.end());
	int autoDensityThreshold = densityValues[densityValues.size() / 2];

	ccLog::Print(QString("自动估计的密度阈值（中位数）: %1").arg(autoDensityThreshold));

	std::vector<std::vector<int>> stripes;
	std::vector<int> currentStripe;
	std::vector<int> sortedBinIds;
	for (const auto& [binId, _] : bins)
		sortedBinIds.push_back(binId);
	std::sort(sortedBinIds.begin(), sortedBinIds.end());

	for (size_t i = 0; i < sortedBinIds.size(); ++i)
	{
		int binId = sortedBinIds[i];
		if (bins[binId].size() >= autoDensityThreshold)
		{
			if (currentStripe.empty() || binId == currentStripe.back() + 1)
				currentStripe.push_back(binId);
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

	// Step 2: 推断白线宽度 + 修正 stripes（基于间距）
	std::vector<float> centerProjs;
	for (const auto& stripe : stripes)
	{
		float proj = 0.5f * (binPosMap[stripe.front()] + binPosMap[stripe.back()]);
		centerProjs.push_back(proj);
	}

	std::vector<float> gaps;
	for (size_t i = 1; i < centerProjs.size(); ++i)
		gaps.push_back(centerProjs[i] - centerProjs[i - 1]);
	if (!gaps.empty()) {
		std::nth_element(gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end());
		float gapMed = gaps[gaps.size() / 2];
		std::vector<std::vector<int>> repaired;
		for (size_t i = 0; i < stripes.size(); ++i)
		{
			repaired.push_back(stripes[i]);
			if (i + 1 < stripes.size())
			{
				float dist = centerProjs[i + 1] - centerProjs[i];
				int missing = int(std::round(dist / gapMed)) - 1;
				if (missing >= 1 && missing <= 3)
				{
					for (int m = 1; m <= missing; ++m)
					{
						float inferred = centerProjs[i] + gapMed * m;
						Eigen::Vector3f inferPos = centroid.head<3>() + axis * inferred;
						if (debug) {
							auto dbgCloud = static_cast<ccPointCloud*>(debug->getChild(0));
							CCVector3 infpt(inferPos.x(), inferPos.y(), inferPos.z());
							dbgCloud->addPoint(infpt);
						}
					}
				}
			}
		}
		stripes = std::move(repaired);
	}


	// 遍历每一段连续高密度 bin 组成的白线 stripe
	for (const std::vector<int>& stripe : stripes)
	{
		// 聚合该 stripe 所有 bin 中的点索引
		std::vector<int> stripeIndices;
		for (int binId : stripe)
		{
			const std::vector<int>& idxs = bins[binId];
			stripeIndices.insert(stripeIndices.end(), idxs.begin(), idxs.end());
		}

		// 计算该 stripe 在主轴投影上的起止位置和长度
		float startProj = binPosMap[stripe.front()];
		float endProj = binPosMap[stripe.back()];
		float stripeLen = std::abs(endProj - startProj);
		if (stripeLen < minStripeLength)
			continue;  // 太短则跳过，可能是误识别

		// 计算 stripe 所有点的中心点坐标（用于可视化和方向计算参考点）
		Eigen::Vector3f sum(0, 0, 0);
		for (int idx : stripeIndices)
		{
			const CCVector3* pt = inputCloud->getPoint(idx);
			sum += Eigen::Vector3f(pt->x, pt->y, pt->z);
		}
		sum /= stripeIndices.size();
		CCVector3 center(sum.x(), sum.y(), sum.z());
		centers.push_back(center);              // 存入输出中心点数组
		outputCloud->addPoint(center);         // 添加到结果点云中

		// 可视化中心点为红点（调试用）
		if (debug)
		{
			auto debugCloud = static_cast<ccPointCloud*>(debug->getChild(0));
			auto cp = center;
			debugCloud->addPoint(cp);
		}

		// 计算该 stripe 的主方向向量
		CCVector3 localDir(0, 0, 0);
		if (stripeIndices.size() >= 5)
		{
			std::vector<CCVector3> stripePts;
			for (int idx : stripeIndices)
				stripePts.push_back(*inputCloud->getPoint(idx));
			localDir = fitPCADirection(stripePts);  // 使用 PCA 拟合方向
		}
		else
		{
			localDir = CCVector3(axis[0], axis[1], axis[2]);  // 太少则退化为整体主方向
		}
		dirs.push_back(localDir);
		lengths.push_back(stripeLen);

		// 可视化白线主方向线段（绿色）
		if (debug)
		{
			ccPointCloud* linePts = new ccPointCloud;
			linePts->addPoint(center - localDir * (stripeLen * 0.5f));
			linePts->addPoint(center + localDir * (stripeLen * 0.5f));

			// 创建用于可视化白线方向的线段，并添加到 debug 树中
			ccPolyline* line = new ccPolyline(linePts);
			line->addPointIndex(0);
			line->addPointIndex(1);
			line->setColor(ccColor::green);
			line->showColors(true);
			line->setName("stripe");
			debug->addChild(line);
		}
	}
}
