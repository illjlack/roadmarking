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

		// 将 polyline 中的点添加到 polylineCloud 中
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

/// <summary>
/// 裁剪点云：
/// 直接用射线法精细筛选点（没必要先使用凸包筛一遍）
/// </summary>
/// <param name="clouds">原始点云</param>
/// <param name="polygon_points">裁剪的闭合折线</param>
/// <param name="cloud_cropped">裁剪后的点云</param>
void CloudProcess::crop_cloud_with_polygon(
	const std::vector<ccPointCloud*>& clouds,            // 输入点云
	const std::vector<CCVector3d>& polygon_points,     // 自定义的裁剪区域（多边形）
	ccPointCloud* cloud_cropped                       // 输出裁剪后的点云
)
{
	if (clouds.empty() || !cloud_cropped)
	{
		return;
	}

	std::vector<cv::Point> polygon_cv;
	for (const auto& pt : polygon_points)
	{
		polygon_cv.push_back(cv::Point(pt.x, pt.y));
	}

	cloud_cropped->addScalarField("intensity");
	auto _sf = cloud_cropped->getScalarField(cloud_cropped->getScalarFieldIndexByName("intensity"));

	for (auto cloud : clouds)
	{
		int sfIdx = PointCloudIO::get_intensity_idx(cloud);
		if (sfIdx > -1)
		{

			auto sf = cloud->getScalarField(sfIdx);
			for (size_t i = 0; i < cloud->size(); ++i)
			{
				const auto& point = cloud->getPoint(i);
				cv::Point pt_2d(point->x, point->y);

				if (cv::pointPolygonTest(polygon_cv, pt_2d, true) >= 0)
				{
					if (sf) {
						cloud_cropped->addPoint(*point);
						_sf->addElement(sf->getValue(i));
					}
				}
			}
		}
	}
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
/// <param name="halfL">半长</param>
/// <param name="halfW">半宽</param>
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

	for (auto p : boxParams.neighbours)
	{
		points.push_back(*p.point);
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

void fitLineAndErode(PCLCloudPtr cloud, float distance_threshold, PCLCloudPtr cloud_filtered)
{
	// 创建一个SAC分割对象，用来从点云中提取直线
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// 设置模型类型和方法类型
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setOptimizeCoefficients(true);

	// 设置分割条件：直线模型
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setDistanceThreshold(distance_threshold); // 设置距离阈值，去除离线段较远的点

	// 执行分割，得到直线模型
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	// 如果没有找到符合条件的内点，退出函数
	if (inliers->indices.size() == 0)
	{
		return;
	}

	// 提取与直线模型相关的点
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false); // 保留点云中的直线部分
	extract.filter(*cloud_filtered);
}

/*
延伸直线、跨越断裂
*/
void CloudProcess::grow_line_from_seed(ccPointCloud* P,
	const CCVector3& p0,
	const CCVector3& v0,
	ccPointCloud* select_points,
	std::vector<CCVector3>& result,
	ccGLWindowInterface* m_glWindow, // debug
	bool isGetGround,
	double W,
	double L,
	unsigned Nmin,
	double theta_max,
	unsigned Kmax)
{
	if (!P || !select_points)return;

	CCVector3 curr_pt = p0;
	CCVector3 curr_dir = v0;
	unsigned jumpCount = 0;


	ccPointCloud* ground = isGetGround?PointCloudIO::get_ground_cloud(P):P;
	// 初步估计高程
	if (ground->size())
	{
		curr_pt.z = ground->getPoint(0)->z;
		std::vector<CCVector3> points;
		getPointsInBox(ground, curr_pt, curr_dir, L, W, points);
		if(points.size())curr_pt.z = points[0].z;
		result.push_back(curr_pt);
	}



	// debug计数
	int cnt = 0;

	ccHObject* debug = nullptr;
	if (m_glWindow)
	{
		debug = new ccHObject;
		debug->setName("debug");
		debug->setVisible(true);
		m_glWindow->addToOwnDB(debug);
	}

	while (true)
	{
		curr_dir.normalize();
		cnt++;

		float len = L;

		CCVector3 next_pt = curr_pt + curr_dir * L;
		std::vector<CCVector3> points;
		for (;; len += L)
		{
			next_pt = curr_pt + curr_dir * len;

			std::vector<CCVector3> new_points;
			getPointsInBox(ground, (next_pt + curr_pt) / 2, curr_dir, len , W, new_points);

			if (new_points.size() > points.size() + Nmin)
			{
				
				// 偏移让质心居中（现在里面应该是一条线段的点云，进行侵蚀操作）
				PCLCloudPtr pclCloud(new PCLCloud);
				for (auto& p : new_points)pclCloud->push_back({ p.x, p.y, p.z });
				PCLCloudPtr cloud_filtered(new PCLCloud);
				fitLineAndErode(pclCloud, 0.1, cloud_filtered);
				new_points.clear();
				for (auto p : *cloud_filtered)
				{
					new_points.push_back({p.x,p.y,p.z});
				}
				new_points.swap(points);
				CCVector3 points_centroid(0, 0, 0);
				for (const auto& pt : points)
				{
					points_centroid += pt;
				}
				points_centroid /= points.size();

				CCVector3 centroid = (curr_pt + next_pt) / 2;
				CCVector3 offset = points_centroid - centroid;
				curr_pt += offset;
				next_pt += offset;
			}
			else
			{
				break;
			}
		}
		
		if(debug){
			CCVector3 dv(-curr_dir.y, curr_dir.x, 0);
			dv.normalize();
			ccPointCloud* cloud = new ccPointCloud;
			cloud->addPoint(curr_pt + dv * W / 2);
			cloud->addPoint(curr_pt - dv * W / 2);
			cloud->addPoint(next_pt - dv * W / 2);
			cloud->addPoint(next_pt + dv * W / 2);
			ccPolyline* poly = new ccPolyline(cloud);
			poly->addChild(cloud);

			poly->addPointIndex(0);
			poly->addPointIndex(1);
			poly->addPointIndex(2);
			poly->addPointIndex(3);
			poly->addPointIndex(0);
			poly->setName(QString("debug_aabb_%1 _cloud_size:%2  _axis:%3,%4,%5").arg(cnt).arg(points.size()).arg(curr_dir[0]).arg(curr_dir[1]).arg(curr_dir[2]));
			debug->addChild(poly);
		}

		for (auto point : points)
		{
			select_points->addPoint(point);
		}

		int last_num = 1;

		// 如果找到足够的点，继续延伸
		if (points.size() >= Nmin)
		{
			Eigen::Vector4f centroid;
			Eigen::Vector3f axis;
			getPointsCentroidAndAxis(points, centroid, axis);
			axis.normalize();

			CCVector3 axis_n = { axis[0], axis[1], axis[2] };

			// 判断框内是否是线段
			// 旋转判断包围盒是否为长宽分明（1/2以上）的矩形
			{
				PCLCloudPtr pclCloud(new PCLCloud);
				for (auto& p : points)pclCloud->push_back({p.x, p.y, p.z});
				PCLCloudPtr rot_cloud = rotate_cloud(pclCloud, axis, {1,0,0}); // 长与x轴对齐

				pcl::PointXYZ min_pt, max_pt;
				pcl::getMinMax3D(*rot_cloud, min_pt, max_pt);

				float length = max_pt.x - min_pt.x; // 长
				float width = max_pt.y - min_pt.y;	// 宽

				// 判断长宽比是否大于等于 1/2
				if (width*2 > length)
				{
					jumpCount++;
					curr_pt = next_pt;
					// 丢弃这个片段
					continue;
				}
			}


			if (fabs(axis_n.z) > (fabs(axis_n.x) + fabs(axis_n.y))*10)
			{
				// xy方向特征太小了
				break;
			}

			if (curr_dir.dot(axis_n) < 0)
			{
				axis_n = -axis_n;
			}

			axis_n.normalize();

			float cosAngle = curr_dir.dot(axis_n);
			cosAngle = std::clamp(cosAngle, -1.0f, 1.0f);
			float angleRad = std::acos(cosAngle);
			//float angleDeg = angleRad * 180.0f / static_cast<float>(M_PI);
			if (angleRad > theta_max) //角度限制
			{
				jumpCount++;
				curr_pt = next_pt;
				continue;
			}


			curr_pt = next_pt;
			curr_dir = curr_dir*last_num + axis_n* points.size();  // 用点的数量做权值
			last_num = points.size();
			axis[2] = 0;
			curr_dir.normalize();
			result.push_back(curr_pt);  // 将新点加入结果

			// 重置跳跃计数器
			jumpCount = 0;
		}
		else
		{
			// 否则，进行跳跃（向前延伸）
			jumpCount++;

			// 如果超过最大跳跃次数，停止延伸
			if (jumpCount > Kmax)
			{
				break;
			}

			// 否则，继续跳跃
			curr_pt = next_pt; // 更新当前点
		}
	}
}




/*
根据设置的起始点和方向，延申提取点云中的“线”，可跨越断裂处

plan1:
（1）基础的，搜索出连续的点云。
（2）判断断裂处延伸方向：用矩形一段段拟合，找到末尾的方向 （这里比较有疑问）
（3）按这个方向，往前延伸矩形，直到阔选到新的点。

plan2：
（1）使用初始的线段拓展为一个矩形（确定长宽），计算矩形中点云数量和方向
（2）数量足够就往这个方向继续延申
（3）不够处继续往前延伸，直到最大阈值的跳跃

plan1：寻找末端方向有问题，且不够鲁棒（比如出现割裂，与别的线相交）。
plan2：对于是否能按预期中的适应线的转弯有疑问（局部的弯曲过大，矩形中的点少判断不了方向）。

*/
