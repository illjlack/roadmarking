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

using namespace roadmarking;

PCLOctreePtr CloudProcess::buildOctree(PCLCloudPtr pclCloud, float targetVoxelSize)
{
	PCLOctreePtr pclOctree(new PCLOctree(targetVoxelSize));
	pclOctree->setInputCloud(pclCloud);
	pclOctree->addPointsFromInputCloud();
	return pclOctree;
}

ccCloudPtr CloudProcess::CropBySparseCloud(ccCloudPtr ccCloud, PCLCloudPtr pclCloud, PCLOctreePtr octree)
{
	ccPointCloud* croppedCloud = new ccPointCloud();
	std::unordered_set<int> unique_indices;

	std::vector<int> scalarFieldIndices(ccCloud->getNumberOfScalarFields());
	std::iota(scalarFieldIndices.begin(), scalarFieldIndices.end(), 0);

	if (!octree)
	{
		octree = buildOctree(PointCloudIO::convertToPCLCloud(ccCloud), 0.2);
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

ccCloudPtr CloudProcess::applyVoxelGridFiltering(ccCloudPtr ccCloud, float targetVoxelSize, PCLOctreePtr octree)
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
		octree = buildOctree(PointCloudIO::convertToPCLCloud(ccCloud), 0.2);
	}

	// 转换为 PCL 格式点云
	PCLCloudPtr pclCloud = PointCloudIO::convertToPCLCloud(ccCloud);
	// 体素网格滤波
	PCLCloudPtr filteredCloud = applyVoxelGridFiltering(pclCloud, targetVoxelSize);

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

PCLCloudPtr CloudProcess::applyVoxelGridFiltering(PCLCloudPtr pclCloud, float targetVoxelSize)
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

ccCloudPtr CloudProcess::applyCSFGroundExtraction(ccCloudPtr ccCloud)
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
	if (clothMesh)delete clothMesh;

	if (result)
	{
		return ccCloudPtr(groundCloud);
	}
	if (groundCloud) delete groundCloud;
	return nullptr;
}

PCLCloudPtr CloudProcess::applyCSFGroundExtraction(PCLCloudPtr pclCloud)
{
	ccCloudPtr ccCloud = PointCloudIO::convertToCCCloud(pclCloud);
	ccCloudPtr groundCloud = applyCSFGroundExtraction(ccCloud);
	PCLCloudPtr pclGroundCloud = PointCloudIO::convertToPCLCloud(groundCloud);
	return pclGroundCloud;
}

PCLCloudPtr CloudProcess::extractMaxCloudByEuclideanCluster(PCLCloudPtr groundCloud, float euclideanClusterRadius)
{
	std::vector<PCLCloudPtr> clouds;
	extractEuclideanClusters<PCLPoint>(groundCloud, clouds, euclideanClusterRadius);
	if (clouds.size() == 0)
	{
		return nullptr;
	}
	auto largestCluster = std::max_element(clouds.begin(), clouds.end(),
		[](const auto& a, const auto& b) { return a->size() < b->size(); });
	return *largestCluster;
}

PCLCloudPtr CloudProcess::extractRoadPoints(PCLCloudPtr groundCloud,
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

std::vector<PCLCloudXYZIPtr> CloudProcess::extractRoadMarking(ccCloudPtr roadCloud,
	float resolution,
	double euclideanClusterRadius,
	int minNum)
{
	GridProcess gridProcess(resolution);
	gridProcess.performOrthogonalGridMapping(roadCloud);
	gridProcess.divByDoubleAdaptiveIntensityThreshold();
	PCLCloudXYZIPtr markingCloud(new PCLCloudXYZI);
	gridProcess.restoreFromGridToPointCloud(markingCloud);

	std::vector<PCLCloudXYZIPtr> clouds;
	extractEuclideanClusters<PCLPointXYZI>(markingCloud, clouds, euclideanClusterRadius, minNum);

	return clouds;
}

PCLCloudPtr CloudProcess::matchRoadMarking(PCLCloudPtr pclCloud)
{
	//TemplateMatcher matcher;
	//matcher.setScenePointCloud(pclCloud);
	//matcher.matchTemplates();
	//return matcher.getBestMatchCloud();
	return nullptr;
}

ccHObject* CloudProcess::applyVectorization(ccCloudPtr cloud)
{
	ccHObject* polylineContainer(new ccHObject);
	GridProcess gridProcess(0.05);
	gridProcess.performOrthogonalGridMapping(cloud);
	gridProcess.processGridToPolylineCloud(polylineContainer);
	return polylineContainer;
}

ccHObject* CloudProcess::applyVectorization(std::vector<PCLCloudPtr> pclClouds)
{
	// 创建分类器对象
	RoadMarkingClassifier classifier;

	// 用于保存分类结果
	RoadMarkings roadmarkings;

	ccHObject* allLinesContainer = new ccHObject();

	// 提示用户选择一个 JSON 文件
	QString model_path = QFileDialog::getOpenFileName(nullptr, "选择 JSON 文件", "", "JSON Files (*.json);;All Files (*)");
	if (model_path.isEmpty()) {
		QMessageBox::information(nullptr, "提示", "未选择文件");
		return allLinesContainer;
	}

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
		allLinesContainer->addChild(polylineObj);
	}
	allLinesContainer->setVisible(true);
	return allLinesContainer;
}

template <typename PointT>
void CloudProcess::extractEuclideanClusters(
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

PCLCloudPtr CloudProcess::extractOutline(const PCLCloudPtr& inputCloud, float alpha)
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

std::vector<PCLPoint> CloudProcess::visualizeAndDrawPolyline(const PCLCloudPtr& inputCloud) {
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
