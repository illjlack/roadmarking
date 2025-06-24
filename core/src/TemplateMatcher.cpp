#include "TemplateMatcher.h"
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/common/pca.h>
#include "CloudProcess.h"
#include "comm.h"
#include "ConfigManager.h"
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <cmath>
#include <omp.h>

using namespace std;
#include <pcl/visualization/pcl_visualizer.h>


enum ROADMARKING_TYPE
{
	SIDE_LINE = 100001,
	COMBINE_SIDE_LINES = 100002
};


using namespace roadmarking;

inline int l(int pos)
{
	return pos << 1;
}

inline int r(int pos)
{
	return pos << 1 | 1;
}

void RoadMarkingClassifier::ClassifyRoadMarkings(const std::vector<PCLCloudPtr>& clouds,
	RoadMarkings& roadmarkings,
	const std::string& model_path)
{
	std::vector<Model> models = Model::loadFromJson(model_path);
	ClassifyRoadMarkings(clouds, roadmarkings, models);
}

void RoadMarkingClassifier::ClassifyRoadMarkings(const std::vector<PCLCloudPtr>& clouds,
	RoadMarkings& roadmarkings,
	std::vector<Model>& models)
{
	model_match(models, clouds, roadmarkings);
	vectorize_roadmarking(models, roadmarkings);
}

void RoadMarkingClassifier::vectorize_roadmarking(std::vector<Model>& models, RoadMarkings& roadmarkings)
{
	for (auto& roadmarking : roadmarkings)
	{
		if (roadmarking.category < 0)
		{
			continue;
		}

		if (roadmarking.category == SIDE_LINE) // 如果是侧线，则跳过
		{
			continue;
		}
		roadmarking.name = models[roadmarking.category].name;

		// 找到对应类别的模型
		Model& model = models[roadmarking.category];

		// 清空旧的多条折线（如果之前已有残留）
		roadmarking.polylines.clear();

		// 获取对应的变换矩阵
		const Eigen::Matrix4f& transform = roadmarking.localization_tranmat_m2s;

		// 遍历该模型下的每一条"原始折线"
		for (const auto& singlePolyline : model.vectorized_polylines)
		{
			// 先把这一条折线的所有顶点装到一个 PCL 点云里
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
			cloud_in->reserve(singlePolyline.size());
			for (const auto& pt : singlePolyline)
			{
				cloud_in->push_back(pt);
			}

			// 创建一个新的点云来存储变换后的结果
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

			// 对该折线进行坐标变换
			pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

			// 将变换后的点云转回 std::vector<pcl::PointXYZ> 并加入到 roadmarking.polylines
			std::vector<pcl::PointXYZ> transformedPolyline;
			transformedPolyline.reserve(cloud_out->size());
			for (const auto& pt : cloud_out->points)
			{
				transformedPolyline.push_back(pt);
			}

			roadmarking.polylines.push_back(std::move(transformedPolyline));
		}
	}

	// 下面依旧按原有逻辑合并侧线，结果也要写回 roadmarkings
	RoadMarkings combine_sideline_markings;
	combine_side_lines(roadmarkings, 20, combine_sideline_markings);

	RoadMarkings result;
	for (auto& rm : roadmarkings)
	{
		if (rm.category != SIDE_LINE)
		{
			result.push_back(rm);
		}
	}
	for (auto& rm : combine_sideline_markings)
	{
		result.push_back(rm);
	}
	result.swap(roadmarkings);
}

bool RoadMarkingClassifier::model_match(const std::vector<Model>& models, const std::vector<PCLCloudPtr>& scenePointClouds, RoadMarkings& roadmarkings)
{
	// 获取配置管理器实例
	auto& config = ConfigManager::getInstance();
	
	// 从配置中获取参数
	float normal_radius = config.get<float>("cloud_processing", "normal_radius");
	float correct_match_fitness_thre = config.get<float>("template_matching", "correct_match_fitness_thre");
	float overlap_dis = config.get<float>("template_matching", "overlap_dis");
	float tolerant_min_overlap = config.get<float>("template_matching", "tolerant_min_overlap");
	float heading_increment = config.get<float>("template_matching", "heading_increment");

	// 预分配结果空间
	std::vector<RoadMarking> temp_roadmarkings(scenePointClouds.size());
	std::vector<bool> valid_matches(scenePointClouds.size(), false);

	// 预清理所有点云并计算法向量
	std::vector<PCLCloudPtr> cleaned_clouds(scenePointClouds.size());

	// 设置OpenMP参数
	int num_threads = omp_get_max_threads();
	omp_set_num_threads(num_threads);
	omp_set_nested(true);
	omp_set_dynamic(false);  // 禁用动态线程数调整

	// 清理点云
	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < scenePointClouds.size(); i++)
	{
		// 清理点云
		cleaned_clouds[i].reset(new PCLCloud);
		cleaned_clouds[i]->reserve(scenePointClouds[i]->size());
		for (const auto& pt : scenePointClouds[i]->points)
		{
			if (pcl::isFinite(pt))
			{
				cleaned_clouds[i]->push_back(pt);
			}
		}
	}

	// 预先判断每个场景点云是否为直线
	std::vector<bool> is_line_cloud(scenePointClouds.size(), false);
	for (int i = 0; i < scenePointClouds.size(); i++)
	{
		if (is_line_cloud_and_get_direction(cleaned_clouds[i], roadmarkings))
		{
			is_line_cloud[i] = true;
		}
	}

	// 为每个模型创建局部变量，避免竞争
	std::vector<std::vector<float>> local_overlapping_ratios(scenePointClouds.size(), std::vector<float>(models.size()));
	std::vector<std::vector<float>> local_match_fitness(scenePointClouds.size(), std::vector<float>(models.size()));
	std::vector<std::vector<Eigen::Matrix4f>> local_transforms(scenePointClouds.size(), std::vector<Eigen::Matrix4f>(models.size()));
	std::vector<std::vector<bool>> local_valid_matches(scenePointClouds.size(), std::vector<bool>(models.size(), false));

	// 使用单层并行，避免嵌套问题
	// #pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < scenePointClouds.size() * models.size(); i++)
	{
		int scene_idx = i / models.size();
		int model_idx = i % models.size();

		// 如果场景点云是直线，跳过匹配
		if (is_line_cloud[scene_idx])
		{
			continue;
		}

		float temp_match_fitness, overlapping_ratio;
		Eigen::Matrix4f tran_mat_m2s_temp;

		// 执行匹配，得到临时匹配拟合度和变换矩阵
		if (!fpfh_ransac(models[model_idx], cleaned_clouds[scene_idx], tran_mat_m2s_temp, heading_increment, 50, 
					   correct_match_fitness_thre, temp_match_fitness, overlapping_ratio))
		{
			continue;
		}

		// 保存局部结果
		local_overlapping_ratios[scene_idx][model_idx] = overlapping_ratio;
		local_match_fitness[scene_idx][model_idx] = temp_match_fitness;
		local_transforms[scene_idx][model_idx] = tran_mat_m2s_temp;
		local_valid_matches[scene_idx][model_idx] = true;
	}

	// 在串行部分选择最佳匹配
	for (int i = 0; i < scenePointClouds.size(); i++)
	{
		float best_overlapping_ratio = tolerant_min_overlap;
		float match_fitness_best;
		Eigen::Matrix4f tran_mat_m2s_best_match;
		int best_model_index = -1;

		for (int j = 0; j < models.size(); j++)
		{
			if (local_valid_matches[i][j] && 
				local_overlapping_ratios[i][j] > best_overlapping_ratio && 
				local_match_fitness[i][j] < correct_match_fitness_thre)
			{
				best_overlapping_ratio = local_overlapping_ratios[i][j];
				match_fitness_best = local_match_fitness[i][j];
				tran_mat_m2s_best_match = local_transforms[i][j];
				best_model_index = j;
			}
		}

		// 如果找到了最佳匹配模型
		if (best_model_index >= 0)
		{
			temp_roadmarkings[i].category = best_model_index;
			temp_roadmarkings[i].accuracy = best_overlapping_ratio;
			temp_roadmarkings[i].localization_tranmat_m2s = tran_mat_m2s_best_match;
			valid_matches[i] = true;
		}
	}

	roadmarkings.reserve(roadmarkings.size()+scenePointClouds.size());
	for (size_t i = 0; i < valid_matches.size(); ++i)
	{
		if (valid_matches[i])
		{
			roadmarkings.push_back(std::move(temp_roadmarkings[i]));
		}
	}
	return true;
}

/**
 * \brief 计算两个点云之间的重叠比例（Cloud1 与 Cloud2 的重叠比例）
 * \param[in]  search_cloud : 用于重叠比计算的点云（Cloud1）
 * \param[in]  tree : KD树，作为最近邻搜索的数据结构
 * \param[in]  thre_dis : 用于估计重叠的搜索半径（距离阈值）
 * \return : 返回计算的重叠比例 [0 到 1 之间]
 */
float RoadMarkingClassifier::cal_overlap_ratio(const PCLCloudPtr& search_cloud,
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree, float thre_dis)
{
	int overlap_point_num = 0;  // 重叠点的数量
	float overlap_ratio;        // 重叠比例

	// 用于最近邻搜索的索引和距离向量
	std::vector<int> search_indices; // 存储点索引
	std::vector<float> distances_square;	// 存储点与查询点的距离

	UILog << "calculate nn\n";

	int down_rate = 3;  // 降采样率，减少计算量

	// 遍历点云中的每个点
	for (int i = 0; i < search_cloud->points.size(); i++) {
		if (i % down_rate == 0) // 降采样：每隔 down_rate 个点进行一次计算
		{
			// 对当前点执行最近邻搜索（1近邻）
			tree->nearestKSearch(search_cloud->points[i], 1, search_indices, distances_square);
			if (distances_square[0] < thre_dis * thre_dis) // 如果最近邻的距离小于阈值，则认为是重叠点
				overlap_point_num++;  // 重叠点数目增加

			// 清空临时的存储空间
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances_square);
		}
	}

	// 计算重叠比例：重叠点数与总点数的比值
	overlap_ratio = (0.001 + overlap_point_num) / (search_cloud->points.size() / down_rate + down_rate - 1);

	UILog << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio;

	return overlap_ratio;
}

/**
 * \brief 点对点的ICP（迭代最近点）算法
 * \param[in]  SourceCloud : 源点云（需要与目标点云对齐）
 * \param[in]  TargetCloud : 目标点云
 * \param[out] TransformedSource : 配准后的源点云（已变换）
 * \param[out] transformationS2T : 源点云与目标点云之间的变换矩阵（4x4）
 * \param[in]  max_iter : 最大迭代次数
 * \param[in]  use_reciprocal_correspondence : 是否使用双向最近邻（布尔值）
 * \param[in]  use_trimmed_rejector : 是否使用剔除离群点的策略（布尔值）
 * \param[in]  thre_dis : 作为重叠估计的搜索半径参数
 */
float RoadMarkingClassifier::icp_reg(const PCLCloudPtr& SourceCloud, const PCLCloudPtr& TargetCloud,
	Eigen::Matrix4f& initial_guess, Eigen::Matrix4f& transformationS2T,
	int max_iter, float thre_dis)
{
	clock_t t0, t1;
	t0 = clock();

	PCLCloudPtr tranSource(new PCLCloud);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	icp.setInputSource(SourceCloud);
	icp.setInputTarget(TargetCloud);

	icp.setMaxCorrespondenceDistance(thre_dis); // 设置最大对应距离（阈值）

	// 设置收敛标准
	icp.setMaximumIterations(max_iter);     // 最大迭代次数
	icp.setTransformationEpsilon(1e-8);     // 变换的精度（停止条件）
	icp.setEuclideanFitnessEpsilon(1e-5);   // 欧几里得误差的停止条件

	// 执行ICP配准
	icp.align(*tranSource, initial_guess);
	transformationS2T = icp.getFinalTransformation().cast<float>();

	t1 = clock();

	float fitness_score = icp.getFitnessScore();  // 返回配准后的误差（配准质量）

	// 输出配准信息
	UILog << "Point-to-Point ICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
	UILog << "The fitness score of this registration is " << icp.getFitnessScore();
	if (icp.getFitnessScore() > 5000) UILog << "The fitness score of this registration is a bit too large";

	return fitness_score;
}

// 罗德里格斯旋转公式计算旋转矩阵
Eigen::Matrix4f RodriguesMatrixTranslation(Eigen::Vector3f n, double angle)
{
	Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
	double cos_angle = cos(angle);
	double sin_angle = sin(angle);
	Eigen::Matrix3f K;

	// 计算旋转轴的反对称矩阵
	K << 0, -n(2), n(1),
		n(2), 0, -n(0),
		-n(1), n(0), 0;

	// 罗德里格斯旋转公式
	rotation_matrix.block<3, 3>(0, 0) = cos_angle * Eigen::Matrix3f::Identity() + sin_angle * K + (1 - cos_angle) * n * n.transpose();

	return rotation_matrix;
}

void RoadMarkingClassifier::align_with_PCA(const PCLCloudPtr& ModelCloud,
	const PCLCloudPtr& SceneCloud,
	Eigen::Matrix4f& transformation)
{
	// 计算模型点云的质心和协方差矩阵
	Eigen::Vector4f model_centroid;
	Eigen::Matrix3f model_covariance;
	pcl::computeMeanAndCovarianceMatrix(*ModelCloud, model_covariance, model_centroid);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> model_solver(model_covariance);
	Eigen::Matrix3f model_eigenvectors = model_solver.eigenvectors();

	// 计算场景点云的质心和协方差矩阵
	Eigen::Vector4f scene_centroid;
	Eigen::Matrix3f scene_covariance;
	pcl::computeMeanAndCovarianceMatrix(*SceneCloud, scene_covariance, scene_centroid);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> scene_solver(scene_covariance);
	Eigen::Matrix3f scene_eigenvectors = scene_solver.eigenvectors();

	// 选取主方向
	Eigen::Vector3f model_axis = model_eigenvectors.col(2);
	Eigen::Vector3f scene_axis = scene_eigenvectors.col(2);

	// 计算旋转角度，并进行范围限制
	double dot_product = model_axis.dot(scene_axis) / (model_axis.norm() * scene_axis.norm());
	dot_product = std::max(-1.0, std::min(1.0, dot_product)); // 防止 acos 计算 NaN
	double angle = acos(dot_product);

	// 计算旋转轴
	Eigen::Vector3f rotation_axis = model_axis.cross(scene_axis);
	if (rotation_axis.norm() < 1e-6) // 防止归一化失败
	{
		transformation.setIdentity();
		transformation.block<3, 1>(0, 3) = scene_centroid.head<3>() - model_centroid.head<3>();
		return;
	}
	rotation_axis.normalize();

	// 计算旋转矩阵
	Eigen::AngleAxisf rotation(angle, rotation_axis);
	Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();

	// 构造最终变换矩阵
	transformation.setIdentity();
	transformation.block<3, 3>(0, 0) = rotation_matrix;
	transformation.block<3, 1>(0, 3) = scene_centroid.head<3>() - rotation_matrix * model_centroid.head<3>();
}

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/fpfh.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <limits>

template <typename PointSource, typename PointTarget>
double calculateFitnessScore(const pcl::PointCloud<PointSource>& source_cloud,
	const pcl::PointCloud<PointTarget>& target_cloud,
	double max_range)
{
	double fitness_score = 0.0;
	int nr = 0;  // 有效内点的数量

	// 创建一个 KD-Tree，用于加速目标点云的最近邻搜索
	pcl::KdTreeFLANN<PointTarget> tree;
	tree.setInputCloud(target_cloud.makeShared());  // 将目标点云设置为 KD-Tree 的输入

	max_range *= max_range;

	// 对源点云的每个点进行遍历
	for (std::size_t i = 0; i < source_cloud.size(); ++i) {
		const auto& point = source_cloud[i];

		// 如果点是无效的（NaN 或无穷大），则跳过
		if (!pcl::isFinite(point)) {
			continue;
		}

		// 找到该点在目标点云中的最近邻
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		if (tree.nearestKSearch(point, 1, nn_indices, nn_dists) > 0) {
			// 如果最近邻点的距离小于等于最大阈值，则认为该点是内点
			if (nn_dists[0] <= max_range) {
				// 累加该点到目标点云的距离
				fitness_score += nn_dists[0];
				nr++;  // 统计有效点
			}
		}
	}

	// 如果有有效内点，则返回平均得分（平均最近邻距离）
	if (nr > 0) {
		return fitness_score / nr;
	}

	// 如果没有有效内点，则返回最大得分值（表示配准失败）
	return std::numeric_limits<double>::max();
}


// 可视化RANSAC配准过程的函数
void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& model_keypoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_keypoints,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr& model_fpfh,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr& scene_fpfh)
{
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	// 添加源点云和目标点云
	viewer.addPointCloud<pcl::PointXYZ>(model_keypoints, "model_keypoints");
	viewer.addPointCloud<pcl::PointXYZ>(scene_keypoints, "scene_keypoints");

	// 计算目标点云（scene_keypoints）的质心
	Eigen::Vector4f scene_centroid;
	pcl::compute3DCentroid(*scene_keypoints, scene_centroid);  // 计算质心
	// 将质心作为视图目标
	viewer.setCameraPosition(scene_centroid[0], scene_centroid[1], scene_centroid[2],   // 目标点的坐标
		scene_centroid[0], scene_centroid[1], scene_centroid[2] + 1.0f,   // 摄像机位置 (稍微偏移以获得视角)
		0.0, 1.0, 0.0);  // 上方向

	// 初始化 SampleConsensusPrerejective
	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
	sac.setInputSource(model_keypoints);
	sac.setInputTarget(scene_keypoints);

	sac.setSourceFeatures(model_fpfh);
	sac.setTargetFeatures(scene_fpfh);

	sac.setMaximumIterations(500);
	sac.setNumberOfSamples(3);
	sac.setCorrespondenceRandomness(3);
	sac.setSimilarityThreshold(0.8f);
	sac.setMaxCorrespondenceDistance(0.2f); // 最大匹配距离
	sac.setInlierFraction(0.6f); // 内点比例

	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>(*model_keypoints));
	pcl::PointCloud<pcl::PointXYZ>::Ptr best_aligned(new pcl::PointCloud<pcl::PointXYZ>(*model_keypoints));

	float lowest_error = std::numeric_limits<float>::max();
	float inliers_ = 0;

	std::string text1 = "Iteration: " + std::to_string(0);
	viewer.addText(text1, 10, 30, "iteration_num");

	for (int i = 0; i < 1; ++i) {
		// 执行配准步骤
		sac.align(*aligned);

		// 获取内点索引
		const pcl::Indices& inliers = sac.getInliers();



		if (inliers.size() < aligned->size() * 0.6)
		{
			text1 = "Iteration: " + std::to_string(i);
			viewer.updateText(text1, 10, 30, "iteration_num");

			if (viewer.wasStopped()) {
				break;
			}
			viewer.spinOnce(10);  // 更新可视化界面
			continue;
		}
		
		// 清除当前点云，并重新添加对齐后的点云
		viewer.removeAllPointClouds();
		viewer.removeAllShapes();
		text1 = "Iteration: " + std::to_string(i);
		viewer.addText(text1, 10, 30, "iteration_num");

		// 使用不同的颜色来显示每个点云
		viewer.addPointCloud<pcl::PointXYZ>(scene_keypoints, "scene_keypoints");
		viewer.addPointCloud<pcl::PointXYZ>(model_keypoints, "model_keypoints");  // 显示原始点云
		viewer.addPointCloud<pcl::PointXYZ>(aligned, "aligned_cloud");            // 显示已对齐的点云
		viewer.addPointCloud<pcl::PointXYZ>(best_aligned, "best_aligned_cloud");            // 显示已对齐的点云

		// 设置不同的颜色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "scene_keypoints"); // 红色表示目标点云
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "model_keypoints"); // 白色表示源点云
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "aligned_cloud"); // 蓝色表示已对齐的点云
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.843, 0.0, "best_aligned_cloud"); // 金色表示最好对齐


		// 可视化内点（绿色表示内点）
		for (size_t j = 0; j < inliers.size(); ++j) {
			// 根据内点索引可视化源点云内点
			viewer.addSphere<pcl::PointXYZ>(aligned->points[inliers[j]], 0.01, 0.0, 1.0, 0.0, "sphere_valid_" + std::to_string(j)); // 绿色球体表示有效点（内点）
		}

		// 每次迭代后进行更新
		viewer.addText("Red indicates the target point cloud\nWhite indicates the source point cloud\nBlue indicates the currently mapped aligned point cloud\nGolden indicates the best alignment", 10, 50, "introduce");

		// 更新文本框，显示迭代次数和配准得分
		std::string text = "\nScore&inliers_: " + std::to_string(sac.getFitnessScore()) + " \t" + std::to_string(1.0f * inliers.size() / aligned->size()) + "\nlowest_error&inliers_:  " + std::to_string(lowest_error)+ " \t" + std::to_string(inliers_);
		viewer.addText(text, 10, 10, "iteration_text");

		if (viewer.wasStopped()) {
			break;
		}
		// 更新可视化界面
		viewer.spinOnce(2);  // 更新可视化界面
		
		// 因为align会init，所以保存迭代后的状态(手动模拟比较最低得分的迭代)
		double score = calculateFitnessScore(*aligned, *scene_keypoints, 0.1f);

		if (score < lowest_error) {
			lowest_error = score;
			inliers_ = 1.0f * inliers.size() / aligned->size();
			*best_aligned = *aligned;
			sac.setInputSource(aligned);
		}
	}
	viewer.spin();
}


float RoadMarkingClassifier::fpfh_ransac(const Model& model, const PCLCloudPtr& sceneCloud,
	Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre,
	float& match_fitness, float& overlapping_ratio) {

	max_iter_num = 50;
	// 1. 体素下采样
	float voxel_size = 0.1f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(voxel_size, voxel_size, voxel_size);

	vg.setInputCloud(model.raw_point_cloud);
	vg.filter(*model_downsampled);
	vg.setInputCloud(sceneCloud);
	vg.filter(*scene_downsampled);

	// 2. Alpha Shape 提取边界点
	auto extract_alpha_shape = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float alpha)
		-> pcl::PointCloud<pcl::PointXYZ>::Ptr
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConcaveHull<pcl::PointXYZ> chull;
		chull.setInputCloud(cloud);
		chull.setAlpha(alpha);
		chull.setKeepInformation(true); // 保留边界点
		chull.reconstruct(*boundary);
		return boundary;
	};

	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = extract_alpha_shape(model_downsampled, 0.1f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = extract_alpha_shape(scene_downsampled, 0.1f);

	if (model_keypoints->empty() || scene_keypoints->empty()) {
		return false;
	}

	// 2.5 粗对齐
	{
		// 1. 使用PCA进行初步对齐
		Eigen::Matrix4f initial_transformation;
		align_with_PCA(model_downsampled, scene_downsampled, initial_transformation);

		// 2. 使用ICP进行精细匹配
		Eigen::Matrix4f final_transformation;
		// 使用icp_reg进行精细匹配，传递必要的参数（初始变换、最大迭代次数、距离阈值等）
		float fitness_score = icp_reg(model_keypoints, scene_keypoints, initial_transformation, final_transformation,
			max_iter_num, dis_thre);

		tran_mat_m2s_best = final_transformation;

		//{
		//	// 对模型点云进行变换
		//	pcl::PointCloud<pcl::PointXYZ>::Ptr model_downsampled_tran(new pcl::PointCloud<pcl::PointXYZ>);
		//	pcl::transformPointCloud(*model_downsampled, *model_downsampled_tran, tran_mat_m2s_best);

		//	// 可视化
		//	visualizePointClouds("预对齐", {}, scene_downsampled, model_downsampled_tran);
		//}
	}

	// 3. 计算法向量
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	//auto getXYNormal = [&](PCLCloudPtr cloud) {
	//	// 创建新的点云集合，给每个点云加上多个不同Z值的"层"
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLayered(new pcl::PointCloud<pcl::PointXYZ>());
	//	cloudLayered->reserve(cloud->points.size() * 15);  // 生成三个层次

	//	// 创建多个不同Z值的"层"
	//	for (float z_offset = -0.1f; z_offset <= 0.1f; z_offset += 0.02f) {
	//		if (fabs(z_offset - 0.0f) < 0.01)continue;
	//		for (const auto& point : cloud->points) {
	//			pcl::PointXYZ new_point = point;
	//			new_point.z += z_offset;  // 在Z轴上增加不同的偏移量
	//			cloudLayered->push_back(new_point);
	//		}
	//	}

	//	ne.setInputCloud(cloudLayered);
	//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//	ne.compute(*normals);

	//	normals->points.resize(cloud->points.size());
	//	return normals;
	//};

	//ne.setSearchMethod(tree);
	//ne.setRadiusSearch(0.2);

	//pcl::PointCloud<pcl::Normal>::Ptr model_normals = getXYNormal(model_keypoints);
	//pcl::PointCloud<pcl::Normal>::Ptr scene_normals = getXYNormal(scene_keypoints);

	pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);
	CloudProcess::computeLocalNormal2D(model_keypoints, 0.3, model_normals);
	CloudProcess::computeLocalNormal2D(scene_keypoints, 0.3, scene_normals);

	// 4. 计算 FPFH 特征描述子
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(model_keypoints);
	fpfh.setInputNormals(model_normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	fpfh.setSearchMethod(tree);
	fpfh.setRadiusSearch(0.1);
	fpfh.compute(*model_fpfh);

	fpfh.setInputCloud(scene_keypoints);
	fpfh.setInputNormals(scene_normals);
	fpfh.compute(*scene_fpfh);

	// 5. 粗配准 - RANSAC
	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
	sac.setInputSource(model_keypoints);
	sac.setInputTarget(scene_keypoints);

	sac.setSourceFeatures(model_fpfh);
	sac.setTargetFeatures(scene_fpfh);

	sac.setMaximumIterations(5000);
	sac.setNumberOfSamples(3);
	sac.setCorrespondenceRandomness(3);
	sac.setSimilarityThreshold(0.8f);
	sac.setMaxCorrespondenceDistance(0.1f);
	sac.setInlierFraction(0.6f);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	sac.align(*sac_aligned, tran_mat_m2s_best);

	if (!sac.hasConverged()) {
		return false;
	}

	Eigen::Matrix4f init_transformation = sac.getFinalTransformation();

	//{
	//	visualizePointCloudWithNormals("model点云二维法向量", model_keypoints, model_normals);
	//	visualizePointCloudWithNormals("scene点云二维法向量", scene_keypoints, scene_normals);

	//	// 对模型点云进行变换
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints_tran(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::transformPointCloud(*model_keypoints, *model_keypoints_tran, init_transformation);

	//	// 可视化
	//	visualizePointClouds("粗配准", {}, scene_keypoints, model_keypoints_tran);
	//}

	// 6. 精配准 - ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(model_downsampled);
	icp.setInputTarget(scene_downsampled);
	icp.setMaxCorrespondenceDistance(dis_thre);
	icp.setMaximumIterations(max_iter_num);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-6);

	pcl::PointCloud<pcl::PointXYZ> icp_result;
	icp.align(icp_result, init_transformation);

	if (!icp.hasConverged()) {
		return false;
	}

	tran_mat_m2s_best = icp.getFinalTransformation();

	//{
	//	// 对模型点云进行变换
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr model_downsampled_tran(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::transformPointCloud(*model_downsampled, *model_downsampled_tran, tran_mat_m2s_best);

	//	// 可视化
	//	visualizePointClouds("精配准", {}, scene_downsampled, model_downsampled_tran);
	//}

	match_fitness = static_cast<float>(icp.getFitnessScore());
	overlapping_ratio = 1.0f - match_fitness;

	return true;
}


#include <pcl/keypoints/harris_3d.h> // 包含 Harris 角点检测器
float RoadMarkingClassifier::harris_fpfh_ransac(const Model& model, const PCLCloudPtr& sceneCloud,
	Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre,
	float& match_fitness, float& overlapping_ratio)
{
	//max_iter_num = 50;
	//// 1. 体素下采样
	//float voxel_size = 0.1f;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr model_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::VoxelGrid<pcl::PointXYZ> vg;
	//vg.setLeafSize(voxel_size, voxel_size, voxel_size);

	//vg.setInputCloud(model.raw_point_cloud);
	//vg.filter(*model_downsampled);
	//vg.setInputCloud(sceneCloud);
	//vg.filter(*scene_downsampled);

	//// 2. Alpha Shape 提取边界点
	//auto extract_alpha_shape = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float alpha)
	//	-> pcl::PointCloud<pcl::PointXYZ>::Ptr
	//{
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::ConcaveHull<pcl::PointXYZ> chull;
	//	chull.setInputCloud(cloud);
	//	chull.setAlpha(alpha);
	//	chull.setKeepInformation(true); // 保留边界点
	//	chull.reconstruct(*boundary);
	//	return boundary;
	//};

	//pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = extract_alpha_shape(model_downsampled, 0.1f);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = extract_alpha_shape(scene_downsampled, 0.1f);

	//if (model_keypoints->empty() || scene_keypoints->empty()) {
	//	return false;
	//}

	//// 2.5 粗对齐
	//{
	//	// 1. 使用PCA进行初步对齐
	//	Eigen::Matrix4f initial_transformation;
	//	align_with_PCA(model_downsampled, scene_downsampled, initial_transformation);

	//	// 2. 使用ICP进行精细匹配
	//	Eigen::Matrix4f final_transformation;
	//	// 使用icp_reg进行精细匹配，传递必要的参数（初始变换、最大迭代次数、距离阈值等）
	//	float fitness_score = icp_reg(model_keypoints, scene_keypoints, initial_transformation, final_transformation,
	//		max_iter_num, dis_thre);

	//	tran_mat_m2s_best = final_transformation;
	//}

	//pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);
	//CloudProcess::computeLocalNormal2D(model_keypoints, 0.3, model_normals);
	//CloudProcess::computeLocalNormal2D(scene_keypoints, 0.3, scene_normals);

	//// 3.5 使用Harris检测关键点并计算FPFH特征
	//pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> harris_detector;
	//pcl::PointCloud<pcl::PointXYZI>::Ptr model_harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::PointCloud<pcl::PointXYZI>::Ptr scene_harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>);

	//// 将XYZ点云转换为XYZI点云（添加强度信息）
	//pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::PointCloud<pcl::PointXYZI>::Ptr scene_keypoints_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
	//
	//// 转换模型关键点，基于法向量变化计算强度
	//model_keypoints_xyzi->points.reserve(model_keypoints->points.size());
	//for (size_t i = 0; i < model_keypoints->points.size(); ++i) {
	//	pcl::PointXYZI pt_xyzi;
	//	pt_xyzi.x = model_keypoints->points[i].x;
	//	pt_xyzi.y = model_keypoints->points[i].y;
	//	pt_xyzi.z = model_keypoints->points[i].z;
	//	
	//	// 基于法向量的曲率计算强度值
	//	if (i < model_normals->points.size()) {
	//		pt_xyzi.intensity = std::abs(model_normals->points[i].curvature);
	//	} else {
	//		pt_xyzi.intensity = 1.0f; // 默认强度值
	//	}
	//	model_keypoints_xyzi->points.push_back(pt_xyzi);
	//}
	//model_keypoints_xyzi->width = model_keypoints_xyzi->points.size();
	//model_keypoints_xyzi->height = 1;
	//model_keypoints_xyzi->is_dense = true;

	//// 转换场景关键点，基于法向量变化计算强度
	//scene_keypoints_xyzi->points.reserve(scene_keypoints->points.size());
	//for (size_t i = 0; i < scene_keypoints->points.size(); ++i) {
	//	pcl::PointXYZI pt_xyzi;
	//	pt_xyzi.x = scene_keypoints->points[i].x;
	//	pt_xyzi.y = scene_keypoints->points[i].y;
	//	pt_xyzi.z = scene_keypoints->points[i].z;
	//	
	//	// 基于法向量的曲率计算强度值
	//	if (i < scene_normals->points.size()) {
	//		pt_xyzi.intensity = std::abs(scene_normals->points[i].curvature);
	//	} else {
	//		pt_xyzi.intensity = 1.0f; // 默认强度值
	//	}
	//	scene_keypoints_xyzi->points.push_back(pt_xyzi);
	//}
	//scene_keypoints_xyzi->width = scene_keypoints_xyzi->points.size();
	//scene_keypoints_xyzi->height = 1;
	//scene_keypoints_xyzi->is_dense = true;

	//// 设置Harris参数
	//double model_resolution = 0.1;  // 模型点云分辨率
	//double scene_resolution = 0.1;  // 场景点云分辨率

	//// 计算模型点云的Harris关键点
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr model_tree(new pcl::search::KdTree<pcl::PointXYZI>());
	//harris_detector.setSearchMethod(model_tree);
	//harris_detector.setRadius(6 * model_resolution);  // 搜索半径
	//harris_detector.setRadiusSearch(6 * model_resolution);  // 搜索半径
	//harris_detector.setThreshold(0.01);  // Harris响应阈值
	//harris_detector.setNonMaxSupression(true);  // 启用非极大值抑制
	//harris_detector.setRefine(true);  // 启用细化
	//harris_detector.setNumberOfThreads(4);  // 线程数
	//harris_detector.setInputCloud(model_keypoints_xyzi);
	//harris_detector.compute(*model_harris_keypoints);

	//// 获取模型关键点的索引并提取法向量
	//auto model_keypoint_indices = harris_detector.getKeypointsIndices();
	//pcl::PointCloud<pcl::Normal>::Ptr model_harris_normals(new pcl::PointCloud<pcl::Normal>);
	//model_harris_normals->points.resize(model_harris_keypoints->points.size());
	//for (size_t i = 0; i < model_keypoint_indices->indices.size(); ++i)
	//{
	//	model_harris_normals->points[i] = model_normals->points[model_keypoint_indices->indices[i]];
	//}

	//// 将Harris关键点转换回XYZ格式用于FPFH计算
	//pcl::PointCloud<pcl::PointXYZ>::Ptr model_harris_keypoints_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//model_harris_keypoints_xyz->points.reserve(model_harris_keypoints->points.size());
	//for (const auto& pt : model_harris_keypoints->points) {
	//	pcl::PointXYZ pt_xyz;
	//	pt_xyz.x = pt.x;
	//	pt_xyz.y = pt.y;
	//	pt_xyz.z = pt.z;
	//	model_harris_keypoints_xyz->points.push_back(pt_xyz);
	//}
	//model_harris_keypoints_xyz->width = model_harris_keypoints_xyz->points.size();
	//model_harris_keypoints_xyz->height = 1;
	//model_harris_keypoints_xyz->is_dense = true;

	//// 计算模型关键点的FPFH特征
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	//pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	//fpfh.setInputCloud(model_harris_keypoints_xyz);
	//fpfh.setInputNormals(model_harris_normals);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//fpfh.setSearchMethod(tree);
	//fpfh.setRadiusSearch(0.1);
	//fpfh.compute(*model_fpfh);

	//// 计算场景点云的Harris关键点
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr scene_tree(new pcl::search::KdTree<pcl::PointXYZI>());
	//harris_detector.setSearchMethod(scene_tree);
	//harris_detector.setRadius(4 * scene_resolution);  // 搜索半径
	//harris_detector.setRadiusSearch(4 * scene_resolution);  // 搜索半径
	//harris_detector.setInputCloud(scene_keypoints_xyzi);
	//harris_detector.compute(*scene_harris_keypoints);

	//// 获取场景关键点的索引并提取法向量
	//auto scene_keypoint_indices = harris_detector.getKeypointsIndices();
	//pcl::PointCloud<pcl::Normal>::Ptr scene_harris_normals(new pcl::PointCloud<pcl::Normal>);
	//scene_harris_normals->points.resize(scene_harris_keypoints->points.size());
	//for (size_t i = 0; i < scene_keypoint_indices->indices.size(); ++i) {
	//	scene_harris_normals->points[i] = scene_normals->points[scene_keypoint_indices->indices[i]];
	//}

	//// 将场景Harris关键点转换回XYZ格式用于FPFH计算
	//pcl::PointCloud<pcl::PointXYZ>::Ptr scene_harris_keypoints_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//scene_harris_keypoints_xyz->points.reserve(scene_harris_keypoints->points.size());
	//for (const auto& pt : scene_harris_keypoints->points) {
	//	pcl::PointXYZ pt_xyz;
	//	pt_xyz.x = pt.x;
	//	pt_xyz.y = pt.y;
	//	pt_xyz.z = pt.z;
	//	scene_harris_keypoints_xyz->points.push_back(pt_xyz);
	//}
	//scene_harris_keypoints_xyz->width = scene_harris_keypoints_xyz->points.size();
	//scene_harris_keypoints_xyz->height = 1;
	//scene_harris_keypoints_xyz->is_dense = true;

	//// 计算场景关键点的FPFH特征
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	//fpfh.setInputCloud(scene_harris_keypoints_xyz);
	//fpfh.setInputNormals(scene_harris_normals);
	//fpfh.compute(*scene_fpfh);


	//{
	//	PCLCloudPtr model(new PCLCloud);
	//	PCLCloudPtr scene(new PCLCloud);
	//	for (size_t i = 0; i < model_keypoint_indices->indices.size(); ++i)
	//	{
	//		model->push_back(model_keypoints->points[model_keypoint_indices->indices[i]]);
	//	}
	//	for (size_t i = 0; i < scene_keypoint_indices->indices.size(); ++i)
	//	{
	//		scene->push_back(scene_keypoints->points[scene_keypoint_indices->indices[i]]);
	//	}

	//	visualizePointCloudWithNormals("model点云二维法向量", model, model_harris_normals);
	//	visualizePointCloudWithNormals("scene点云二维法向量", scene, scene_normals);
	//}


	//// 5. 粗配准 - RANSAC
	//pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
	//sac.setInputSource(model_harris_keypoints_xyz);
	//sac.setInputTarget(scene_harris_keypoints_xyz);

	//sac.setSourceFeatures(model_fpfh);
	//sac.setTargetFeatures(scene_fpfh);

	//sac.setMaximumIterations(5000);
	//sac.setNumberOfSamples(3);
	//sac.setCorrespondenceRandomness(3);
	//sac.setSimilarityThreshold(0.8f);
	//sac.setMaxCorrespondenceDistance(0.1f);
	//sac.setInlierFraction(0.6f);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr sac_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	//sac.align(*sac_aligned, tran_mat_m2s_best);

	//if (!sac.hasConverged())
	//{
	//	return false;
	//}

	//Eigen::Matrix4f init_transformation = sac.getFinalTransformation();

	//// 6. 精配准 - ICP
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//icp.setInputSource(model_downsampled);
	//icp.setInputTarget(scene_downsampled);
	//icp.setMaxCorrespondenceDistance(dis_thre);
	//icp.setMaximumIterations(max_iter_num);
	//icp.setTransformationEpsilon(1e-8);
	//icp.setEuclideanFitnessEpsilon(1e-6);

	//pcl::PointCloud<pcl::PointXYZ> icp_result;
	//icp.align(icp_result, init_transformation);

	//if (!icp.hasConverged()) {
	//	return false;
	//}

	//tran_mat_m2s_best = icp.getFinalTransformation();

	//match_fitness = static_cast<float>(icp.getFitnessScore());
	//overlapping_ratio = 1.0f - match_fitness;

	return true;
}


PCLCloudPtr RoadMarkingClassifier::get_hull_cloud(PCLCloudPtr cloud)
{
	double alpha = 0.05;
	PCLCloudPtr hullCloud(new PCLCloud);
	// 计算 Concave Hull（凹包）
	pcl::ConcaveHull<PCLPoint> concaveHull;
	concaveHull.setInputCloud(cloud);
	concaveHull.setAlpha(alpha); // α 值越小，轮廓越紧
	concaveHull.reconstruct(*hullCloud);
	// 如果点数不足，尝试使用 Convex Hull（凸包）
	if (hullCloud->size() < 3)
	{
		pcl::ConvexHull<PCLPoint> convexHull;
		convexHull.setInputCloud(cloud);
		convexHull.reconstruct(*hullCloud);
	}
	return hullCloud;
}

void RoadMarkingClassifier::align_cloud_to_x_axis(const PCLCloudPtr& cloud, Eigen::Matrix4f& transformation)
{
	if (!cloud || cloud->empty())
	{
		std::cerr << "输入点云为空！" << std::endl;
		transformation.setIdentity();
		return;
	}

	// 1. 计算点云质心和协方差矩阵
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariance;
	pcl::computeMeanAndCovarianceMatrix(*cloud, covariance, centroid);

	// 2. 利用自伴特征分解得到 PCA 主成分
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
	// Eigen 的自伴特征分解特征值按升序排列，这里取最大方差对应的主成分，即最后一列
	Eigen::Vector3f principal_axis = eigen_vectors.col(2).normalized();

	// 3. 定义目标方向为 X 轴
	Eigen::Vector3f x_axis(1.0f, 0.0f, 0.0f);

	// 4. 计算主成分与 X 轴之间的夹角
	double angle = acos(principal_axis.dot(x_axis)); // 两向量均为单位向量
	// 5. 计算旋转轴（叉乘得到旋转轴）
	Eigen::Vector3f rotation_axis = principal_axis.cross(x_axis);
	if (rotation_axis.norm() < 1e-6)
	{
		// 如果已经对齐或反向（旋转轴无法确定），则选择任意正交轴，这里选择 Z 轴
		rotation_axis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
	}
	rotation_axis.normalize();

	// 6. 计算旋转矩阵（齐次矩阵）
	Eigen::Matrix4f R = RodriguesMatrixTranslation(rotation_axis, angle);

	// 7. 构造平移矩阵，将点云平移到原点，再旋转，最后平移回质心位置
	Eigen::Matrix4f T_translate_to_origin = Eigen::Matrix4f::Identity();
	T_translate_to_origin.block<3, 1>(0, 3) = -centroid.head<3>();

	Eigen::Matrix4f T_translate_back = Eigen::Matrix4f::Identity();
	T_translate_back.block<3, 1>(0, 3) = centroid.head<3>();

	// 8. 组合变换：先平移到原点，再旋转，再平移回原质心位置
	transformation = T_translate_back * R * T_translate_to_origin;
}

bool RoadMarkingClassifier::is_line_cloud_and_get_direction(PCLCloudPtr cloud, RoadMarkings& roadmarkings)
{
	// 1. 计算将点云主方向对齐到 X 轴的变换矩阵
	Eigen::Matrix4f T;
	align_cloud_to_x_axis(cloud, T);

	// 2. 将点云旋转到主成分坐标系（X 轴对齐）
	PCLCloudPtr rotated_cloud(new PCLCloud);
	pcl::transformPointCloud(*cloud, *rotated_cloud, T);

	// 3. 计算旋转后点云的轴对齐包围盒（仅考虑 XY 平面）
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*rotated_cloud, minPt, maxPt);

	float dx = maxPt.x - minPt.x;
	float dy = maxPt.y - minPt.y;
	float length = std::max(dx, dy);
	float width = std::min(dx, dy);

	// 4. 判断是否为线形点云（宽度小于 0.35，长度大于 0.8）
	if (width < 0.35f && length > 0.8f)
	{
		// 新建一个 RoadMarking 条目
		roadmarkings.emplace_back();
		RoadMarking& rm = roadmarkings.back();
		rm.category = SIDE_LINE;  // 标记为线形

		// 5. 计算包围盒在旋转空间中的四个顶点（Z 取平均值）
		float avg_z = (minPt.z + maxPt.z) * 0.5f;
		Eigen::Vector4f c1(minPt.x, minPt.y, avg_z, 1.0f); // 左下
		Eigen::Vector4f c2(minPt.x, maxPt.y, avg_z, 1.0f); // 左上
		Eigen::Vector4f c3(maxPt.x, maxPt.y, avg_z, 1.0f); // 右上
		Eigen::Vector4f c4(maxPt.x, minPt.y, avg_z, 1.0f); // 右下

		// 6. 反变换到原始坐标系
		Eigen::Matrix4f T_inv = T.inverse();
		Eigen::Vector4f o1 = T_inv * c1;
		Eigen::Vector4f o2 = T_inv * c2;
		Eigen::Vector4f o3 = T_inv * c3;
		Eigen::Vector4f o4 = T_inv * c4;

		// 7. 计算中心线的两个端点（沿 Y 方向中线）
		Eigen::Vector4f mid_bottom = (o1 + o2) * 0.5f;
		Eigen::Vector4f mid_top = (o3 + o4) * 0.5f;

		// 8. 将方向向量和这一条"直线折线"保存到 rm 中
		rm.direction = (mid_top - mid_bottom).head<4>(); // 四维向量

		// 这里把两个端点作为一条折线，插入到 polylines 列表里
		std::vector<pcl::PointXYZ> oneLine;
		oneLine.emplace_back(pcl::PointXYZ(mid_bottom.x(), mid_bottom.y(), mid_bottom.z()));
		oneLine.emplace_back(pcl::PointXYZ(mid_top.x(), mid_top.y(), mid_top.z()));

		rm.polylines.clear();
		rm.polylines.push_back(std::move(oneLine));

		rm.name = "sideline";

		rm.accuracy = 1;

		return true;
	}
	else
	{
		return false;
	}
}

void RoadMarkingClassifier::combine_side_lines(const RoadMarkings& roadmarkings,
	double combine_length,
	RoadMarkings& combine_sideline_markings)
{
	// 存储所有侧线段的起始和结束点（来自 polylines）
	std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> sidelines;
	std::vector<Eigen::Vector3f> direction;
	std::vector<int> sidelines_index_in_roadmarkings;  // 存储每条线段在原始 roadmarkings 中的索引

	// 1. 遍历所有 roadmarkings，找出 category == SIDE_LINE 的那些标记
	for (int i = 0; i < static_cast<int>(roadmarkings.size()); ++i)
	{
		const RoadMarking& rm = roadmarkings[i];
		if (rm.category == SIDE_LINE)
		{
			// 如果一个 RoadMarking 包含多条折线，这里假设只取第一条折线进行合并。
			// 如果你需要把同一个 rm.polylines 中的所有折线都提取出来，可以再加一个内循环：
			//
			//    for (const auto& oneLine : rm.polylines) { …相同逻辑… }
			//
			// 但通常 SIDE_LINE 在前面 vectorize 阶段已经只存了一条中线折线在 polylines[0]。
			if (!rm.polylines.empty() && rm.polylines[0].size() >= 2)
			{
				// 取出这一条折线的前两个点，作为 "一个线段"
				const pcl::PointXYZ& p0 = rm.polylines[0][0];
				const pcl::PointXYZ& p1 = rm.polylines[0][1];

				sidelines.emplace_back(p0, p1);
				direction.push_back(rm.direction.head<3>()); // 只用前三维表示方向
				sidelines_index_in_roadmarkings.push_back(i);
			}
		}
	}

	int sideline_number = static_cast<int>(sidelines.size());
	if (sideline_number == 0)
	{
		// 如果没有任何 SIDE_LINE，直接返回空
		return;
	}

	// 用于存储最终合并后的多段折线（每个元素本身就是一条由若干 PCLPoint 组成的折线）
	std::vector<std::vector<pcl::PointXYZ>> combinelines;
	std::vector<bool> line_used(sideline_number, false);

	// 计算两点间距离平方的 lambda
	auto len2 = [](const pcl::PointXYZ& a, const pcl::PointXYZ& b)
	{
		return std::pow(b.x - a.x, 2) +
			std::pow(b.y - a.y, 2) +
			std::pow(b.z - a.z, 2);
	};

	// 计算两向量夹角的 lambda（返回弧度）
	auto calculateAngleBetweenVectors = [](const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
	{
		double dotProduct = v1.dot(v2);
		double magnitude1 = v1.norm();
		double magnitude2 = v2.norm();
		double cosTheta = dotProduct / (magnitude1 * magnitude2);
		// 数值安全：裁剪 cosTheta 到 [-1,1]
		cosTheta = std::min(1.0, std::max(-1.0, cosTheta));
		return std::acos(cosTheta);
	};

	const double threshold_length2 = combine_length * combine_length;
	const double threshold_angle = 5.0 * M_PI / 180.0; // 5° 的阈值

	// 2. DFS 搜索函数：从当前线段 pos 的某端开始，沿前向（is_forward=true）或后向（is_forward=false）扩展
	std::function<void(int pos, bool is_r, bool is_forward)> dfs =
		[&](int pos, bool is_r, bool is_forward)
	{
		// is_r == false 表示取 sidelines[pos].first 作为当前端点，
		// is_r == true  表示取 sidelines[pos].second 作为当前端点。
		pcl::PointXYZ point = (is_r ? sidelines[pos].second : sidelines[pos].first);

		// 如果是"前向搜索"，则把当前点加到 combinelines.back() 的尾部
		if (is_forward)
		{
			combinelines.back().push_back(point);
		}

		// 遍历所有线段，寻找与当前端点相连且方向一致的下一个线段
		for (int i = 0; i < sideline_number; ++i)
		{
			if (i == pos || line_used[i])
				continue;

			// 检查：下一个线段的哪个端点与当前 point 距离在阈值以内
			bool next_is_r = false;
			double  d0 = len2(point, sidelines[i].first);
			double  d1 = len2(point, sidelines[i].second);

			// 只有当距离平方 < threshold_length2 时，才进一步判断方向
			if (d0 < threshold_length2)
			{
				next_is_r = false; // 用 sidelines[i].first 作为连接点
			}
			else if (d1 < threshold_length2)
			{
				next_is_r = true; // 用 sidelines[i].second 作为连接点
			}
			else
			{
				continue; // 两个端点都太远，跳过
			}

			// 计算 v_curr：如果 is_r=true，说明我们是从 sidelines[pos].second 出发，
			// 那么当前方向向量用 direction[pos]；否则取 -direction[pos]
			Eigen::Vector3f v_curr = is_r ? direction[pos] : -direction[pos];

			// 计算 v_next：如果 next_is_r=false，说明用 sidelines[i].first 作为连接点，
			// 那么下一个线段的方向起点应当是 first，方向向量要用 direction[i]
			// 如果 next_is_r=true，则方向向量需取 -direction[i]
			Eigen::Vector3f v_to_next;     // 从 point 指向下一个线段的连接端
			Eigen::Vector3f v_next_dir;    // 下一个线段指向其第二个端点的方向

			if (!next_is_r)
			{
				// 连接到 sidelines[i].first
				v_to_next = Eigen::Vector3f(
					sidelines[i].first.x - point.x,
					sidelines[i].first.y - point.y,
					sidelines[i].first.z - point.z);
				v_next_dir = direction[i]; // 从 first 指向 second
			}
			else
			{
				// 连接到 sidelines[i].second
				v_to_next = Eigen::Vector3f(
					sidelines[i].second.x - point.x,
					sidelines[i].second.y - point.y,
					sidelines[i].second.z - point.z);
				v_next_dir = -direction[i]; // 从 second 指向 first
			}

			// 判断三个方向夹角是否都在阈值以内：v_curr 与 v_to_next、v_to_next 与 v_next_dir、v_curr 与 v_next_dir
			double ang1 = calculateAngleBetweenVectors(v_curr, v_to_next);
			double ang2 = calculateAngleBetweenVectors(v_to_next, v_next_dir);
			double ang3 = calculateAngleBetweenVectors(v_curr, v_next_dir);

			if (ang1 < threshold_angle && ang2 < threshold_angle && ang3 < threshold_angle)
			{
				// 符合方向连续性，标记 i 为已使用
				line_used[i] = true;

				// 如果是"前向"，先把下一个线段的连接端 push 到尾部
				if (is_forward)
				{
					if (!next_is_r)
						combinelines.back().push_back(sidelines[i].first);
					else
						combinelines.back().push_back(sidelines[i].second);
				}

				// 继续 DFS，下一个线段成为新的 pos，端点由 next_is_r 决定
				dfs(i, next_is_r, is_forward);

				// 如果是"后向"（is_forward == false），则在回溯时把连接端 push 到尾部
				if (!is_forward)
				{
					if (!next_is_r)
						combinelines.back().push_back(sidelines[i].first);
					else
						combinelines.back().push_back(sidelines[i].second);
				}

				// 找到并处理一个满足条件的线段后，就跳出 for 循环（只取一条连续分支）
				break;
			}
		}

		// 如果是"后向"搜索，当所有可扩展分支处理完后，把当前端点 point push 到尾部
		if (!is_forward)
		{
			combinelines.back().push_back(point);
		}
	};

	// 3. 对每条未被使用的侧线段调用 dfs，生成两端的延伸结果
	for (int i = 0; i < sideline_number; ++i)
	{
		if (!line_used[i])
		{
			// 为每个新的组合折线申请一个空的向量
			combinelines.emplace_back();
			// 先"后向"搜一遍，填充 combinelines.back() 的前半部分
			dfs(i, /*is_r=*/false, /*is_forward=*/false);
			// 再"前向"搜一遍，填充 combinelines.back() 的后半部分
			dfs(i, /*is_r=*/true,  /*is_forward=*/true);
		}
	}

	// 4. 将 combinelines 转换为最终的 combine_sideline_markings（带类别标记 COMBINE_SIDE_LINES）
	combine_sideline_markings.clear();
	combine_sideline_markings.resize(static_cast<int>(combinelines.size()));

	for (int i = 0; i < static_cast<int>(combinelines.size()); ++i)
	{
		RoadMarking& rm_comb = combine_sideline_markings[i];
		rm_comb.name = "sideline";
		rm_comb.accuracy = 1;
		rm_comb.category = COMBINE_SIDE_LINES;

		// 把一条组合好的折线插入到 rm_comb.polylines 中
		rm_comb.polylines.clear();
		rm_comb.polylines.push_back(std::move(combinelines[i]));
	}
}
