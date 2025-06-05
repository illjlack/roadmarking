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

using namespace std;
#include <pcl/visualization/pcl_visualizer.h>
#include <nlohmann/json.hpp>
#include <fstream>

enum ROADMARKING_TYPE
{
	SIDE_LINE = 100001,
	COMBINE_SIDE_LINES = 100002
};


using namespace roadmarking;
using json = nlohmann::json;

inline int l(int pos)
{
	return pos << 1;
}

inline int r(int pos)
{
	return pos << 1 | 1;
}

class Model
{
public:
	// 成员变量
	std::string outline_point_cloud_path;  // 轮廓点云
	std::string raw_point_cloud_path;      // 原始点云路径
	std::vector<std::vector<PCLPoint>> vectorized_polylines;  // 向量化点的列表

	std::string name;

	PCLCloudPtr outline_point_cloud;
	PCLCloudPtr raw_point_cloud;

	// 主成分分析结果
	Eigen::Vector3f raw_pca_values;      // 原始点云的特征值
	Eigen::Matrix3f raw_pca_matrix;      // 原始点云的特征向量矩阵

	Eigen::Vector3f centroid;			 // 质心
	Eigen::Vector4f min_bounding_box;    // 最小旋转包围盒（xmin, xmax, ymin, ymax）
	// 构造函数
	Model(const std::string& name,
		const std::string& outline_path,
		const std::string& raw_path,
		const std::vector<std::vector<PCLPoint>>& polylines)
		: name(name)
		, outline_point_cloud_path(outline_path)
		, raw_point_cloud_path(raw_path)
		, vectorized_polylines(polylines)
	{
		// 加载轮廓点云
		outline_point_cloud = loadPointCloud(outline_path);
		// 加载原始点云
		raw_point_cloud = loadPointCloud(raw_path);

		if (!outline_point_cloud || outline_point_cloud->empty()
			|| !raw_point_cloud || raw_point_cloud->empty())
		{
			// 如果任一条点云加载失败，则直接返回
			return;
		}

		calculatePCA();
		calculateCentroid();
		calculateBoundingBox();
	}

	static std::vector<Model> loadFromJson(const std::string& filename)
	{
		std::ifstream input(filename);
		if (!input.is_open()) {
			std::cerr << "Could not open file " << filename << std::endl;
			return {};
		}

		json j;
		input >> j;

		std::vector<Model> models;
		// 顶层键名应为 "models"（注意与旧代码中 "model" 不同）
		if (!j.contains("models") || !j["models"].is_array()) {
			std::cerr << "Invalid JSON format: no \"models\" array found." << std::endl;
			return {};
		}

		// 遍历 JSON 中的每一个 model 对象
		for (const auto& model_data : j["models"])
		{
			// 读取基本字段
			std::string name = model_data.value("name", "");
			std::string outline_path = model_data.value("outline_point_cloud_path", "");
			std::string raw_path = model_data.value("raw_point_cloud_path", "");

			// 用于存储该 model 对应的多条折线
			std::vector<std::vector<PCLPoint>> polylines;

			// 如果存在 "graph_elements"，就逐条解析
			if (model_data.contains("graph_elements") && model_data["graph_elements"].is_array())
			{
				for (const auto& elem : model_data["graph_elements"])
				{
					// 只处理 type == "polyline" 的元素
					if (elem.value("type", "") == "polyline"
						&& elem.contains("points") && elem["points"].is_array())
					{
						std::vector<PCLPoint> onePolyline;
						for (const auto& ptArr : elem["points"])
						{
							// 每个 ptArr 应该是 [x, y, z] 这样的数组
							if (ptArr.is_array() && ptArr.size() == 3)
							{
								float x = ptArr[0].get<float>();
								float y = ptArr[1].get<float>();
								float z = ptArr[2].get<float>();
								onePolyline.emplace_back(x, y, z);
							}
						}
						// 将这一条折线加入到 polylines 中
						polylines.push_back(std::move(onePolyline));
					}
				}
			}

			// 创建 Model 对象并加入到返回列表
			models.emplace_back(name, outline_path, raw_path, polylines);
		}

		return models;
	}

	static PCLCloudPtr loadPointCloud(const std::string& file_path) {
		PCLCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
			std::cerr << "Couldn't read file " << file_path << std::endl;
			return nullptr;
		}

		// 清除源点云中的 NaN Inf点
		PCLCloudPtr cleaned_cloud(new PCLCloud);
		cleaned_cloud->reserve(cloud->size());
		for (const auto& pt : cloud->points)
		{
			if (pcl::isFinite(pt))  // PCL 提供的检查点是否是有限数（非 NaN 且非 Inf）
			{
				cleaned_cloud->push_back(pt);
			}
		}

		return cleaned_cloud;
	}

	void calculatePCA()
{
		if (!raw_point_cloud || raw_point_cloud->empty()) {
			std::cerr << "Point cloud is empty or not valid." << std::endl;
			return;
		}

		// 使用PCA计算主成分
		pcl::PCA<pcl::PointXYZ> pca;
		pca.setInputCloud(raw_point_cloud);

		raw_pca_matrix = pca.getEigenVectors();
		raw_pca_values = pca.getEigenValues();
	}

	void calculateCentroid()
	{
		centroid = { 0.0f, 0.0f, 0.0f };
		for (const auto& point : raw_point_cloud->points) {
			centroid[0] += point.x;
			centroid[1] += point.y;
			centroid[2] += point.z;
		}
		centroid /= static_cast<float>(raw_point_cloud->points.size());  // 计算质心
	}

	// 计算最小旋转包围盒（XY平面）
	void calculateBoundingBox() {
		if (!raw_point_cloud || raw_point_cloud->empty()) {
			std::cerr << "Point cloud is empty or not valid." << std::endl;
			return;
		}

		// 使用主成分矩阵旋转点云到新的坐标系
		Eigen::Matrix3f rotation_matrix = raw_pca_matrix;

		// 转换点到主成分坐标系
		std::vector<Eigen::Vector3f> projected_points;
		for (const auto& point : raw_point_cloud->points) {
			Eigen::Vector3f transformed_point(point.x, point.y, point.z);
			// 旋转到主成分坐标系
			projected_points.push_back(rotation_matrix.transpose() * (transformed_point - centroid));
		}

		// 计算最小包围盒（XY平面上的包围盒）
		float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
		float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();

		for (const auto& projected_point : projected_points) {
			min_x = std::min(min_x, projected_point.x());
			max_x = std::max(max_x, projected_point.x());
			min_y = std::min(min_y, projected_point.y());
			max_y = std::max(max_y, projected_point.y());
		}

		// 存储最小包围盒的4个参数：xmin, xmax, ymin, ymax
		min_bounding_box[0] = min_x;  // xmin
		min_bounding_box[1] = max_x;  // xmax
		min_bounding_box[2] = min_y;  // ymin
		min_bounding_box[3] = max_y;  // ymax
	}
};


void RoadMarkingClassifier::ClassifyRoadMarkings(const std::vector<PCLCloudPtr>& clouds,
	RoadMarkings& roadmarkings,
	const std::string& model_path)
{
	std::vector<Model> models = Model::loadFromJson(model_path);
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

		// 找到对应类别的模型
		Model& model = models[roadmarking.category];

		// 清空旧的多条折线（如果之前已有残留）
		roadmarking.polylines.clear();

		// 获取对应的变换矩阵
		const Eigen::Matrix4f& transform = roadmarking.localization_tranmat_m2s;

		// 遍历该模型下的每一条“原始折线”
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

bool RoadMarkingClassifier::model_match(const std::vector<Model>& models, const vector<PCLCloudPtr>& sceneClouds, RoadMarkings& roadmarkings)
{
	int iter_num_ = 12;							// 迭代次数
	float correct_match_fitness_thre_ = 0.1;	// ICP 适配度阈值（越小，匹配要求越严格）
	float overlapDis_ = 0.1;					// 允许的点云重叠距离（决定匹配点云的贴合程度）
	float tolerantMinOverlap_ = 0.70;			// 最小重叠比例
	float heading_increment_ = 20.0;			// 旋转角度步长（单位：度），用于旋转匹配

	float correct_match_fitness_thre = correct_match_fitness_thre_;
	float overlapping_dist_thre = overlapDis_;
	float heading_increment = heading_increment_;
	int iter_num = iter_num_;

	//modeldatas.resize(scenePointClouds.size());
	//is_rights.resize(scenePointClouds.size());

	int min_point_num_for_match = 50;
	int i;
	// #pragma omp parallel for private(i)  // roadmarkings的push有竞争
	for (i = 0; i < sceneClouds.size(); i++)
	{
		if (is_line_cloud_and_get_direction(sceneClouds[i], roadmarkings))
		{
			continue;
		}


		// 清除源点云中的 NaN  Inf点 (应该来自高程图没有完全插值)
		PCLCloudPtr sceneCloud(new PCLCloud);
		sceneCloud->reserve(sceneClouds[i]->size());
		for (const auto& pt : sceneClouds[i]->points)
		{
			if (pcl::isFinite(pt))  // PCL 提供的检查点是否是有限数（非 NaN 且非 Inf）
			{
				sceneCloud->push_back(pt);
			}
		}

		// 初始化最佳重叠比率为一个容忍的最小重叠比率
		float best_overlapping_ratio = tolerantMinOverlap_;
		float match_fitness_best; // 最佳拟合度
		Eigen::Matrix4f tran_mat_m2s_best_match; // 最佳变换矩阵
		int best_model_index = -1; // 最佳模型索引

		// 创建场景点云的 KdTree 用于最近邻查找
		pcl::KdTreeFLANN<pcl::PointXYZ> scene_kdtree;
		scene_kdtree.setInputCloud(sceneCloud);

		// 遍历所有的模型点云进行匹配
		for (int j = 0; j < models.size(); j++)
		{
			float temp_match_fitness, overlapping_ratio;

			// 为变换后的模型点云创建新指针
			PCLCloudPtr modelPointClouds_tran(new PCLCloud);
			Eigen::Matrix4f tran_mat_m2s_temp; // 临时的变换矩阵

			// 执行ICP匹配，得到临时匹配拟合度和变换矩阵
			temp_match_fitness = reg_pca_then_icp(models[j], sceneCloud, tran_mat_m2s_temp, heading_increment, iter_num, correct_match_fitness_thre);

			if (temp_match_fitness < 0)continue;

			// 对模型点云进行变换
			pcl::transformPointCloud(*models[j].raw_point_cloud, *modelPointClouds_tran, tran_mat_m2s_temp);

			// 计算变换后模型与场景点云的重叠比率
			overlapping_ratio = cal_overlap_ratio(modelPointClouds_tran, scene_kdtree.makeShared(), overlapping_dist_thre);

			// 如果重叠比率大于阈值，则计算场景点云与变换后模型的重叠比率
			if (overlapping_ratio > tolerantMinOverlap_ - 0.05)
			{
				// 使用预先构建的模型 KD 树
				pcl::KdTreeFLANN<pcl::PointXYZ> model_kdtree;
				model_kdtree.setInputCloud(modelPointClouds_tran);
				// 计算并取两个重叠比率的平均值
				overlapping_ratio = 0.5 * (overlapping_ratio +
					cal_overlap_ratio(sceneCloud, model_kdtree.makeShared(), overlapping_dist_thre));
			}

			//std::vector<QString> dynamic_text = { "模板点云：" + QString::fromStdString(models[j].name),"重叠率：" + QString::number(overlapping_ratio),"得分：" + QString::number(temp_match_fitness) };
			//visualizePointClouds("点云匹配", dynamic_text, sceneCloud, modelPointClouds_tran);

			// 如果当前模型的重叠比率更好且拟合度小于阈值，则更新最佳匹配
			if (overlapping_ratio > best_overlapping_ratio && temp_match_fitness < correct_match_fitness_thre)
			{
				best_overlapping_ratio = overlapping_ratio;
				match_fitness_best = temp_match_fitness;
				tran_mat_m2s_best_match = tran_mat_m2s_temp;
				best_model_index = j;
			}
		}

		// 如果找到了最佳匹配模型
		if (best_model_index >= 0)
		{
			roadmarkings.push_back({});
			// 为当前道路标线分配最佳模型类别
			roadmarkings.back().category = best_model_index;
			roadmarkings.back().localization_tranmat_m2s = tran_mat_m2s_best_match;

			//std::vector<QString> dynamic_text = { t("最佳匹配模板点云：") + QString::fromStdString(models[best_model_index].name),t("重叠率：") + QString::number(best_overlapping_ratio),t("得分：") + QString::number(match_fitness_best) };

			//pcXYZPtr modelPointClouds_tran(new pcXYZ);
			//pcl::transformPointCloud(*models[best_model_index].raw_point_cloud, *modelPointClouds_tran, tran_mat_m2s_best_match);
			//visualizePointClouds(t("点云匹配"), dynamic_text, sceneCloud, modelPointClouds_tran);
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

/*
#include <pcl/features/fpfh.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
// 特征点配准
float feature_based_registration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& model_cloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_cloud,
	Eigen::Matrix4f& transformation_matrix)
{
	// Step 1: 提取源点云的 FPFH 特征
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);

	fpfh_estimation.setInputCloud(model_cloud);
	fpfh_estimation.setRadiusSearch(0.05);
	fpfh_estimation.compute(*model_fpfh);

	fpfh_estimation.setInputCloud(scene_cloud);
	fpfh_estimation.compute(*scene_fpfh);

	// Step 2: 使用 Kd-Tree 进行最近邻匹配
	pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree;
	kdtree.setInputCloud(model_fpfh);

	std::vector<int> match_indices;
	std::vector<float> match_distances;
	float total_distance = 0.0f;
	int match_count = 0;

	for (size_t i = 0; i < scene_fpfh->size(); ++i)
	{
		if (kdtree.nearestKSearch(scene_fpfh->points[i], 1, match_indices, match_distances) > 0)
		{
			total_distance += match_distances[0];
			match_count++;
		}
	}

	// Step 3: 判断特征匹配质量
	float average_distance = total_distance / match_count;
	if (average_distance > 0.1f)  // 假设阈值
	{
		// 如果匹配的误差较大，说明可能存在较大方向偏差
		return -1.0f;
	}

	// Step 4: 计算变换矩阵
	// 这里可以用简单的 **SVD** 或其他方法来计算点云之间的变换
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr_estimation;
	corr_estimation.setInputSource(model_fpfh);
	corr_estimation.setInputTarget(scene_fpfh);

	pcl::registration::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> reg;
	reg.setInputSource(model_cloud);
	reg.setInputTarget(scene_cloud);
	reg.setMinSampleDistance(0.05);
	reg.setMaxCorrespondenceDistance(0.1);
	reg.setMaximumIterations(1000);
	reg.align(*model_cloud);

	// 返回配准后的变换矩阵
	transformation_matrix = reg.getFinalTransformation();
	return 0.0f; // 匹配成功
}

float RoadMarkingClassifier::reg_pca_then_icp(const Model& model, const PCLCloudPtr& sceneCloud,
	Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre)
{
	// 1. 使用PCA进行初步对齐
	Eigen::Matrix4f initial_transformation;
	align_with_PCA(model.raw_point_cloud, sceneCloud, initial_transformation);

	PCLCloudPtr outline_scene_cloud = get_hull_cloud(sceneCloud);

	// 2. 使用ICP进行精细匹配
	Eigen::Matrix4f final_transformation;
	// 使用icp_reg进行精细匹配，传递必要的参数（初始变换、最大迭代次数、距离阈值等）
	float fitness_score = icp_reg(model.outline_point_cloud, outline_scene_cloud, initial_transformation, final_transformation,
		max_iter_num, dis_thre);

	tran_mat_m2s_best = final_transformation;

	//// 3. 反射
	//{
	//	// 反射模型并重新计算PCA和ICP
	//	PCLCloudPtr reflect_model(new PCLCloud);
	//	Eigen::Matrix4f reflect_mat;
	//	reflect_mat.setIdentity();

	//	reflect_mat(0, 0) = -1.0;  // 反射矩阵，沿模板主方向对称

	//	PCLCloudPtr reflect_raw_model(new PCLCloud);

	//	// 对模型进行反射变换
	//	pcl::transformPointCloud(*model.outline_point_cloud, *reflect_model, reflect_mat);
	//	pcl::transformPointCloud(*model.raw_point_cloud, *reflect_raw_model, reflect_mat);


	//	// 使用PCA对反射后的模型进行初步对齐
	//	Eigen::Matrix4f reflect_initial_transformation;
	//	alignWithPCA(reflect_raw_model, sceneCloud, reflect_initial_transformation);

	//	// 使用icp_reg进行精细配准
	//	Eigen::Matrix4f reflect_final_transformation;
	//	float fitness_score2 = icp_reg(reflect_model, outline_scene_cloud, reflect_initial_transformation, reflect_final_transformation,
	//		max_iter_num, dis_thre);

	//	if (fitness_score2 < fitness_score)
	//	{
	//		fitness_score = fitness_score2;
	//		tran_mat_m2s_best = reflect_final_transformation;
	//	}
	//}
	return fitness_score;
}
*/


/*
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/normal_3d.h>

// 使用特征配准代替 PCA
float RoadMarkingClassifier::reg_pca_then_icp(const Model& model, const PCLCloudPtr& sceneCloud,
	Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre)
{
	// ----- 第零步：异常值滤除（可选） -----
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(sceneCloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(0.5);  // 调低标准差阈值
		sor.filter(*scene_filtered);
	}

	// ----- 第一步：体素下采样（可选，但一般推荐） -----
	float voxel_size = 0.05f; // 增大体素大小
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampled(new pcl::PointCloud<pcl::PointXYZ>);

	{
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setLeafSize(voxel_size, voxel_size, voxel_size);

		vg.setInputCloud(model.raw_point_cloud);
		vg.filter(*model_downsampled);

		vg.setInputCloud(scene_filtered);
		vg.filter(*scene_downsampled);
	}

	// ----- 第二步：法向量估计 -----
	pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);

	{
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		ne.setInputCloud(model_downsampled);
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(0.2);  // 增大法向量估计半径
		ne.compute(*model_normals);

		ne.setInputCloud(scene_downsampled);
		ne.compute(*scene_normals);
	}

	// ----- 第三步：FPFH 特征计算 -----
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);

	{
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr fpfh_tree(new pcl::search::KdTree<pcl::PointXYZ>);
		fpfh.setSearchMethod(fpfh_tree);
		fpfh.setRadiusSearch(0.2);  // 增大特征半径

		fpfh.setInputCloud(model_downsampled);
		fpfh.setInputNormals(model_normals);
		fpfh.compute(*model_fpfh);

		fpfh.setInputCloud(scene_downsampled);
		fpfh.setInputNormals(scene_normals);
		fpfh.compute(*scene_fpfh);
	}

	// ----- 第四步：基于 RANSAC 的粗配准（SampleConsensusPrerejective） -----
	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
	sac.setInputSource(model_downsampled);
	sac.setInputTarget(scene_downsampled);
	sac.setSourceFeatures(model_fpfh);
	sac.setTargetFeatures(scene_fpfh);

	sac.setMaximumIterations(10000);  // 增加 RANSAC 迭代次数
	sac.setNumberOfSamples(3);
	sac.setCorrespondenceRandomness(5);
	sac.setSimilarityThreshold(0.9f);
	sac.setMaxCorrespondenceDistance(0.08f);  // 调整最大对应距离
	sac.setInlierFraction(0.4f);  // 增加内点比例

	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	sac.align(*sac_aligned);

	if (!sac.hasConverged()) {
		return -1.0f;  // 特征配准失败
	}

	Eigen::Matrix4f init_transformation = sac.getFinalTransformation();

	// ----- 第五步：ICP 精细配准 -----
	// 如果想使用轮廓点云，可从 outline_point_cloud 获取，否则直接用下采样的完整场景点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_for_icp = get_hull_cloud(sceneCloud);
	if (target_for_icp->empty())
	{
		// 如果 hull 云为空，就退回到下采样后的完整场景云
		target_for_icp = scene_downsampled;
	}

	// 同样，对目标 ICP 云进行下采样或滤波以提高效率（可选）
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	{
		pcl::VoxelGrid<pcl::PointXYZ> vg2;
		vg2.setLeafSize(voxel_size, voxel_size, voxel_size);
		vg2.setInputCloud(target_for_icp);
		vg2.filter(*target_downsampled);
	}

	// 也可以对 sac_aligned 做一次下采样，保证与 target_downsampled 点数相近
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_for_icp(new pcl::PointCloud<pcl::PointXYZ>);
	{
		pcl::VoxelGrid<pcl::PointXYZ> vg3;
		vg3.setLeafSize(voxel_size, voxel_size, voxel_size);
		vg3.setInputCloud(sac_aligned);
		vg3.filter(*source_for_icp);
	}

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source_for_icp);
	icp.setInputTarget(target_downsampled);
	icp.setMaxCorrespondenceDistance(dis_thre);   // 点到点最大对应距离阈值
	icp.setMaximumIterations(max_iter_num);      // 最大迭代次数
	icp.setTransformationEpsilon(1e-8);          // 变换矩阵收敛阈值
	icp.setEuclideanFitnessEpsilon(1e-6);        // 坐标误差收敛阈值

	pcl::PointCloud<pcl::PointXYZ> icp_result;
	icp.align(icp_result, init_transformation);

	if (!icp.hasConverged())
	{
		// ICP 未收敛
		return -1.0f;
	}

	tran_mat_m2s_best = icp.getFinalTransformation();
	float fitness_score = static_cast<float>(icp.getFitnessScore());

	return fitness_score;
}

*/
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/fpfh.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>

float RoadMarkingClassifier::reg_pca_then_icp(const Model& model, const PCLCloudPtr& sceneCloud,
	Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre) {

	max_iter_num = 50;
	// 1. 体素下采样
	float voxel_size = 0.05f;
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
		return -1.0f;
	}

	// 3. 计算法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	auto getXYNormal = [&](PCLCloudPtr cloud) {
		// 创建新的点云集合，给每个点云加上多个不同Z值的“层”
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLayered(new pcl::PointCloud<pcl::PointXYZ>());
		cloudLayered->reserve(cloud->points.size() * 15);  // 生成三个层次

		// 创建多个不同Z值的“层”
		for (float z_offset = -0.1f; z_offset <= 0.1f; z_offset += 0.02f) {
			if (fabs(z_offset - 0.0f) < 0.01)continue;
			for (const auto& point : cloud->points) {
				pcl::PointXYZ new_point = point;
				new_point.z += z_offset;  // 在Z轴上增加不同的偏移量
				cloudLayered->push_back(new_point);
			}
		}

		ne.setInputCloud(cloudLayered);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		ne.compute(*normals);

		normals->points.resize(cloud->points.size());
		return normals;
	};

	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.1);

	pcl::PointCloud<pcl::Normal>::Ptr model_normals = getXYNormal(model_keypoints);
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals = getXYNormal(scene_keypoints);

	// 4. 计算 FPFH 特征描述子
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(model_keypoints);
	fpfh.setInputNormals(model_normals);
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
	// 设置每次迭代时使用的最小样本集（通常为3个点）
	sac.setNumberOfSamples(3);

	// 设置随机匹配的点数（默认为10个，通常为3到5个足够）
	sac.setCorrespondenceRandomness(10);

	sac.setSimilarityThreshold(0.8f);
	sac.setMaxCorrespondenceDistance(0.2f);
	sac.setInlierFraction(0.6f);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	sac.align(*sac_aligned);

	if (!sac.hasConverged()) {
		return -1.0f;  // 配准失败
	}

	Eigen::Matrix4f init_transformation = sac.getFinalTransformation();

	{
		visualizePointCloudWithNormals("点云二维法向量", model_keypoints, model_normals);

		// 对模型点云进行变换
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints_tran(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*model_keypoints, *model_keypoints_tran, init_transformation);

		// 可视化
		visualizePointClouds("粗配准", {}, scene_keypoints, model_keypoints_tran);
	}

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
		return -1.0f;
	}

	tran_mat_m2s_best = icp.getFinalTransformation();

	{
		// 对模型点云进行变换
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_downsampled_tran(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*model_downsampled, *model_downsampled_tran, tran_mat_m2s_best);

		// 可视化
		visualizePointClouds("精配准", {}, scene_downsampled, model_downsampled_tran);
	}

	return static_cast<float>(icp.getFitnessScore());
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

		// 8. 将方向向量和这一条“直线折线”保存到 rm 中
		rm.direction = (mid_top - mid_bottom).head<4>(); // 四维向量

		// 这里把两个端点作为一条折线，插入到 polylines 列表里
		std::vector<pcl::PointXYZ> oneLine;
		oneLine.emplace_back(pcl::PointXYZ(mid_bottom.x(), mid_bottom.y(), mid_bottom.z()));
		oneLine.emplace_back(pcl::PointXYZ(mid_top.x(), mid_top.y(), mid_top.z()));

		rm.polylines.clear();
		rm.polylines.push_back(std::move(oneLine));

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
				// 取出这一条折线的前两个点，作为 “一个线段”
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

		// 如果是“前向搜索”，则把当前点加到 combinelines.back() 的尾部
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

				// 如果是“前向”，先把下一个线段的连接端 push 到尾部
				if (is_forward)
				{
					if (!next_is_r)
						combinelines.back().push_back(sidelines[i].first);
					else
						combinelines.back().push_back(sidelines[i].second);
				}

				// 继续 DFS，下一个线段成为新的 pos，端点由 next_is_r 决定
				dfs(i, next_is_r, is_forward);

				// 如果是“后向”（is_forward == false），则在回溯时把连接端 push 到尾部
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

		// 如果是“后向”搜索，当所有可扩展分支处理完后，把当前端点 point push 到尾部
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
			// 先“后向”搜一遍，填充 combinelines.back() 的前半部分
			dfs(i, /*is_r=*/false, /*is_forward=*/false);
			// 再“前向”搜一遍，填充 combinelines.back() 的后半部分
			dfs(i, /*is_r=*/true,  /*is_forward=*/true);
		}
	}

	// 4. 将 combinelines 转换为最终的 combine_sideline_markings（带类别标记 COMBINE_SIDE_LINES）
	combine_sideline_markings.clear();
	combine_sideline_markings.resize(static_cast<int>(combinelines.size()));

	for (int i = 0; i < static_cast<int>(combinelines.size()); ++i)
	{
		RoadMarking& rm_comb = combine_sideline_markings[i];
		rm_comb.category = COMBINE_SIDE_LINES;

		// 把一条组合好的折线插入到 rm_comb.polylines 中
		rm_comb.polylines.clear();
		rm_comb.polylines.push_back(std::move(combinelines[i]));
	}
}
