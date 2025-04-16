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
	std::vector<PCLPoint> vectorized_points;  // 向量化点的列表

	std::string name;

	PCLCloudPtr outline_point_cloud;
	PCLCloudPtr raw_point_cloud;

	// 主成分分析结果
	Eigen::Vector3f raw_pca_values;      // 原始点云的特征值
	Eigen::Matrix3f raw_pca_matrix;      // 原始点云的特征向量矩阵

	Eigen::Vector3f centroid;			 // 质心
	Eigen::Vector4f min_bounding_box;    // 最小旋转包围盒（xmin, xmax, ymin, ymax）
	// 构造函数
	Model(const string& name, const std::string& outline_path, const std::string& raw_path, const std::vector<PCLPoint>& points)
		:name(name), outline_point_cloud_path(outline_path), raw_point_cloud_path(raw_path), vectorized_points(points)
	{
		// 读取轮廓点云文件
		outline_point_cloud = loadPointCloud(outline_path);
		// 读取原始点云文件
		raw_point_cloud = loadPointCloud(raw_path);

		if (outline_point_cloud->empty() || raw_point_cloud->empty())
		{
			return;
		}

		calculatePCA();

		calculateCentroid();

		calculateBoundingBox();
	}

	// 静态函数加载 JSON 文件
	static std::vector<Model> loadFromJson(const std::string& filename) {
		// 打开文件
		std::ifstream input(filename);
		if (!input.is_open()) {
			std::cerr << "Could not open file " << filename << std::endl;
			return {};
		}

		// 解析 JSON 数据
		nlohmann::json j;
		input >> j;
		std::vector<Model> models;

		// 遍历 JSON 中的 "model" 数组
		for (const auto& model_data : j["model"]) {
			std::string name = model_data["name"];
			std::string outline_path = model_data["outline_point_cloud_path"];
			std::string raw_path = model_data["raw_point_cloud_path"];
			std::vector<PCLPoint> points;

			// 遍历 "vectorized_points"
			for (const auto& point : model_data["vectorized_points"]) {
				std::vector<float> point_vec = point.get<std::vector<float>>();
				points.push_back({ point_vec[0], point_vec[1] ,point_vec[2] });
			}

			// 创建 Model 对象并添加到 models 向量中
			models.push_back(Model(name, outline_path, raw_path, points));
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

	void calculatePCA() {
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

	MarkingVectorization(models, roadmarkings);
}

void test_CombineSideLines(RoadMarkingClassifier& obj,
	void (RoadMarkingClassifier::* func_ptr)(const RoadMarkings&, double, RoadMarkings&)
) {
	// 使用 reinterpret_cast 将成员函数指针转换为可以调用的指针
	void (RoadMarkingClassifier:: * func_ptr_cast)(const RoadMarkings&, double, RoadMarkings&) =
		reinterpret_cast<void (RoadMarkingClassifier::*)(const RoadMarkings&, double, RoadMarkings&)>(func_ptr);

	RoadMarkings roadmarkings;

	for (int i = 1; i < 5; i++)
	{
		roadmarkings.push_back({});
		roadmarkings.back().category = SIDE_LINE;
		roadmarkings.back().polyline = { {0,(float)10.0 * i,0}, {0,(float)10.0 * i + 5,0} };
		roadmarkings.back().direction = { 0,1,0,0 };
	}

	double combine_length = 11;
	RoadMarkings combine_sideline_markings;
	// 调用私有成员函数
	(obj.*func_ptr_cast)(roadmarkings, combine_length, combine_sideline_markings);
};

void RoadMarkingClassifier::MarkingVectorization(std::vector<Model>& models, RoadMarkings& roadmarkings)
{
	for (auto& roadmarking : roadmarkings)
	{
		if (roadmarking.category >= 0)
		{
			if (roadmarking.category == SIDE_LINE) // 线形
			{
				continue;
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
			for (const auto& point : models[roadmarking.category].vectorized_points)
			{
				cloud_in->push_back(point);
			}

			// 获取对应的变换矩阵
			Eigen::Matrix4f transform = roadmarking.localization_tranmat_m2s;

			// 创建一个新的点云来存储变换后的结果
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

			// 进行变换
			pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

			// 将变换后的点云转换回 std::vector
			roadmarking.polyline.clear();  // 清空原有的点
			for (const auto& point : cloud_out->points)
			{
				roadmarking.polyline.push_back({ point.x, point.y, point.z });
			}
		}
	}


	//test_CombineSideLines(*this, &RoadMarkingClassifier::CombineSideLines);



	RoadMarkings combine_sideline_markings;
	CombineSideLines(roadmarkings, 20, combine_sideline_markings);

	RoadMarkings result;
	for (auto& line : roadmarkings)
	{
		if (line.category != SIDE_LINE)result.push_back(line);
	}
	for (auto& line : combine_sideline_markings)
	{
		result.push_back(line);
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

	float corr_dist_thre = FLT_MAX;

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
			temp_match_fitness = reg_pca_then_icp(models[j], sceneCloud, tran_mat_m2s_temp, heading_increment, iter_num, corr_dist_thre);

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

			/*std::vector<QString> dynamic_text = { t("模板点云：") + QString::fromStdString(models[j].name),t("重叠率：") + QString::number(overlapping_ratio),t("得分：") + QString::number(temp_match_fitness) };
			visualizePointClouds(t("点云匹配"), dynamic_text, sceneCloud, modelPointClouds_tran);*/

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

void RoadMarkingClassifier::alignWithPCA(const PCLCloudPtr& ModelCloud,
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


float RoadMarkingClassifier::reg_pca_then_icp(const Model& model, const PCLCloudPtr& sceneCloud,
	Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre)
{
	// 1. 使用PCA进行初步对齐
	Eigen::Matrix4f initial_transformation;
	alignWithPCA(model.raw_point_cloud, sceneCloud, initial_transformation);

	PCLCloudPtr outline_scene_cloud = getHullCloud(sceneCloud);

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

PCLCloudPtr RoadMarkingClassifier::getHullCloud(PCLCloudPtr cloud)
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

void RoadMarkingClassifier::alignPointCloudToXAxisInPlace(const PCLCloudPtr& cloud, Eigen::Matrix4f& transformation)
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
	// 1. 计算原地旋转变换，使点云的主要方向对齐到 X 轴
	Eigen::Matrix4f T;
	alignPointCloudToXAxisInPlace(cloud, T);

	// 2. 将点云旋转到主成分坐标系（对齐到 X 轴）
	PCLCloudPtr rotated_cloud(new PCLCloud);
	pcl::transformPointCloud(*cloud, *rotated_cloud, T);

	// 3. 计算旋转后点云的轴对齐包围盒（仅考虑 XY 平面）
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*rotated_cloud, minPt, maxPt);

	float dx = maxPt.x - minPt.x;
	float dy = maxPt.y - minPt.y;
	float length = std::max(dx, dy);
	float width = std::min(dx, dy);

	// 4. 根据条件判断是否为线形点云
	if (width < 0.35f && length > 0.8f)
	{
		// 将当前道路标线添加到 roadmarkings 中
		roadmarkings.push_back({});



		roadmarkings.back().category = SIDE_LINE;  // SIDE_LINE 表示线形点云

		// 计算包围盒在旋转空间中的四个角点（XY 平面），Z 取平均值
		float avg_z = (minPt.z + maxPt.z) / 2.0f;
		// 四角点（局部：旋转后的坐标）
		Eigen::Vector4f corner1(minPt.x, minPt.y, avg_z, 1.0f); // 左下
		Eigen::Vector4f corner2(minPt.x, maxPt.y, avg_z, 1.0f); // 左上
		Eigen::Vector4f corner3(maxPt.x, maxPt.y, avg_z, 1.0f); // 右上
		Eigen::Vector4f corner4(maxPt.x, minPt.y, avg_z, 1.0f); // 右下
		// 5. 计算 T 的逆矩阵，用于将角点转换回原始坐标系
		Eigen::Matrix4f T_inv = T.inverse();
		Eigen::Vector4f orig_corner1 = T_inv * corner1;
		Eigen::Vector4f orig_corner2 = T_inv * corner2;
		Eigen::Vector4f orig_corner3 = T_inv * corner3;
		Eigen::Vector4f orig_corner4 = T_inv * corner4;

		// 6. 将转换后的四个顶点保存到 roadmarkings.back().polyline
		//roadmarkings.back().polyline.push_back(pcl::PointXYZ(orig_corner1.x(), orig_corner1.y(), orig_corner1.z()));
		//roadmarkings.back().polyline.push_back(pcl::PointXYZ(orig_corner2.x(), orig_corner2.y(), orig_corner2.z()));
		//roadmarkings.back().polyline.push_back(pcl::PointXYZ(orig_corner3.x(), orig_corner3.y(), orig_corner3.z()));
		//roadmarkings.back().polyline.push_back(pcl::PointXYZ(orig_corner4.x(), orig_corner4.y(), orig_corner4.z()));
		// 闭合
		//roadmarkings.back().polyline.push_back(pcl::PointXYZ(orig_corner1.x(), orig_corner1.y(), orig_corner1.z()));

		// 从左到右(沿记录的方向)
		Eigen::Vector4f mid_bottom = (orig_corner1 + orig_corner2) / 2.0f;
		Eigen::Vector4f mid_top = (orig_corner3 + orig_corner4) / 2.0f;

		//roadmarkings.back().direction = T.col(0); // 旋转后的 X 轴方向, 就是点云的方向
		roadmarkings.back().direction = mid_top - mid_bottom;
		roadmarkings.back().polyline.push_back(pcl::PointXYZ(mid_bottom.x(), mid_bottom.y(), mid_bottom.z()));
		roadmarkings.back().polyline.push_back(pcl::PointXYZ(mid_top.x(), mid_top.y(), mid_top.z()));
		return true;
	}
	else
	{
		return false;
	}
}

// 主函数：将符合条件的侧线段进行组合
void RoadMarkingClassifier::CombineSideLines(const RoadMarkings& roadmarkings, double combine_length, RoadMarkings& combine_sideline_markings)
{
	// 存储所有侧线段的起始和结束点
	vector<pair<pcl::PointXYZ, pcl::PointXYZ>> sidelines;
	vector<Eigen::Vector3f> direction;
	vector<int> sidelines_index_in_roadmarkings;  // 存储每条线段在原始路标中的索引

	// 遍历所有路标，找出所有类别为SIDE_LINE的侧线段
	for (int i = 0; i < roadmarkings.size(); i++)
	{
		if (roadmarkings[i].category == SIDE_LINE)
		{
			pair<pcl::PointXYZ, pcl::PointXYZ> sideline;
			sideline.first = roadmarkings[i].polyline[0];
			sideline.second = roadmarkings[i].polyline[1];
			sidelines.push_back(sideline);
			direction.push_back(roadmarkings[i].direction.head<3>());
			sidelines_index_in_roadmarkings.push_back(i);
		}
	}

	int sideline_number = sidelines.size();  // 侧线段的数量

	vector<vector<PCLPoint>> combinelines;
	vector<bool> line_used(sideline_number);

	auto len2 = [](PCLPoint& a, PCLPoint& b)
	{
		return std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2) + std::pow(b.z - a.z, 2);
	};

	auto calculateAngleBetweenVectors = [](const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
	{
		// 计算点积
		double dotProduct = v1.dot(v2);

		// 计算向量的模长
		double magnitude1 = v1.norm();
		double magnitude2 = v2.norm();

		// 计算夹角的余弦值
		double cosTheta = dotProduct / (magnitude1 * magnitude2);

		double temp = std::acos(cosTheta);
		// 通过反余弦计算夹角（返回值单位为弧度）
		return std::acos(cosTheta);
	};

	const double threshold_length2 = combine_length * combine_length;
	const double threshold_angle = 5.0 * M_PI / 180.0;

	/// <summary>
	/// dfs搜索
	/// </summary>
	/// <param name="roadmarkings">当前搜索的线段索引</param>
	/// <param name="combine_length">是否为线段的右端</param>
	/// <param name="combine_sideline_markings">是否前向</param>
	function<void(int, bool, bool)> dfs = [&](int pos, bool is_r, bool is_forward)
	{
		PCLPoint point = sidelines[pos].first;
		if (is_r)
		{
			point = sidelines[pos].second;
		}

		if (is_forward)
		{
			combinelines.back().push_back(point);
		}

		for (int i = 0; i < sideline_number; i++)
		{
			if (i == pos)continue;
			int len1 = len2(point, sidelines[i].first);
			int len11 = len2(point, sidelines[i].second);
			if (!line_used[i] && len2(point, sidelines[i].first) < threshold_length2)
			{
				Eigen::Vector3f dir(sidelines[i].first.x - point.x,
					sidelines[i].first.y - point.y,
					sidelines[i].first.z - point.z);
				if (calculateAngleBetweenVectors(is_r ? direction[pos] : -direction[pos], dir) < threshold_angle
					&& calculateAngleBetweenVectors(dir, direction[i]) < threshold_angle
					&& calculateAngleBetweenVectors(is_r ? direction[pos] : -direction[pos], direction[i]) < threshold_angle
					)
				{
					line_used[i] = true;
					if (is_forward)combinelines.back().push_back(sidelines[i].first);
					dfs(i, true, is_forward);
					if (!is_forward)combinelines.back().push_back(sidelines[i].first);
					break;
				}
			}

			if (!line_used[i] && len2(point, sidelines[i].second) < threshold_length2)
			{
				Eigen::Vector3f dir(sidelines[i].second.x - point.x,
					sidelines[i].second.y - point.y,
					sidelines[i].second.z - point.z);
				if (calculateAngleBetweenVectors(is_r ? direction[pos] : -direction[pos], dir) < threshold_angle
					&& calculateAngleBetweenVectors(dir, -direction[i]) < threshold_angle
					&& calculateAngleBetweenVectors(is_r ? direction[pos] : -direction[pos], -direction[i]) < threshold_angle
					)
				{
					line_used[i] = true;
					if (is_forward)combinelines.back().push_back(sidelines[i].second);
					dfs(i, false, is_forward);
					if (!is_forward)combinelines.back().push_back(sidelines[i].second);
					break;
				}
			}
		}

		if (!is_forward)
		{
			combinelines.back().push_back(point);
		}
	};

	for (int i = 0; i < sidelines.size(); i++)
	{
		if (!line_used[i])
		{
			combinelines.push_back({});

			// 索引，是否为右端，是否为前方
			dfs(i, false, false);
			dfs(i, true, true);
		}
	}

	// 初始化组合后的侧线标记
	combine_sideline_markings.resize(combinelines.size());

	// 将组合后的线段添加到最终的组合标记中
	for (int i = 0; i < combinelines.size(); i++)
	{
		combine_sideline_markings[i].category = COMBINE_SIDE_LINES;
		combine_sideline_markings[i].polyline.insert(combine_sideline_markings[i].polyline.end(),
			combinelines[i].begin(), combinelines[i].end());
	}
}
