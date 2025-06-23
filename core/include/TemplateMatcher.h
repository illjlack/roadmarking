#pragma once

#pragma execution_character_set("utf-8")

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <omp.h>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <omp.h>
#include <nlohmann/json.hpp>
#include <fstream>

using PCLPoint = pcl::PointXYZ;
using PCLCloud = pcl::PointCloud<PCLPoint>;
using PCLCloudPtr = PCLCloud::Ptr;

class Model;

namespace roadmarking
{
	struct RoadMarking
	{
		int category;
		std::string name;        // 道路标线名称
		double accuracy;         // 准确率，要找到得分和准确率的关系，这个需要统计，暂时用得分表示
		std::vector<std::vector<pcl::PointXYZ>> polylines;  // 用于矢量化
		Eigen::Matrix4f localization_tranmat_m2s;
		Eigen::Vector4f direction;
		// 其他语义或拓扑信息
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	class Model
	{
	public:
		// 成员变量
		std::vector<std::vector<PCLPoint>> vectorized_polylines;  // 矢量化折线列表
		std::string name;
		PCLCloudPtr outline_point_cloud;
		PCLCloudPtr raw_point_cloud;

		// 构造函数
		inline Model(const std::string& name,
			const std::string& outline_path,
			const std::string& raw_path,
			const std::vector<std::vector<PCLPoint>>& polylines)
			: name(name)
			, vectorized_polylines(polylines)
		{
			// 加载轮廓点云
			outline_point_cloud = loadPointCloud(outline_path);
			// 加载原始点云
			raw_point_cloud = loadPointCloud(raw_path);
		}

		inline Model(): outline_point_cloud(new PCLCloud), raw_point_cloud(new PCLCloud){}

		// 构造函数
		inline Model(const std::string& name,
			PCLCloudPtr outline_point_cloud,
			PCLCloudPtr raw_point_cloud,
			const std::vector<std::vector<PCLPoint>>& polylines)
			: name(name)
			, outline_point_cloud(outline_point_cloud)
			, raw_point_cloud(raw_point_cloud)
			, vectorized_polylines(polylines)
		{
		}

		inline static std::vector<Model> loadFromJson(const std::string& filename)
		{
			std::ifstream input(filename);
			if (!input.is_open()) {
				std::cerr << "无法打开文件 " << filename << std::endl;
				return {};
			}
			nlohmann::json j;
			input >> j;

			std::vector<Model> models;
			// 根节点应为 "models"，注意不要写成 "model" 不同
			if (!j.contains("models") || !j["models"].is_array()) {
				std::cerr << "无效的JSON格式：未找到 \"models\" 数组" << std::endl;
				return {};
			}

			// 遍历 JSON 中的每一个 model 对象
			for (const auto& model_data : j["models"])
			{
				// 获取基本字段
				std::string name = model_data.value("name", "");
				std::string outline_path = model_data.value("outline_point_cloud_path", "");
				std::string raw_path = model_data.value("raw_point_cloud_path", "");

				// 用于存储该 model 对应的折线列表
				std::vector<std::vector<PCLPoint>> polylines;

				// 如果存在 "graph_elements"，则处理它
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
								// 每个 ptArr 应该是 [x, y, z] 的数组
								if (ptArr.is_array() && ptArr.size() == 3)
								{
									float x = ptArr[0].get<float>();
									float y = ptArr[1].get<float>();
									float z = ptArr[2].get<float>();
									onePolyline.emplace_back(x, y, z);
								}
							}
							// 将一条折线加入到 polylines 中
							polylines.push_back(std::move(onePolyline));
						}
					}
				}

				// 创建 Model 对象并加入到模型列表
				models.emplace_back(name, outline_path, raw_path, polylines);
			}

			return models;
		}

		inline static PCLCloudPtr loadPointCloud(const std::string& file_path) {
			PCLCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
				std::cerr << "无法读取文件 " << file_path << std::endl;
				return nullptr;
			}

			// 清理点云中的 NaN Inf
			PCLCloudPtr cleaned_cloud(new PCLCloud);
			cleaned_cloud->reserve(cloud->size());
			for (const auto& pt : cloud->points)
			{
				if (pcl::isFinite(pt))  // PCL 提供的检查是否为有限点的函数，排除 NaN 和 Inf
				{
					cleaned_cloud->push_back(pt);
				}
			}

			return cleaned_cloud;
		}
	};

	typedef std::vector<RoadMarking, Eigen::aligned_allocator<RoadMarking>> RoadMarkings;

	class RoadMarkingClassifier
	{
	public:
		void ClassifyRoadMarkings(const std::vector<PCLCloudPtr>& clouds,
			RoadMarkings& roadmarkings,
			const std::string& model_path);

		void ClassifyRoadMarkings(const std::vector<PCLCloudPtr>& clouds,
			RoadMarkings& roadmarkings,
			std::vector<Model>& models);

		float fpfh_ransac(const Model& model, const PCLCloudPtr& sceneCloud,
			Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre,
			float& match_fitness, float& overlapping_ratio);

		float iss_fpfh_ransac(const Model& model, const PCLCloudPtr& sceneCloud, Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre, float& match_fitness, float& overlapping_ratio);

	private:
		void vectorize_roadmarking(std::vector<Model>& models, RoadMarkings& roadmarkings);

		bool model_match(const std::vector<Model>& models, const std::vector<PCLCloudPtr>& scenePointClouds, RoadMarkings& roadmarkings);

		float cal_overlap_ratio(const PCLCloudPtr& search_cloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree, float thre_dis);

		float icp_reg(const PCLCloudPtr& SourceCloud, const PCLCloudPtr& TargetCloud, Eigen::Matrix4f& initial_guess, Eigen::Matrix4f& transformationS2T, int max_iter, float thre_dis);

		void align_with_PCA(const PCLCloudPtr& ModelCloud, const PCLCloudPtr& SceneCloud, Eigen::Matrix4f& initial_transformation);

		PCLCloudPtr get_hull_cloud(PCLCloudPtr cloud);

		bool is_line_cloud_and_get_direction(PCLCloudPtr cloud, RoadMarkings& roadmarkings);

		void combine_side_lines(const RoadMarkings& roadmarkings, double Combine_length, RoadMarkings& combine_sideline_markings);

		void align_cloud_to_x_axis(const PCLCloudPtr& cloud, Eigen::Matrix4f& transformation);
	};
};

