#pragma once

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


using PCLPoint = pcl::PointXYZ;
using PCLCloud = pcl::PointCloud<PCLPoint>;
using PCLCloudPtr = PCLCloud::Ptr;

class Model;

namespace roadmarking
{
	struct RoadMarking
	{
		int category;

		std::vector<pcl::PointXYZ> polyline; //For vectorization

		Eigen::Matrix4f localization_tranmat_m2s;

		Eigen::Vector4f direction;

		//Other semantic or topological information

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	typedef std::vector<RoadMarking, Eigen::aligned_allocator<RoadMarking>> RoadMarkings;


	class RoadMarkingClassifier
	{
	public:
		void ClassifyRoadMarkings(const std::vector<PCLCloudPtr>& clouds,
			RoadMarkings& roadmarkings,
			const std::string& model_path);

	private:
		void vectorize_roadmarking(std::vector<Model>& models, RoadMarkings& roadmarkings);

		bool model_match(const std::vector<Model>& models, const std::vector<PCLCloudPtr>& scenePointClouds, RoadMarkings& roadmarkings);

		float cal_overlap_ratio(const PCLCloudPtr& search_cloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree, float thre_dis);

		float icp_reg(const PCLCloudPtr& SourceCloud, const PCLCloudPtr& TargetCloud, Eigen::Matrix4f& initial_guess, Eigen::Matrix4f& transformationS2T, int max_iter, float thre_dis);

		void align_with_PCA(const PCLCloudPtr& ModelCloud, const PCLCloudPtr& SceneCloud, Eigen::Matrix4f& initial_transformation);

		float reg_pca_then_icp(const  Model& model, const PCLCloudPtr& sceneCloud, Eigen::Matrix4f& tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre);

		PCLCloudPtr get_hull_cloud(PCLCloudPtr cloud);

		bool is_line_cloud_and_get_direction(PCLCloudPtr cloud, RoadMarkings& roadmarkings);

		void combine_side_lines(const RoadMarkings& roadmarkings, double Combine_length, RoadMarkings& combine_sideline_markings);

		void align_cloud_to_x_axis(const PCLCloudPtr& cloud, Eigen::Matrix4f& transformation);
	};
};

