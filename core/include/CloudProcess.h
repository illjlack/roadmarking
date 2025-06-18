#pragma once

#include "PointCloudIO.h"
//#include <opencv2/opencv.hpp>

namespace roadmarking
{
	// 道路标线元数据
	class MetaRoadmarking : public ccHObject
	{
	};

	class CloudProcess
	{
	public:
		static PCLOctreePtr build_octree(PCLCloudPtr pclCloud, float targetVoxelSize);

		static ccCloudPtr crop_raw_by_sparse_cloud(ccCloudPtr ccCloud, PCLCloudPtr pclCloud, PCLOctreePtr octree = nullptr);

		static ccCloudPtr apply_voxel_grid_filtering(ccCloudPtr ccCloud, float targetVoxelSize, PCLOctreePtr octree = nullptr);

		static PCLCloudPtr apply_voxel_grid_filtering(PCLCloudPtr pclCloud, float targetVoxelSize);

		static ccCloudPtr apply_csf_ground_extraction(ccCloudPtr ccCloud);

		static PCLCloudPtr apply_csf_ground_extraction(PCLCloudPtr pclCloud);

		static PCLCloudPtr extract_max_cloud_by_euclidean_cluster(PCLCloudPtr groundCloud, float euclideanClusterRadius);

		static PCLCloudPtr extract_road_points(PCLCloudPtr groundCloud,
			float searchRadius,
			float angleThreshold,
			float curvatureBaseThreshold,
			PCLPoint* pickSeedPoint = nullptr);

		static std::vector<PCLCloudXYZIPtr> extract_roadmarking(ccCloudPtr roadCloud,
			float resolution,
			double euclideanClusterRadius = 0.2,
			int minNum = 100);

		static PCLCloudPtr match_roadmarking(PCLCloudPtr pclCloud);

		static ccHObject* apply_roadmarking_vectorization(ccPointCloud* cloud);
		static ccHObject* apply_roadmarking_vectorization(ccCloudPtr cloud);
		static ccHObject* apply_roadmarking_vectorization(std::vector<PCLCloudPtr> clouds);

		static PCLCloudPtr extract_outline(const PCLCloudPtr& inputCloud, float alpha = 0.05);

		static void crop_cloud_with_polygon(ccPointCloud* cloud, const std::vector<CCVector3d>& polygon_points, ccPointCloud* cloud_cropped);
		static void crop_cloud_with_polygon(const std::vector<ccPointCloud*>& clouds, const std::vector<CCVector3d>& polygon_points, ccPointCloud* cloud_cropped);

		template<typename PointT>
		static void extract_euclidean_clusters(typename pcl::PointCloud<PointT>::Ptr inputCloud,
			std::vector<typename pcl::PointCloud<PointT>::Ptr>& outputClusters,
			double euclideanClusterRadius = 0.075,
			int minNum = 10);

		static void apply_default_intensity_and_visible(ccCloudPtr cloud);
		static void apply_default_intensity_and_visible(ccPointCloud* cloud);

		static void filter_cloud_by_intensity(ccPointCloud* inCloud, double lowerThreshold, double upperThreshold, ccPointCloud* cloud_filtered);

		static void filter_cloud_by_z(ccPointCloud* inCloud, double lowerThreshold, double upperThreshold, ccPointCloud* cloud_filtered);

		/// <summary>
		/// 从种子点开始，沿一条线段，按指定方向不断扩展，提取其中的线状部分。
		/// </summary>
		/// <param name="P">点云移动对象</param>
		/// <param name="p0">种子起始点</param>
		/// <param name="v0">起始方向向量</param>
		/// <param name="select_points">点云中已选中的点云部分</param>
		/// <param name="result">最终得到的线段端点集合</param>
		/// <param name="m_glWindow">可选，用于实时显示扩展过程</param>
		/// <param name="isGetGround">是否同时提取地面点云部分</param>
		/// <param name="doFitLine">是否在每次扩展时用RANSAC拟合直线并提取inliers</param>
		/// <param name="useDynamicRect">是否在扩展过程中使用动态矩形（根据点云密度动态调整范围）</param>
		/// <param name="W">矩形宽度（米），默认0.2</param>
		/// <param name="L">每次扩展的长度（米），默认2</param>
		/// <param name="Nmin">每次扩展最少包含的点数，默认50</param>
		/// <param name="theta_max">扩展方向允许的最大偏转角度（弧度），默认45°</param>
		/// <param name="Kmax">最大扩展次数，默认10</param>
		static void grow_line_from_seed(
			ccPointCloud* P,
			const CCVector3& p0,
			const CCVector3& v0,
			ccPointCloud* select_points,
			std::vector<CCVector3>& result,
			ccGLWindowInterface* m_glWindow,
			bool                     isGetGround,
			bool                     doFitLine = false,
			bool                     useDynamicRect = false,
			double                   W = 0.2,
			double                   L = 2.0,
			unsigned                 Nmin = 50,
			double                   theta_max = 45.0 * M_PI / 180.0,
			unsigned                 Kmax = 10
		);

		static void extract_zebra_by_struct(ccPointCloud* inputCloud, ccGLWindowInterface* m_glWindow);

		/// <summary>
		/// 使用密度投影法，根据密度阈值提取斑马线的中心线段。
		/// </summary>
		/// <param name="inputCloud">输入点云</param>
		/// <param name="binWidth">投影bin宽度（米），用于将点云分成等距的bin</param>
		/// <param name="densityThreshold">每个bin的最小点数阈值，超过该值认为是斑马线</param>
		/// <param name="minStripeLength">斑马线最小有效长度（未使用，待扩展）</param>
		/// <param name="outputCloud">输出点云，用于显示识别到的斑马线的点</param>
		/// <param name="centers">每个斑马线段的中心点集合</param>
		static void extract_zebra_by_projection(
			ccPointCloud* inputCloud,
			float binWidth,
			int densityThreshold,
			float minStripeLength,
			ccGLWindowInterface* m_glWindow,
			ccPointCloud* outputCloud,
			std::vector<CCVector3>& centers
		);

		static void cluster_points_around_pos(ccPointCloud* select_cloud, unsigned idx,
			float radius, ccPointCloud& clustered_cloud);

		static void computeLocalNormal2D(const PCLCloudPtr& cloud, float radius, pcl::PointCloud<pcl::Normal>::Ptr normals);
	private:
		static ccPointCloud* rotate_cloud(ccPointCloud* P, const CCVector3& now_v, const CCVector3& new_v);
		static PCLCloudPtr rotate_cloud(PCLCloudPtr pclCloud, const Eigen::Vector3f& now_v, const Eigen::Vector3f& new_v);
	};
}
