#pragma once

#include "PointCloudIO.h"
#include <opencv2/opencv.hpp>

namespace roadmarking
{
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

		static ccHObject* apply_roadmarking_vectorization(ccCloudPtr cloud);

		static ccHObject* apply_roadmarking_vectorization(std::vector<PCLCloudPtr> clouds);

		static PCLCloudPtr extract_outline(const PCLCloudPtr& inputCloud, float alpha = 0.05);

		static std::vector<PCLPoint> draw_polyline_on_cloud_by_pcl_view(const PCLCloudPtr& inputCloud);


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
		/// 从种子点开始生长一条线段，沿指定方向不断扩展并提取点云中的线状部分。
		/// </summary>
		/// <param name="P">输入点云对象。</param>
		/// <param name="p0">种子起始点。</param>
		/// <param name="v0">初始生长方向。</param>
		/// <param name="select_points">输出：被选中的点云部分。</param>
		/// <param name="result">输出：生长得到的线段顶点序列。</param>
		/// <param name="m_glWindow">可选：用于调试时将生长过程可视化到窗口。</param>
		/// <param name="isGetGround">是否先提取地面点云再生长。</param>
		/// <param name="doFitLine">是否在每步生长时做 RANSAC 拟合直线并提取 inliers。</param>
		/// <param name="useDynamicRect">是否在生长过程中使用动态矩形（根据点云质心移动窗口）。</param>
		/// <param name="W">矩形宽度（米），默认 0.2。</param>
		/// <param name="L">每次生长步长（米），默认 2。</param>
		/// <param name="Nmin">每段线至少包含的点数，默认 50。</param>
		/// <param name="theta_max">相邻两段最大允许弯折角度（弧度），默认 45°。</param>
		/// <param name="Kmax">最大跳跃次数（处理断裂），默认 10。</param>
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

	private:
		static ccPointCloud* rotate_cloud(ccPointCloud* P, const CCVector3& now_v, const CCVector3& new_v);
		static PCLCloudPtr rotate_cloud(PCLCloudPtr pclCloud, const Eigen::Vector3f& now_v, const Eigen::Vector3f& new_v);
	};
}
