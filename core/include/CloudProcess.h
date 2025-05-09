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


		static void grow_line_from_seed(
			ccPointCloud* P,                  // 点云对象
			const CCVector3& p0,              // 起始点
			const CCVector3& v0,              // 初始方向
			ccPointCloud* select_points,      // 被选择中的部分点云
			std::vector<CCVector3>& result,    // 被选中部分的线
			ccGLWindowInterface* m_glWindow,  // 用于调试显示
			bool isGetGround,
			double W = 0.2,                   // 矩形宽度，默认 0.2 米
			double L = 2,                     // 每次生长步长，默认 2 米
			unsigned Nmin = 50,               // 最小点数，默认至少 50 个点
			double theta_max = 45.0 * M_PI / 180.0, // 最大弯折角度，默认 45 度，单位是弧度
			unsigned Kmax = 10                // 最大跳跃次数，默认 10 次
		);
	};
}
