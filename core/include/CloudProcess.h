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
			ccPointCloud* P,                  // ���ƶ���
			const CCVector3& p0,              // ��ʼ��
			const CCVector3& v0,              // ��ʼ����
			ccPointCloud* select_points,      // ��ѡ���еĲ��ֵ���
			std::vector<CCVector3>& result,    // ��ѡ�в��ֵ���
			ccGLWindowInterface* m_glWindow,  // ���ڵ�����ʾ
			bool isGetGround,
			double W = 0.2,                   // ���ο�ȣ�Ĭ�� 0.2 ��
			double L = 2,                     // ÿ������������Ĭ�� 2 ��
			unsigned Nmin = 50,               // ��С������Ĭ������ 50 ����
			double theta_max = 45.0 * M_PI / 180.0, // ������۽Ƕȣ�Ĭ�� 45 �ȣ���λ�ǻ���
			unsigned Kmax = 10                // �����Ծ������Ĭ�� 10 ��
		);
	};
}
