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
		/// �����ӵ㿪ʼ����һ���߶Σ���ָ�����򲻶���չ����ȡ�����е���״���֡�
		/// </summary>
		/// <param name="P">������ƶ���</param>
		/// <param name="p0">������ʼ�㡣</param>
		/// <param name="v0">��ʼ��������</param>
		/// <param name="select_points">�������ѡ�еĵ��Ʋ��֡�</param>
		/// <param name="result">����������õ����߶ζ������С�</param>
		/// <param name="m_glWindow">��ѡ�����ڵ���ʱ���������̿��ӻ������ڡ�</param>
		/// <param name="isGetGround">�Ƿ�����ȡ���������������</param>
		/// <param name="doFitLine">�Ƿ���ÿ������ʱ�� RANSAC ���ֱ�߲���ȡ inliers��</param>
		/// <param name="useDynamicRect">�Ƿ�������������ʹ�ö�̬���Σ����ݵ��������ƶ����ڣ���</param>
		/// <param name="W">���ο�ȣ��ף���Ĭ�� 0.2��</param>
		/// <param name="L">ÿ�������������ף���Ĭ�� 2��</param>
		/// <param name="Nmin">ÿ�������ٰ����ĵ�����Ĭ�� 50��</param>
		/// <param name="theta_max">������������������۽Ƕȣ����ȣ���Ĭ�� 45�㡣</param>
		/// <param name="Kmax">�����Ծ������������ѣ���Ĭ�� 10��</param>
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
