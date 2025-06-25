#pragma once

#include "PointCloudIO.h"
#include "algorithms/PointCloudDivider.h"

namespace roadmarking
{
	/// <summary>
	/// 道路标线元数据类，继承自 CloudCompare 的 ccHObject
	/// </summary>
	class MetaRoadmarking : public ccHObject
	{
	};

	/// <summary>
	/// 点云处理类，提供各种点云处理和分析功能
	/// </summary>
	class CloudProcess
	{
	public:
		/// <summary>
		/// 为 PCL 点云构建八叉树
		/// </summary>
		/// <param name="pclCloud">输入的点云</param>
		/// <param name="targetVoxelSize">体素大小</param>
		/// <returns>构建的八叉树指针</returns>
		static PCLOctreePtr build_octree(PCLCloudPtr pclCloud, float targetVoxelSize);

		/// <summary>
		/// 使用稀疏点云裁剪原始点云
		/// </summary>
		/// <param name="ccCloud">原始点云</param>
		/// <param name="pclCloud">稀疏点云</param>
		/// <param name="octree">可选的八叉树，如果不提供会自动构建</param>
		/// <returns>裁剪后的点云</returns>
		static ccCloudPtr crop_raw_by_sparse_cloud(ccCloudPtr ccCloud, PCLCloudPtr pclCloud, PCLOctreePtr octree = nullptr);

		/// <summary>
		/// 对点云应用体素网格滤波
		/// </summary>
		/// <param name="ccCloud">输入点云</param>
		/// <param name="targetVoxelSize">目标体素大小</param>
		/// <param name="octree">可选的八叉树，如果不提供会自动构建</param>
		/// <returns>滤波后的点云</returns>
		static ccCloudPtr apply_voxel_grid_filtering(ccCloudPtr ccCloud, float targetVoxelSize, PCLOctreePtr octree = nullptr);

		/// <summary>
		/// 对 PCL 点云应用体素网格滤波
		/// </summary>
		/// <param name="pclCloud">输入 PCL 点云</param>
		/// <param name="targetVoxelSize">目标体素大小</param>
		/// <returns>滤波后的点云</returns>
		static PCLCloudPtr apply_voxel_grid_filtering(PCLCloudPtr pclCloud, float targetVoxelSize);

		/// <summary>
		/// 使用 CSF 算法提取地面点云
		/// </summary>
		/// <param name="ccCloud">输入点云</param>
		/// <returns>地面点云</returns>
		static ccCloudPtr apply_csf_ground_extraction(ccCloudPtr ccCloud);

		/// <summary>
		/// 使用 CSF 算法提取地面点云（PCL格式）
		/// </summary>
		/// <param name="pclCloud">输入 PCL 点云</param>
		/// <returns>地面点云</returns>
		static PCLCloudPtr apply_csf_ground_extraction(PCLCloudPtr pclCloud);

		/// <summary>
		/// 使用欧几里得聚类提取最大点云
		/// </summary>
		/// <param name="groundCloud">地面点云</param>
		/// <param name="euclideanClusterRadius">聚类半径</param>
		/// <returns>最大的聚类点云</returns>
		static PCLCloudPtr extract_max_cloud_by_euclidean_cluster(PCLCloudPtr groundCloud, float euclideanClusterRadius);

		/// <summary>
		/// 提取道路点云
		/// </summary>
		/// <param name="groundCloud">地面点云</param>
		/// <param name="searchRadius">搜索半径</param>
		/// <param name="angleThreshold">角度阈值</param>
		/// <param name="curvatureBaseThreshold">曲率阈值</param>
		/// <param name="pickSeedPoint">种子点</param>
		/// <returns>道路点云</returns>
		static PCLCloudPtr extract_road_points(PCLCloudPtr groundCloud,
			float searchRadius,
			float angleThreshold,
			float curvatureBaseThreshold,
			PCLPoint* pickSeedPoint = nullptr);

		/// <summary>
		/// 提取道路标线
		/// </summary>
		/// <param name="roadCloud">道路点云</param>
		/// <param name="resolution">分辨率</param>
		/// <param name="euclideanClusterRadius">欧几里得聚类半径</param>
		/// <param name="minNum">最小点数</param>
		/// <returns>道路标线点云列表</returns>
		static std::vector<PCLCloudXYZIPtr> extract_roadmarking(ccCloudPtr roadCloud,
			float resolution,
			double euclideanClusterRadius = 0.2,
			int minNum = 100);

		/// <summary>
		/// 匹配道路标线
		/// </summary>
		/// <param name="pclCloud">输入点云</param>
		/// <returns>匹配结果</returns>
		static PCLCloudPtr match_roadmarking(PCLCloudPtr pclCloud);

		/// <summary>
		/// 计算局部法向量（2D）
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="radius">搜索半径</param>
		/// <param name="normals">输出法向量</param>
		static void computeLocalNormal2D(const PCLCloudPtr& cloud, float radius, pcl::PointCloud<pcl::Normal>::Ptr normals);

		/// <summary>
		/// 旋转点云
		/// </summary>
		/// <param name="pclCloud">输入点云</param>
		/// <param name="now_v">当前方向</param>
		/// <param name="new_v">目标方向</param>
		/// <returns>旋转后的点云</returns>
		static PCLCloudPtr rotate_cloud(PCLCloudPtr pclCloud, const Eigen::Vector3f& now_v, const Eigen::Vector3f& new_v);

		/// <summary>
		/// 旋转点云（CloudCompare格式）
		/// </summary>
		/// <param name="P">输入点云</param>
		/// <param name="now_v">当前方向</param>
		/// <param name="new_v">目标方向</param>
		/// <returns>旋转后的点云</returns>
		static ccPointCloud* rotate_cloud(ccPointCloud* P, const CCVector3& now_v, const CCVector3& new_v);

		/// <summary>
		/// 对点云进行道路标线矢量化
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <returns>矢量化结果</returns>
		static ccHObject* apply_roadmarking_vectorization(ccPointCloud* cloud);
		
		/// <summary>
		/// 对点云进行道路标线矢量化
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <returns>矢量化结果</returns>
		static ccHObject* apply_roadmarking_vectorization(ccCloudPtr cloud);
		
		/// <summary>
		/// 对多个点云进行道路标线矢量化
		/// </summary>
		/// <param name="clouds">输入点云集合</param>
		/// <returns>矢量化结果</returns>
		static ccHObject* apply_roadmarking_vectorization(std::vector<PCLCloudPtr> clouds);

		/// <summary>
		/// 提取点云轮廓（alpha_shape）
		/// </summary>
		/// <param name="inputCloud">输入点云</param>
		/// <param name="alpha">Alpha 形状参数</param>
		/// <returns>轮廓点云</returns>
		static PCLCloudPtr extract_outline(const PCLCloudPtr& inputCloud, float alpha = 0.05);

		/// <summary>
		/// 提取欧几里得聚类
		/// </summary>
		/// <typeparam name="PointT">点类型</typeparam>
		/// <param name="inputCloud">输入点云</param>
		/// <param name="outputClusters">输出聚类集合</param>
		/// <param name="euclideanClusterRadius">聚类半径</param>
		/// <param name="minNum">最小点数</param>
		template<typename PointT>
		static void extract_euclidean_clusters(typename pcl::PointCloud<PointT>::Ptr inputCloud,
			std::vector<typename pcl::PointCloud<PointT>::Ptr>& outputClusters,
			double euclideanClusterRadius = 0.075,
			int minNum = 10);

		/// <summary>
		/// 应用默认强度值和可见性设置
		/// </summary>
		/// <param name="cloud">输入点云</param>
		static void apply_default_intensity_and_visible(ccCloudPtr cloud);
		
		/// <summary>
		/// 应用默认强度值和可见性设置
		/// </summary>
		/// <param name="cloud">输入点云</param>
		static void apply_default_intensity_and_visible(ccPointCloud* cloud);

		/// <summary>
		/// 根据强度值过滤点云
		/// </summary>
		/// <param name="inCloud">输入点云</param>
		/// <param name="lowerThreshold">强度下限</param>
		/// <param name="upperThreshold">强度上限</param>
		/// <param name="cloud_filtered">过滤后的点云</param>
		static void filter_cloud_by_intensity(ccPointCloud* inCloud, double lowerThreshold, double upperThreshold, ccPointCloud* cloud_filtered);

		/// <summary>
		/// 根据 Z 坐标过滤点云
		/// </summary>
		/// <param name="inCloud">输入点云</param>
		/// <param name="lowerThreshold">Z 坐标下限</param>
		/// <param name="upperThreshold">Z 坐标上限</param>
		/// <param name="cloud_filtered">过滤后的点云</param>
		static void filter_cloud_by_z(ccPointCloud* inCloud, double lowerThreshold, double upperThreshold, ccPointCloud* cloud_filtered);

		/// <summary>
		/// 从种子点开始，沿一条线段，按指定方向不断扩展，提取其中的线状部分
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

		/// <summary>
		/// 使用结构特征提取斑马线
		/// </summary>
		/// <param name="inputCloud">输入点云</param>
		/// <param name="m_glWindow">图形窗口，用于显示结果</param>
		static void extract_zebra_by_struct(ccPointCloud* inputCloud, ccGLWindowInterface* m_glWindow);

		/// <summary>
		/// 使用密度投影法，根据密度阈值提取斑马线的中心线段
		/// </summary>
		/// <param name="inputCloud">输入点云</param>
		/// <param name="binWidth">投影bin宽度（米），用于将点云分成等距的bin</param>
		/// <param name="densityThreshold">每个bin的最小点数阈值，超过该值认为是斑马线</param>
		/// <param name="minStripeLength">斑马线最小有效长度（未使用，待扩展）</param>
		/// <param name="m_glWindow">图形窗口，用于显示结果</param>
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

		/// <summary>
		/// 在指定位置周围聚类点云
		/// </summary>
		/// <param name="select_cloud">选择的点云</param>
		/// <param name="idx">中心点索引</param>
		/// <param name="radius">聚类半径</param>
		/// <param name="clustered_cloud">聚类后的点云</param>
		static void cluster_points_around_pos(ccPointCloud* select_cloud, unsigned idx,
			float radius, ccPointCloud& clustered_cloud);
	};
}
