#pragma once

#include "PointCloudIO.h"

namespace roadmarking
{
	class CloudProcess
	{
	public:
		static PCLOctreePtr buildOctree(PCLCloudPtr pclCloud, float targetVoxelSize);

		static ccCloudPtr CropBySparseCloud(ccCloudPtr ccCloud, PCLCloudPtr pclCloud, PCLOctreePtr octree = nullptr);

		static ccCloudPtr applyVoxelGridFiltering(ccCloudPtr ccCloud, float targetVoxelSize, PCLOctreePtr octree = nullptr);

		static PCLCloudPtr applyVoxelGridFiltering(PCLCloudPtr pclCloud, float targetVoxelSize);

		static ccCloudPtr applyCSFGroundExtraction(ccCloudPtr ccCloud);

		static PCLCloudPtr applyCSFGroundExtraction(PCLCloudPtr pclCloud);

		static PCLCloudPtr extractMaxCloudByEuclideanCluster(PCLCloudPtr groundCloud, float euclideanClusterRadius);

		static PCLCloudPtr extractRoadPoints(PCLCloudPtr groundCloud,
			float searchRadius,
			float angleThreshold,
			float curvatureBaseThreshold,
			PCLPoint* pickSeedPoint = nullptr);

		static std::vector<PCLCloudXYZIPtr> extractRoadMarking(ccCloudPtr roadCloud,
			float resolution,
			double euclideanClusterRadius = 0.2,
			int minNum = 100);

		static PCLCloudPtr matchRoadMarking(PCLCloudPtr pclCloud);

		static ccHObject* applyVectorization(ccCloudPtr cloud);

		static ccHObject* applyVectorization(std::vector<PCLCloudPtr> clouds);

		static PCLCloudPtr extractOutline(const PCLCloudPtr& inputCloud, float alpha = 0.05);

		static std::vector<PCLPoint> visualizeAndDrawPolyline(const PCLCloudPtr& inputCloud);

		template<typename PointT>
		static void extractEuclideanClusters(typename pcl::PointCloud<PointT>::Ptr inputCloud,
			std::vector<typename pcl::PointCloud<PointT>::Ptr>& outputClusters,
			double euclideanClusterRadius = 0.075,
			int minNum = 10);
	};
}
