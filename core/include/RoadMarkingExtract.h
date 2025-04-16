#pragma once
#pragma execution_character_set("utf-8")

#include <ccPointCloud.h>
#include "comm.h"

namespace roadmarking
{
	class RoadMarkingExtract
	{
	public:
		RoadMarkingExtract() = default;

		// �����˲�
		static ccCloudPtr applyVoxelGridFiltering(
			ccCloudPtr inputCloud,
			float voxelSize = 0.2f,
			ccMainAppInterface* m_app = nullptr);

		// CSF ��ȡ����
		static ccCloudPtr applyCSFGroundExtraction(
			ccCloudPtr inputCloud,
			ccMainAppInterface* m_app = nullptr);

		// ��ȡ�����ͨ��
		static ccCloudPtr extractLargestComponent(
			ccCloudPtr inputCloud,
			float clusterRadius = 0.3f,
			ccMainAppInterface* m_app = nullptr);

		// ��ȡ��·����
		static ccCloudPtr extractRoadPoints(
			ccCloudPtr inputCloud,
			const CCVector3& seedPoint,
			float curvatureThreshold = 0.010f,
			float angleThreshold = cos(0.5 * M_PI / 180.0f),
			float searchRadius = 0.3f,
			ccMainAppInterface* m_app = nullptr);

		// ʸ����
		static ccHObject* applyVectorization(
			ccCloudPtr inputCloud,
			ccMainAppInterface* m_app = nullptr);

		// ȫ�Զ���ȡ
		static void automaticExtraction(
			ccCloudPtr inputCloud,
			ccMainAppInterface* m_app = nullptr,
			float voxelSize = 0.2f,
			float clusterRadius = 0.3f,
			float curvatureThreshold = 0.01f,
			float angleThreshold = 5.0f,
			float searchRadius = 0.3f
			);

		static void applyDefaultIntensityDisplay(ccCloudPtr cloud);
	};
};
