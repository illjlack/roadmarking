#pragma once
#pragma execution_character_set("utf-8")

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccHObject.h>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>

#include "comm.h"


namespace roadmarking
{
	class PointCloudIO
	{
	public:
		PointCloudIO() = default;
		~PointCloudIO() = default;

		static ccCloudPtr getSelectedCloud(ccMainAppInterface* app);
		static void saveCloudToDB(ccMainAppInterface* app, ccCloudPtr cloud);
		static ccCloudPtr convertToCCCloud(ccHObject* ob, ccMainAppInterface* app = nullptr);
		static PCLCloudPtr convertToPCLCloud(ccCloudPtr cloud);
		static PCLCloudPtr convertToPCLCloud(PCLCloudXYZIPtr cloud);
		static ccCloudPtr convertToCCCloud(PCLCloudPtr pclCloud);

		static PCLCloudXYZIPtr convertToPCLCloudXYZI(ccCloudPtr cloud);
		static ccCloudPtr convertToCCCloudXYZI(PCLCloudXYZIPtr pclCloud);
	};
}
