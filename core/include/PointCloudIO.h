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

		static ccCloudPtr get_selected_cloud_from_DB(ccMainAppInterface* app);
		static ccCloudPtr convert_to_ccCloudPtr(ccPointCloud* p_cloud);
		static std::vector<ccHObject*> get_selected_clouds_from_DB(ccMainAppInterface* app);
		static void save_ccCloudPtr_to_DB(ccMainAppInterface* app, ccCloudPtr cloud);
		static ccCloudPtr convert_to_ccCloudPtr(ccHObject* ob, ccMainAppInterface* app = nullptr);
		static PCLCloudPtr convert_to_PCLCloudPtr(ccPointCloud* cloud);
		static PCLCloudPtr convert_to_PCLCloudPtr(ccCloudPtr cloud);
		static PCLCloudPtr convert_to_PCLCloudPtr(PCLCloudXYZIPtr cloud);
		static ccCloudPtr convert_to_ccCloudPtr(PCLCloudPtr pclCloud);

		static PCLCloudXYZIPtr convert_to_PCLCloudPtr_with_XYZI(ccCloudPtr cloud);
		static ccCloudPtr convert_to_ccCloudPtr_with_XYZI(PCLCloudXYZIPtr pclCloud);

		static int get_intensity_idx(ccCloudPtr cloud);
		static int get_intensity_idx(ccPointCloud* cloud);

		static ccPointCloud* get_ground_cloud(ccPointCloud* cloud);
	};
}
