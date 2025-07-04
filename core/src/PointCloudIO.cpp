#include "PointCloudIO.h"
#include "comm.h"
#include "CloudProcess.h"
#include <ccOctree.h>
#include <ccProgressDialog.h>
#include <QMessageBox>

using namespace roadmarking;

ccCloudPtr PointCloudIO::get_selected_cloud_from_DB(ccMainAppInterface* p_app)
{
	const ccHObject::Container& selectedEntities = p_app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		p_app->dispToConsole("请先选择至少一个点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return nullptr;
	}
	for (ccHObject* ent : selectedEntities)
	{
		if (ent->isA(CC_TYPES::POINT_CLOUD))
		{
			// 这里不管理当前点云，使用空删除器
			return ccCloudPtr(static_cast<ccPointCloud*>(ent), [](ccPointCloud*) {});
		}
	}
	p_app->dispToConsole("选择的实体不是点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	return nullptr;
}

ccCloudPtr PointCloudIO::convert_to_ccCloudPtr(ccPointCloud* p_cloud)
{
	// 这里不管理当前点云，使用空删除器
	return ccCloudPtr(static_cast<ccPointCloud*>(p_cloud), [](ccPointCloud*) {});
}

std::vector<ccHObject*> PointCloudIO::get_selected_clouds_from_DB(ccMainAppInterface* app)
{
	const ccHObject::Container& selectedEntities = app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		app->dispToConsole("请先选择至少一个点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return {};
	}
	return selectedEntities;
}

ccCloudPtr PointCloudIO::convert_to_ccCloudPtr(ccHObject* ob, ccMainAppInterface* app)
{
	if (ob == nullptr)return nullptr;
	if (ob->isA(CC_TYPES::POINT_CLOUD))
	{
		return ccCloudPtr(static_cast<ccPointCloud*>(ob), [](ccPointCloud*) {});
	}
	if(app)app->dispToConsole("选择的实体不是点云", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	return nullptr;
}

void PointCloudIO::save_ccCloudPtr_to_DB(ccMainAppInterface* app, ccCloudPtr cloud)
{
	// 完全转移点云所有权，交由主界面管理
	ccPointCloud* cccloud = cloud.release();

	app->addToDB(cccloud);
	app->refreshAll();
}

PCLCloudPtr PointCloudIO::convert_to_PCLCloudPtr(ccPointCloud* cloud)
{
	PCLCloudPtr pclCloud(new PCLCloud);
	size_t numPoints = cloud->size();
	pclCloud->resize(numPoints);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		const CCVector3& point = *cloud->getPoint(i);
		pclCloud->points[i] = pcl::PointXYZ(point.x, point.y, point.z);
	}

	return pclCloud;
}


PCLCloudPtr PointCloudIO::convert_to_PCLCloudPtr(ccCloudPtr cloud)
{
	return convert_to_PCLCloudPtr(cloud.get());
}


PCLCloudPtr PointCloudIO::convert_to_PCLCloudPtr(PCLCloudXYZIPtr cloud)
{
	PCLCloudPtr pclCloud(new PCLCloud);
	size_t numPoints = cloud->size();
	pclCloud->resize(numPoints);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		const PCLPointXYZI& point = (*cloud)[i];
		pclCloud->points[i] = pcl::PointXYZ(point.x, point.y, point.z);
	}

	return pclCloud;
}


ccCloudPtr PointCloudIO::convert_to_ccCloudPtr(PCLCloudPtr pclCloud)
{
	ccCloudPtr ccCloud(new ccPointCloud());
	size_t numPoints = pclCloud->size();
	ccCloud->reserve(numPoints);

	std::vector<CCVector3> tempPoints(numPoints);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		tempPoints[i] = CCVector3(pclCloud->points[i].x, pclCloud->points[i].y, pclCloud->points[i].z);
	}

	for (size_t i = 0; i < numPoints; ++i)
	{
		ccCloud->addPoint(tempPoints[i]);
	}
	return ccCloud;
}

PCLCloudXYZIPtr PointCloudIO::convert_to_PCLCloudPtr_with_XYZI(ccCloudPtr cloud)
{
	PCLCloudXYZIPtr pclCloud(new PCLCloudXYZI);
	size_t numPoints = cloud->size();
	pclCloud->resize(numPoints);

	int intensitySFIndex = PointCloudIO::get_intensity_idx(cloud);
	ccScalarField* intensitySF = (intensitySFIndex >= 0) ? static_cast<ccScalarField*>(cloud->getScalarField(intensitySFIndex)) : nullptr;
	float defaultIntensity = 0.0f;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(numPoints); ++i)
	{
		const CCVector3* point = cloud->getPoint(i);
		pcl::PointXYZI& pclPoint = pclCloud->points[i];
		pclPoint.x = point->x;
		pclPoint.y = point->y;
		pclPoint.z = point->z;
		pclPoint.intensity = (intensitySF) ? intensitySF->getValue(i) : defaultIntensity;
	}

	return pclCloud;
}


ccCloudPtr PointCloudIO::convert_to_ccCloudPtr_with_XYZI(PCLCloudXYZIPtr pclCloud)
{
	ccCloudPtr ccCloud(new ccPointCloud());

	ccScalarField* intensitySF = nullptr;
	intensitySF = new ccScalarField("Intensity");
	ccCloud->addScalarField(intensitySF);

	for (const auto& pclPoint : pclCloud->points)
	{
		CCVector3 ccPoint(pclPoint.x, pclPoint.y, pclPoint.z);
		ccCloud->addPoint(ccPoint);
		intensitySF->addElement(static_cast<float>(pclPoint.intensity));
	}
	intensitySF->computeMinAndMax();
	return ccCloud;
}

int PointCloudIO::get_intensity_idx(ccCloudPtr cloud)
{
	return get_intensity_idx(cloud.get());
}

int PointCloudIO::get_intensity_idx(ccPointCloud* cloud)
{
	if (!cloud)
		return -1;

	int sfIdx = -1;
	const int sfCount = cloud->getNumberOfScalarFields();
	for (int i = 0; i < sfCount; ++i)
	{
		if (QString::fromStdString(cloud->getScalarField(i)->getName()).contains("intensity", Qt::CaseInsensitive))
		{
			sfIdx = i;
			break;
		}
	}
	return sfIdx;
}

void PointCloudIO::apply_intensity(ccCloudPtr cloud)
{
	apply_intensity(cloud.get());
}

void PointCloudIO::apply_intensity(ccPointCloud* cloud)
{
	if (!cloud)
		return;

	int sfIdx = get_intensity_idx(cloud);

	// 如果找到了强度标量字段
	if (sfIdx >= 0)
	{
		// 设置该标量字段作为颜色显示
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->getScalarField(sfIdx)->computeMinAndMax();
		cloud->showSF(true);  // 显示标量字段
	}
	else
	{
		// 如果没有强度标量字段，保持默认行为
		cloud->showSF(false);
	}
	cloud->setVisible(true);
}

ccPointCloud* PointCloudIO::get_ground_cloud(ccPointCloud* cloud)
{
	QString name = cloud->getName() + "_ground_part";
	for (int i = 0; i < cloud->getChildrenNumber(); i++)
	{
		if(cloud->getChild(i)->getName() == name)return static_cast<ccPointCloud*>(cloud->getChild(i));
	}
	ccCloudPtr ground = CloudProcess::apply_csf_ground_extraction(PointCloudIO::convert_to_ccCloudPtr(cloud));
	cloud->addChild(ground.release());
	ground->setName(name);
	return ground.get();
}

ccOctree::Shared PointCloudIO::get_octree(ccPointCloud* cloud)
{
	if (!cloud)
		return nullptr;

	// 如果点云已经有八叉树，直接返回
	if (cloud->getOctree())
	{
		return cloud->getOctree();
	}

	// 如果没有八叉树，创建进度对话框并构建八叉树
	ccProgressDialog progressDlg(false);
	progressDlg.setWindowTitle("构建八叉树");
	progressDlg.setAutoClose(true);
	
	if (!cloud->computeOctree(&progressDlg))
	{
		// 构建失败
		QMessageBox::critical(nullptr,
			QStringLiteral("错误"),
			QStringLiteral("八叉树构建失败！"));
		return nullptr;
	}

	return cloud->getOctree();
}

void PointCloudIO::save_height_as_scalar(ccPointCloud* cloud)
{
	if (!cloud)
		return;
	
	if(get_height_idx(cloud)>=0)return;

	// 创建高度标量字段
	ccScalarField* heightSF = new ccScalarField("Height");
	cloud->addScalarField(heightSF); // 这里会resize

	// 获取点云中的点数量
	size_t numPoints = cloud->size();

	// 为每个点计算高度（Z坐标）并添加到标量字段
	for (size_t i = 0; i < numPoints; ++i)
	{
		const CCVector3* point = cloud->getPoint(i);
		heightSF->setValue(i, static_cast<float>(point->z));
	}

	// 计算标量字段的最小值和最大值
	heightSF->computeMinAndMax();

	// 设置为当前显示的标量字段
	cloud->setCurrentDisplayedScalarField(cloud->getNumberOfScalarFields() - 1);
	cloud->showSF(true);
	cloud->setVisible(true);
}

void PointCloudIO::save_height_as_scalar(ccCloudPtr cloud)
{
	save_height_as_scalar(cloud.get());
}

void PointCloudIO::apply_height_as_scalar(ccPointCloud* cloud)
{
	if (!cloud)
		return;

	int sfIdx = get_height_idx(cloud);

	// 如果找到了高度标量字段
	if (sfIdx >= 0)
	{
		// 设置该标量字段作为颜色显示
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->getScalarField(sfIdx)->computeMinAndMax();
		cloud->showSF(true);  // 显示标量字段
	}
	else
	{
		// 如果没有高度标量字段，创建并应用
		save_height_as_scalar(cloud);
	}
	cloud->setVisible(true);
}

void PointCloudIO::apply_height_as_scalar(ccCloudPtr cloud)
{
	apply_height_as_scalar(cloud.get());
}

int PointCloudIO::get_height_idx(ccPointCloud* cloud)
{
	if (!cloud)
		return -1;

	int sfIdx = -1;
	const int sfCount = cloud->getNumberOfScalarFields();
	for (int i = 0; i < sfCount; ++i)
	{
		QString sfName = QString::fromStdString(cloud->getScalarField(i)->getName());
		if (sfName.contains("height", Qt::CaseInsensitive) || 
			sfName.contains("elevation", Qt::CaseInsensitive) ||
			sfName.contains("高程", Qt::CaseInsensitive))
		{
			sfIdx = i;
			break;
		}
	}
	return sfIdx;
}

int PointCloudIO::get_height_idx(ccCloudPtr cloud)
{
	return get_height_idx(cloud.get());
}

// ==================== 密度相关方法 ====================

void PointCloudIO::save_density_as_scalar(ccPointCloud* cloud, const std::vector<unsigned>& densityArray)
{
	if (!cloud || densityArray.empty())
		return;

	// 检查密度数组大小是否与点云点数匹配
	if (densityArray.size() != cloud->size())
	{
		// 如果大小不匹配，输出警告并返回
		return;
	}

	// 创建密度标量字段
	ccScalarField* densitySF = new ccScalarField("Density");
	cloud->addScalarField(densitySF);

	// 将密度值添加到标量字段
	for (size_t i = 0; i < densityArray.size(); ++i)
	{
		densitySF->setValue(i, static_cast<float>(densityArray[i]));
	}
	// 计算标量字段的最小值和最大值
	densitySF->computeMinAndMax();

	// 设置为当前显示的标量字段
	cloud->setCurrentDisplayedScalarField(cloud->getNumberOfScalarFields() - 1);
	cloud->showSF(true);
	cloud->setVisible(true);
}

void PointCloudIO::save_density_as_scalar(ccCloudPtr cloud, const std::vector<unsigned>& densityArray)
{
	save_density_as_scalar(cloud.get(), densityArray);
}

void PointCloudIO::apply_density_as_scalar(ccPointCloud* cloud)
{
	if (!cloud)
		return;

	int sfIdx = get_density_idx(cloud);

	// 如果找到了密度标量字段
	if (sfIdx >= 0)
	{
		// 设置该标量字段作为颜色显示
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->getScalarField(sfIdx)->computeMinAndMax();
		cloud->showSF(true);  // 显示标量字段
	}
	else
	{
		// 如果没有密度标量字段，隐藏标量字段显示
		cloud->showSF(false);
	}
	cloud->setVisible(true);
}

void PointCloudIO::apply_density_as_scalar(ccCloudPtr cloud)
{
	apply_density_as_scalar(cloud.get());
}

int PointCloudIO::get_density_idx(ccPointCloud* cloud)
{
	if (!cloud)
		return -1;

	int sfIdx = -1;
	const int sfCount = cloud->getNumberOfScalarFields();
	for (int i = 0; i < sfCount; ++i)
	{
		if (QString::fromStdString(cloud->getScalarField(i)->getName()).contains("density", Qt::CaseInsensitive))
		{
			sfIdx = i;
			break;
		}
	}
	return sfIdx;
}

int PointCloudIO::get_density_idx(ccCloudPtr cloud)
{
	return get_density_idx(cloud.get());
}
