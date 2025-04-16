#include "RoadMarkingExtract.h"
#include <QMap>
#include <QVariant>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QDebug>
#include <cmath>

#include "CloudProcess.h"

using namespace roadmarking;


ccCloudPtr RoadMarkingExtract::applyVoxelGridFiltering(ccCloudPtr inputCloud, float voxelSize, ccMainAppInterface* m_app)
{
	Timer timer("�����˲�");
	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("δѡ�����");
		return nullptr;
	}

	ccCloudPtr filteredCloud = CloudProcess::applyVoxelGridFiltering(inputCloud, voxelSize);
	filteredCloud->setName("downsampledCloud");

	if (m_app)
		timer.elapsed(m_app, "�����˲����");
	return filteredCloud;
}

ccCloudPtr RoadMarkingExtract::applyCSFGroundExtraction(ccCloudPtr inputCloud, ccMainAppInterface* m_app)
{
	Timer timer("CSF ��ȡ�������");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("δѡ�����");
		return nullptr;
	}

	ccCloudPtr filteredCloud = CloudProcess::applyCSFGroundExtraction(inputCloud);
	if (!filteredCloud)
	{
		if (m_app) m_app->dispToConsole("CSF ����ʧ��");
		return nullptr;
	}

	filteredCloud->setName("groundCloud");

	if (m_app)
		timer.elapsed(m_app, "CSF �������");
	return filteredCloud;
}

ccCloudPtr RoadMarkingExtract::extractLargestComponent(ccCloudPtr inputCloud, float clusterRadius, ccMainAppInterface* m_app)
{
	Timer timer("ŷʽ������ȡ�����ͨ��");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("δѡ�����");
		return nullptr;
	}

	PCLCloudPtr pclCloud = PointCloudIO::convertToPCLCloud(inputCloud);
	PCLCloudPtr filteredCloud = CloudProcess::extractMaxCloudByEuclideanCluster(pclCloud, clusterRadius);
	if (!filteredCloud)
	{
		if (m_app) m_app->dispToConsole("�Ҳ��������ͨ��");
		return nullptr;
	}

	ccCloudPtr ccFilteredCloud = CloudProcess::CropBySparseCloud(inputCloud, filteredCloud);
	ccFilteredCloud->setName("LargestRoadComponent");

	if (m_app)
		timer.elapsed(m_app, "�ҵ������ͨ��");
	return ccFilteredCloud;
}

void RoadMarkingExtract::automaticExtraction(ccCloudPtr inputCloud,
	ccMainAppInterface* m_app, float voxelSize,
	float clusterRadius, float curvatureThreshold,
	float angleThreshold, float searchRadius
	)
{
	Timer timer("ȫ�Զ���ȡ");

	if (!inputCloud)
	{
		if (m_app) m_app->dispToConsole("δѡ�����");
		return;
	}

	PCLCloudPtr pclCloud = PointCloudIO::convertToPCLCloud(inputCloud);
	if (m_app) timer.restart(m_app, "ת��pcl����");

	// 2.================================�����˲�
	PCLCloudPtr voxelCloud = CloudProcess::applyVoxelGridFiltering(pclCloud, voxelSize);
	if (m_app) timer.restart(m_app, "�����˲��ɹ�");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convertToCCCloud(voxelCloud);
		bugCloud->setName("VoxelGridFiltering");
		applyDefaultIntensityDisplay(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	// 3.================================csf
	ccCloudPtr groundCloud = CloudProcess::applyCSFGroundExtraction(PointCloudIO::convertToCCCloud(voxelCloud));
	if (!groundCloud)
	{
		if (m_app) timer.elapsed(m_app, "CSF ����ʧ��");
		return;
	}
	if (m_app) timer.restart(m_app, "CSF ������ɣ���ȡ���������");

#ifdef DEBUG
	{
		if (groundCloud)
		{
			groundCloud->setName("groundCloud");
			applyDefaultIntensityDisplay(groundCloud);
			inputCloud->addChild(groundCloud.release());
		}
	}
#endif // DEBUG

	// 4.=====================================ŷʽ���࣬��ȡ�����ͨ��
	PCLCloudPtr largestClusterCloud = CloudProcess::extractMaxCloudByEuclideanCluster(PointCloudIO::convertToPCLCloud(groundCloud), clusterRadius);
	if (!largestClusterCloud)
	{
		if (m_app) timer.elapsed(m_app, "�Ҳ��������ͨ��");
		return;
	}
	if (m_app) timer.restart(m_app, "ŷʽ���࣬�ҵ������ͨ��");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convertToCCCloud(largestClusterCloud);
		bugCloud->setName("euclideanClusterCloud");
		applyDefaultIntensityDisplay(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	// 5.====================================�����㷨��ȡ��·����
	PCLCloudPtr roadCloud = CloudProcess::extractRoadPoints(largestClusterCloud, searchRadius, angleThreshold, curvatureThreshold);
	if (m_app) timer.restart(m_app, "��·������ȡ���");

#ifdef DEBUG
	{
		ccCloudPtr bugCloud = PointCloudIO::convertToCCCloud(roadCloud);
		bugCloud->setName("roadCloud");
		applyDefaultIntensityDisplay(bugCloud);
		inputCloud->addChild(bugCloud.release());
	}
#endif // DEBUG

	ccCloudPtr oRoadCloud = CloudProcess::CropBySparseCloud(inputCloud, roadCloud);
	if (m_app) timer.restart(m_app, "�ܼ���·������ȡ���");

#ifdef DEBUG
	{
		oRoadCloud->setName("oroadCloud");
		applyDefaultIntensityDisplay(oRoadCloud);
		inputCloud->addChild(oRoadCloud.release());
	}
#endif // DEBUG

	auto markingClouds = CloudProcess::extractRoadMarking(oRoadCloud, 0.03);
	if (m_app) timer.restart(m_app, "·�������ȡ�ɹ�");

#ifdef DEBUG
	{
		ccHObject* parentNode = new ccHObject("Clusters_Parent");
		size_t clusterId = 0;
		for (auto& cluster : markingClouds)
		{
			ccCloudPtr markingCloud = PointCloudIO::convertToCCCloudXYZI(cluster);
			markingCloud->setName(QString("Cluster_%1").arg(clusterId++));
			applyDefaultIntensityDisplay(markingCloud);
			parentNode->addChild(markingCloud.release());
		}
		parentNode->setVisible(true);
		inputCloud->addChild(parentNode);
	}
#endif // DEBUG


	std::vector<PCLCloudPtr> pclClouds;
	for (auto markingCloud : markingClouds)
	{
		pclClouds.push_back(PointCloudIO::convertToPCLCloud(markingCloud));
	}

	auto markings = CloudProcess::applyVectorization(pclClouds);
	markings->setName("roadmarking");
	inputCloud->addChild(markings);

	if (m_app) timer.elapsed(m_app, "�������,һ����ʱ");
}

void RoadMarkingExtract::applyDefaultIntensityDisplay(ccCloudPtr cloud)
{
	if (!cloud)
		return;

	// ������Ϊ��intensity���ı����ֶ�
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

	// ����ҵ���ǿ�ȱ����ֶ�
	if (sfIdx >= 0)
	{
		// ���øñ����ֶ���Ϊ��ɫ��ʾ
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);  // ��ʾ�����ֶ�
		cloud->showColors(true);  // ������ɫ��ʾ
	}
	else
	{
		// ���û��ǿ�ȱ����ֶΣ�����Ĭ����Ϊ
		cloud->showSF(false);
		cloud->showColors(false);
	}
	cloud->setVisible(true);
}
