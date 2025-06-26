#include "algorithms/PointCloudDivider.h"
#include "PointCloudIO.h"
#include <omp.h>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <DgmOctree.h>
using namespace roadmarking;


// ===========================================================================================================================
// DgmOctree(莫顿编码)的操作接口
// ===========================================================================================================================

/// <summary>
/// 获取指定code和level对应的cell的XY平面边界框
/// </summary>
/// <param name="octree">八叉树指针</param>
/// <param name="code">莫顿编码</param>
/// <param name="level">八叉树层级</param>
/// <param name="xmin">输出：cell的X最小值</param>
/// <param name="ymin">输出：cell的Y最小值</param>
/// <param name="cellSize">输出：cell的尺寸</param>
void getCellXYBounds(CCCoreLib::DgmOctree* octree, CCCoreLib::DgmOctree::CellCode code, unsigned char level, 
					float& xmin, float& ymin, float& cellSize)
{
	cellSize = octree->getCellSize(level);
	Tuple3i pos;
	octree->getCellPos(code, level, pos, false);
	CCVector3 bbMin = octree->getOctreeMins();  // AABB 起点

	xmin = bbMin.x + pos.x * cellSize;
	ymin = bbMin.y + pos.y * cellSize;
}

/// <summary>
/// 获取指定code和level对应的cell的八个子cell的莫顿编码
/// </summary>
/// <param name="octree">八叉树指针</param>
/// <param name="code">父cell的莫顿编码</param>
/// <param name="level">父cell的八叉树层级</param>
/// <returns>八个子cell的莫顿编码</returns>
std::vector<CCCoreLib::DgmOctree::CellCode> getChildCellCodes(CCCoreLib::DgmOctree* octree, CCCoreLib::DgmOctree::CellCode code, unsigned char level)
{
	std::vector<CCCoreLib::DgmOctree::CellCode> childCodes;
	
	// 子cell的层级
	unsigned char childLevel = level + 1;
	
	// 遍历8个子cell
	for (int i = 0; i < 8; ++i)
	{
		// 计算子cell的莫顿编码
		// 子cell的编码 = 父cell编码 * 8 + 子cell索引(0-7)
		CCCoreLib::DgmOctree::CellCode childCode = (code << 3) + i;
		childCodes.push_back(childCode);
	}
	
	return childCodes;
}


/// <summary>
/// 判断指定 cell（由整数体素坐标 pos 与 level 定义）是否为空（不包含任何点）
/// 使用 getCellPopulation 方法判断是否存在点，无需解码 Morton 编码
/// </summary>
/// <param name="octree">八叉树指针</param>
/// <param name="pos">体素坐标（三维整数）</param>
/// <param name="level">八叉树层级</param>
/// <returns>true 表示该 cell 为空；false 表示该 cell 中包含点</returns>
bool isCellEmpty(CCCoreLib::DgmOctree* octree, CCCoreLib::DgmOctree::CellCode code, unsigned char level)
{
	unsigned char bitShift = CCCoreLib::DgmOctree::GET_BIT_SHIFT(level);
	code >>= bitShift;
	unsigned cellIndex = octree->getCellIndex(code, bitShift);
	//check that cell exists!
	if (cellIndex < octree->getNumberOfProjectedPoints())
	{
		return false;
	}
	return true;
}


/// <summary>
/// 获取指定cell内所有点云的下标
/// </summary>
/// <param name="octree">八叉树指针</param>
/// <param name="code">cell的莫顿编码</param>
/// <param name="level">八叉树层级</param>
/// <param name="pointIndices">输出：cell内所有点云的下标向量</param>
void getPointIndicesInCell(CCCoreLib::DgmOctree* octree, CCCoreLib::DgmOctree::CellCode code, unsigned char level, std::vector<unsigned>& pointIndices)
{	
	// 创建ReferenceCloud对象来存储cell内的点
	CCCoreLib::ReferenceCloud* subset = new CCCoreLib::ReferenceCloud(octree->associatedCloud());
	
	// 调用getPointsInCell方法获取cell内的点
	bool success = octree->getPointsInCell(code, level, subset, false, true);
	
	if (success && subset->size() > 0)
	{
		// 预分配空间以提高性能, 追加
		pointIndices.reserve(pointIndices.size() + subset->size());
		
		// 获取所有点的下标
		for (unsigned i = 0; i < subset->size(); ++i)
		{
			pointIndices.push_back(subset->getPointGlobalIndex(i));
		}
	}
	
	// 清理内存
	delete subset;
}



// ===========================================================================================================================
// 几何关系判断工具Cell与多边形(复杂面)的xy平面关系
// ===========================================================================================================================

/// <summary>
/// Cell与多边形的关系枚举
/// </summary>
enum class CellRelation { 
	OUTSIDE,    ///< Cell完全在多边形外
	INSIDE,     ///< Cell完全在多边形内
	INTERSECT   ///< Cell与多边形边界相交
};

/// <summary>
/// 2D向量结构
/// </summary>
struct Vec2 {
	float x, y;
	Vec2(float _x, float _y) : x(_x), y(_y) {}
};

/// <summary>
/// 判断两条线段是否相交
/// </summary>
/// <param name="p1">第一条线段的起点</param>
/// <param name="p2">第一条线段的终点</param>
/// <param name="q1">第二条线段的起点</param>
/// <param name="q2">第二条线段的终点</param>
/// <returns>true表示相交，false表示不相交</returns>
bool segmentsIntersect(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2)
{
	auto cross = [](const Vec2& a, const Vec2& b) {
		return a.x * b.y - a.y * b.x;
	};

	auto subtract = [](const Vec2& a, const Vec2& b) {
		return Vec2(a.x - b.x, a.y - b.y);
	};

	Vec2 r = subtract(p2, p1);
	Vec2 s = subtract(q2, q1);
	Vec2 qp = subtract(q1, p1);
	float rxs = cross(r, s);
	float qpxr = cross(qp, r);

	if (rxs == 0 && qpxr == 0) return false; // 共线
	if (rxs == 0) return false;              // 平行不相交

	float t = cross(qp, s) / rxs;
	float u = cross(qp, r) / rxs;

	return (t >= 0 && t <= 1) && (u >= 0 && u <= 1);
}

/// <summary>
/// 判断点是否在多边形内（射线法）
/// </summary>
/// <param name="polygon">多边形顶点集合</param>
/// <param name="px">点的X坐标</param>
/// <param name="py">点的Y坐标</param>
/// <returns>true表示点在多边形内，false表示在多边形外</returns>
bool pointInPolygon(const std::vector<CCVector3>& polygon, float px, float py)
{
	bool inside = false;
	size_t n = polygon.size();

	// j指向上条边终点
	for (size_t i = 0, j = n - 1; i < n; j = i++)
	{
		float xi = polygon[i].x, yi = polygon[i].y;
		float xj = polygon[j].x, yj = polygon[j].y;

		// Check if the point is exactly on the edge
		if (((yi == py && xi == px) || (yj == py && xj == px)) ||
			((yi > py) != (yj > py)))  // Point is between the y values of the edge
		{
			// Calculate the x-coordinate of the intersection of the ray with the edge
			if (px < (xj - xi) * (py - yi) / (yj - yi + 1e-6f) + xi)
				inside = !inside;
		}
	}
	return inside;
}

/// <summary>
/// 判断cell与多边形的关系（仅考虑XY平面）
/// 优化算法：对于正方形cell，使用更简单高效的判断方法
/// 1. 首先检查cell的四个角点是否都在多边形内
/// 2. 如果四个角点都在多边形内，则cell完全在多边形内
/// 3. 如果四个角点都在多边形外，需要进一步检查多边形是否在cell内
/// 4. 否则cell与多边形边界相交
/// </summary>
/// <param name="polygon">多边形顶点集合</param>
/// <param name="xmin">cell的X最小值</param>
/// <param name="ymin">cell的Y最小值</param>
/// <param name="cellSize">cell的尺寸</param>
/// <returns>CellRelation枚举值，表示cell与多边形的关系</returns>
CellRelation classifyCellWithPolygonXY(const std::vector<CCVector3>& polygon, float xmin, float ymin, float cellSize)
{
	// 构建cell的四个角点
	Vec2 corners[4] = {
		{xmin, ymin},                    // 左下角
		{xmin + cellSize, ymin},         // 右下角
		{xmin + cellSize, ymin + cellSize}, // 右上角
		{xmin, ymin + cellSize}          // 左上角
	};

	// 检查四个角点是否在多边形内
	int insideCount = 0;
	for (int i = 0; i < 4; ++i) {
		if (pointInPolygon(polygon, corners[i].x, corners[i].y)) {
			insideCount++;
		}
	}

	// 根据角点位置判断cell与多边形的关系
	if (insideCount == 4) {
		// 四个角点都在多边形内，cell完全在多边形内
		return CellRelation::INSIDE;
	} else if (insideCount == 0) {
		// 四个角点都在多边形外，需要检查多边形是否在cell内
		// 检查多边形的所有顶点是否都在cell内
		bool allPolygonPointsInCell = true;
		for (const auto& vertex : polygon) {
			if (vertex.x < xmin || vertex.x > xmin + cellSize || 
				vertex.y < ymin || vertex.y > ymin + cellSize) {
				allPolygonPointsInCell = false;
				break;
			}
		}
		
		if (allPolygonPointsInCell) {
			// 多边形完全在cell内，属于相交情况
			return CellRelation::INTERSECT;
		} else {
			// 多边形和cell完全分离
			return CellRelation::OUTSIDE;
		}
	} else {
		// 部分角点在多边形内，部分在外，cell与多边形边界相交
		return CellRelation::INTERSECT;
	}
}

/// <summary>
/// 使用code+level判断cell与多边形的关系
/// 这是八叉树优化的核心函数，用于快速判断cell是否在多边形内
/// </summary>
/// <param name="octree">八叉树指针</param>
/// <param name="code">莫顿编码</param>
/// <param name="level">八叉树层级</param>
/// <param name="polygon">多边形顶点集合</param>
/// <returns>CellRelation枚举值，表示cell与多边形的关系</returns>
CellRelation classifyCellByCodeLevel(CCCoreLib::DgmOctree* octree, CCCoreLib::DgmOctree::CellCode code, 
									unsigned char level, const std::vector<CCVector3>& polygon)
{
	float xmin, ymin, cellSize;
	getCellXYBounds(octree, code, level, xmin, ymin, cellSize);
	return classifyCellWithPolygonXY(polygon, xmin, ymin, cellSize);
}

void PointCloudDivider::cropWithPolygon(ccPointCloud* cloud, const std::vector<CCVector3d>& polygon_points, ccPointCloud* cloud_cropped)
{
	std::vector<ccPointCloud*> clouds;
	clouds.push_back(cloud);
	cropWithPolygon(clouds, polygon_points, cloud_cropped);
}

void PointCloudDivider::cropWithPolygon(
	const std::vector<ccPointCloud*>& clouds,
	const std::vector<CCVector3d>& polygon_points,
	ccPointCloud* cloud_cropped)
{
	if (clouds.empty() || !cloud_cropped || polygon_points.size() < 3)
		return;

	// ==================== 1. 构建 polygon + AABB ====================
	// 一次性转换多边形格式，避免重复转换
	std::vector<CCVector3> polygon_cc;
	std::vector<CCVector3d> polygon_cc_double = polygon_points;
	
	for (const auto& pt : polygon_points)
	{
		polygon_cc.push_back(CCVector3(static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z)));
	}

	double minX = DBL_MAX, maxX = -DBL_MAX;
	double minY = DBL_MAX, maxY = -DBL_MAX;

	for (const auto& pt : polygon_points)
	{
		minX = std::min(minX, pt.x);
		maxX = std::max(maxX, pt.x);
		minY = std::min(minY, pt.y);
		maxY = std::max(maxY, pt.y);
	}

	// ==================== 2. 准备输出 scalar field ====================
	cloud_cropped->addScalarField("intensity");
	auto _sf = cloud_cropped->getScalarField(cloud_cropped->getScalarFieldIndexByName("intensity"));

	// ==================== 3. 并行裁剪所有点云 ====================
	for (auto cloud : clouds)
	{
		int sfIdx = PointCloudIO::get_intensity_idx(cloud);
		if (sfIdx < 0) continue;

		auto sf = cloud->getScalarField(sfIdx);
		if (!sf) continue;

		// ========== 3.1 检查是否有八叉树可用 ==========
		ccOctree::Shared octree = PointCloudIO::get_octree(cloud);
		bool useOctree = (octree != nullptr);

		// 不知道哪里有问题
		useOctree = false;
		if (useOctree) {
			// ========== 3.2 使用八叉树体素级优化 ==========
			std::vector<unsigned> indices;
			octreeBasedPolygonCrop(cloud, octree, polygon_cc, indices);
			
			// 根据索引添加点和标量值
			for (unsigned idx : indices) {
				cloud_cropped->addPoint(*cloud->getPoint(idx));
				_sf->addElement(sf->getValue(idx));
			}
		} else {
			// ========== 3.3 传统逐点处理（无八叉树） ==========
			std::vector<unsigned> indices;
			pointBasedPolygonCrop(cloud, polygon_cc, indices);
			
			// 根据索引添加点和标量值
			for (unsigned idx : indices) {
				cloud_cropped->addPoint(*cloud->getPoint(idx));
				_sf->addElement(sf->getValue(idx));
			}
		}
	}

	// ==================== 4. 计算标量字段统计信息 ====================
	_sf->computeMinAndMax();
}

void PointCloudDivider::pointBasedPolygonCrop(
	ccPointCloud* cloud,
	const std::vector<CCVector3>& polygon,
	std::vector<unsigned>& result_indices)
{
	// 计算多边形的AABB
	double minX = DBL_MAX, maxX = -DBL_MAX;
	double minY = DBL_MAX, maxY = -DBL_MAX;
	for (const auto& pt : polygon) {
		minX = std::min(minX, static_cast<double>(pt.x));
		maxX = std::max(maxX, static_cast<double>(pt.x));
		minY = std::min(minY, static_cast<double>(pt.y));
		maxY = std::max(maxY, static_cast<double>(pt.y));
	}

	const size_t pointCount = cloud->size();

	// 使用OpenMP并行处理
	omp_set_num_threads(8);
#pragma omp parallel
	{
		std::vector<unsigned> local_indices;
#pragma omp for schedule(static)
		for (int i = 0; i < static_cast<int>(pointCount); ++i) {
			const CCVector3* pt = cloud->getPoint(i);
			double x = pt->x;
			double y = pt->y;

			// AABB快速排除
			if (x < minX || x > maxX || y < minY || y > maxY)
				continue;

			// 使用上面定义的pointInPolygon接口进行精确判断
			if (pointInPolygon(polygon, static_cast<float>(x), static_cast<float>(y))) {
				local_indices.push_back(i);
			}
		}

		// 合并线程结果
#pragma omp critical
		{
			result_indices.insert(result_indices.end(), local_indices.begin(), local_indices.end());
		}
	}
}


void PointCloudDivider::octreeBasedPolygonCrop(
	ccPointCloud* cloud,
	ccOctree::Shared octree,
	const std::vector<CCVector3>& polygon,
	std::vector<unsigned>& result_indices)
{
	// 获取八叉树的根节点信息
	CCCoreLib::DgmOctree* dgmOctree = octree.get();
	if (!dgmOctree) return;

	// 从根节点开始递归处理
	CCCoreLib::DgmOctree::CellCode rootCode = 0;
	unsigned char rootLevel = 0;
	
	// 递归处理八叉树节点
	processOctreeCellByCode(cloud, dgmOctree, rootCode, rootLevel, polygon, result_indices);
}


/// <summary>
/// 使用莫顿编码递归处理八叉树节点
/// </summary>
void PointCloudDivider::processOctreeCellByCode(
	ccPointCloud* cloud,
	CCCoreLib::DgmOctree* dgmOctree,
	CCCoreLib::DgmOctree::CellCode code,
	unsigned char level,
	const std::vector<CCVector3>& polygon,
	std::vector<unsigned>& result_indices)
{
	// 检查cell是否为空
	if (isCellEmpty(dgmOctree, code, level)) {
		return;
	}

	// 使用上面定义的接口判断cell与多边形的关系
	CellRelation relation = classifyCellByCodeLevel(dgmOctree, code, level, polygon);

	switch (relation) {
	case CellRelation::INSIDE:
		// Cell完全在多边形内，添加所有点
		addCellPointsByCode(cloud, dgmOctree, code, level, result_indices);
		break;
		
	case CellRelation::OUTSIDE:
		// Cell完全在多边形外，排除所有点
		return;
		
	case CellRelation::INTERSECT:
		// Cell与多边形边界相交，需要进一步细分
		if (level < dgmOctree->MAX_OCTREE_LEVEL) {
			// 获取子cell的莫顿编码
			std::vector<CCCoreLib::DgmOctree::CellCode> childCodes = getChildCellCodes(dgmOctree, code, level);
			
			// 递归处理每个子cell
			for (const auto& childCode : childCodes) {
				processOctreeCellByCode(cloud, dgmOctree, childCode, level + 1, polygon, result_indices);
			}
		} else {
			// 达到最大层级，逐点检查
			processCellPointsByCode(cloud, dgmOctree, code, level, polygon, result_indices);
		}
		break;
	}
}

/// <summary>
/// 使用莫顿编码添加cell内的所有点
/// </summary>
void PointCloudDivider::addCellPointsByCode(
	ccPointCloud* cloud,
	CCCoreLib::DgmOctree* dgmOctree,
	CCCoreLib::DgmOctree::CellCode code,
	unsigned char level,
	std::vector<unsigned>& result_indices)
{
	// 获取cell的边界信息
	float xmin, ymin, cellSize;
	getCellXYBounds(dgmOctree, code, level, xmin, ymin, cellSize);
	
	getPointIndicesInCell(dgmOctree, code, level, result_indices);
}

/// <summary>
/// 使用莫顿编码逐点处理cell内的点
/// </summary>
void PointCloudDivider::processCellPointsByCode(
	ccPointCloud* cloud,
	CCCoreLib::DgmOctree* dgmOctree,
	CCCoreLib::DgmOctree::CellCode code,
	unsigned char level,
	const std::vector<CCVector3>& polygon,
	std::vector<unsigned>& result_indices)
{

	std::vector<unsigned> filteredIndices;
	getPointIndicesInCell(dgmOctree, code, level, filteredIndices);
	
	for (unsigned idx : result_indices) {
		const CCVector3* pt = cloud->getPoint(idx);
		if (pointInPolygon(polygon, pt->x, pt->y)) {
			result_indices.push_back(idx);
		}
	}
}
