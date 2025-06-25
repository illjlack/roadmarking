#pragma once
#pragma execution_character_set("utf-8")

#include <vector>
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccScalarField.h>
#include <ccBBox.h>
#include "comm.h"

namespace roadmarking
{
	/// <summary>
	/// 点云划分算法类
	/// 提供各种点云分割和划分功能
	/// </summary>
	class PointCloudDivider
	{
	public:

		static void cropWithPolygon(
			ccPointCloud* cloud,
			const std::vector<CCVector3d>& polygon_points,
			ccPointCloud* cloud_cropped);

		/// <summary>
		/// 使用多边形裁剪点云（支持八叉树优化）
		/// </summary>
		/// <param name="clouds">输入点云列表</param>
		/// <param name="polygon_points">多边形顶点</param>
		/// <param name="cloud_cropped">输出裁剪后的点云</param>
		static void cropWithPolygon(
			const std::vector<ccPointCloud*>& clouds,
			const std::vector<CCVector3d>& polygon_points,
			ccPointCloud* cloud_cropped);

		/// <summary>
		/// 使用八叉树优化的多边形裁剪
		/// 核心算法：基于莫顿编码的八叉树遍历，使用code+level快速判断cell与多边形的关系
		/// 优化策略：
		/// 1. 使用AABB快速排除完全不相交的cell
		/// 2. 通过线段相交检测判断cell是否与多边形边界相交
		/// 3. 对于相交的cell，递归到子层级进行更精确的判断
		/// 4. 对于完全在多边形内/外的cell，批量处理其中的所有点
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="octree">八叉树，用于空间索引和快速查询</param>
		/// <param name="polygon">多边形顶点集合，定义裁剪区域</param>
		/// <param name="result_indices">输出：在多边形内的点索引</param>
		static void octreeBasedPolygonCrop(
			ccPointCloud* cloud,
			ccOctree::Shared octree,
			const std::vector<CCVector3>& polygon,
			std::vector<unsigned>& result_indices);

		/// <summary>
		/// 传统逐点处理（无八叉树优化）
		/// 算法：对每个点使用射线法判断是否在多边形内
		/// 特点：简单可靠，但计算复杂度为O(n*m)，n为点数，m为多边形边数
		/// 适用场景：点云较小或没有八叉树索引时使用
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="polygon">多边形顶点集合</param>
		/// <param name="result_indices">输出点索引</param>
		static void pointBasedPolygonCrop(
			ccPointCloud* cloud,
			const std::vector<CCVector3>& polygon,
			std::vector<unsigned>& result_indices);

	private:
		/// <summary>
		/// 检查体素是否完全在多边形内
		/// 判断方法：检查体素的四个角点是否都在多边形内
		/// 如果四个角点都在多边形内，则整个体素都在多边形内
		/// 这是八叉树优化的关键：可以批量处理体素内的所有点
		/// 优化：使用角点判断比线段相交检测更简单高效
		/// </summary>
		/// <param name="minX,maxX,minY,maxY">体素的AABB边界</param>
		/// <param name="polygon">多边形顶点集合</param>
		/// <returns>true表示体素完全在多边形内</returns>
		static bool isCellFullyInsidePolygon(double minX, double maxX, double minY, double maxY, const std::vector<CCVector3>& polygon);

		/// <summary>
		/// 检查体素是否完全在多边形外
		/// 判断方法：检查体素的四个角点是否都在多边形外
		/// 如果四个角点都在多边形外，则整个体素都在多边形外
		/// 这是八叉树优化的关键：可以完全排除该体素内的所有点
		/// 优化：使用角点判断比线段相交检测更简单高效
		/// </summary>
		/// <param name="minX,maxX,minY,maxY">体素的AABB边界</param>
		/// <param name="polygon">多边形顶点集合</param>
		/// <returns>true表示体素完全在多边形外</returns>
		static bool isCellFullyOutsidePolygon(double minX, double maxX, double minY, double maxY, const std::vector<CCVector3>& polygon);

		/// <summary>
		/// 添加体素内的所有点到结果中
		/// 当确定体素完全在多边形内时，可以批量添加该体素内的所有点
		/// 注意：当前实现为简化版本，实际应该只添加体素内的点
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="cell">八叉树节点</param>
		/// <param name="result_indices">输出：点索引</param>
		static void addCellPoints(ccPointCloud* cloud, const CCCoreLib::DgmOctree::octreeCell* cell, std::vector<unsigned>& result_indices);

		/// <summary>
		/// 逐点处理体素内的点
		/// 当体素与多边形边界相交时，需要逐点进行精确判断
		/// 这是八叉树优化的边界情况处理
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="cell">八叉树节点</param>
		/// <param name="polygon">多边形顶点集合</param>
		/// <param name="result_indices">输出：在多边形内的点索引</param>
		static void processCellPointsIndividually(ccPointCloud* cloud, const CCCoreLib::DgmOctree::octreeCell* cell, const std::vector<CCVector3>& polygon, std::vector<unsigned>& result_indices);

		/// <summary>
		/// 使用莫顿编码递归处理八叉树节点
		/// 这是基于上面定义的接口的优化实现
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="dgmOctree">DgmOctree指针</param>
		/// <param name="code">莫顿编码</param>
		/// <param name="level">八叉树层级</param>
		/// <param name="polygon">多边形顶点集合</param>
		/// <param name="result_indices">输出：在多边形内的点索引</param>
		static void processOctreeCellByCode(
			ccPointCloud* cloud,
			CCCoreLib::DgmOctree* dgmOctree,
			CCCoreLib::DgmOctree::CellCode code,
			unsigned char level,
			const std::vector<CCVector3>& polygon,
			std::vector<unsigned>& result_indices);

		/// <summary>
		/// 使用莫顿编码添加cell内的所有点
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="dgmOctree">DgmOctree指针</param>
		/// <param name="code">莫顿编码</param>
		/// <param name="level">八叉树层级</param>
		/// <param name="result_indices">输出：点索引</param>
		static void addCellPointsByCode(
			ccPointCloud* cloud,
			CCCoreLib::DgmOctree* dgmOctree,
			CCCoreLib::DgmOctree::CellCode code,
			unsigned char level,
			std::vector<unsigned>& result_indices);

		/// <summary>
		/// 使用莫顿编码逐点处理cell内的点
		/// </summary>
		/// <param name="cloud">输入点云</param>
		/// <param name="dgmOctree">DgmOctree指针</param>
		/// <param name="code">莫顿编码</param>
		/// <param name="level">八叉树层级</param>
		/// <param name="polygon">多边形顶点集合</param>
		/// <param name="result_indices">输出：在多边形内的点索引</param>
		static void processCellPointsByCode(
			ccPointCloud* cloud,
			CCCoreLib::DgmOctree* dgmOctree,
			CCCoreLib::DgmOctree::CellCode code,
			unsigned char level,
			const std::vector<CCVector3>& polygon,
			std::vector<unsigned>& result_indices);
	};
} 
