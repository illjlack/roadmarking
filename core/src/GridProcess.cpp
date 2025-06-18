#include "GridProcess.h"
#include <cmath>
#include <algorithm>
#include <PointCloudIO.h>
#include <KdTree.h>
#include <vector>
#include <queue>
#include <functional>
// #include <opencv2/opencv.hpp>
#include <QDir>
#include <ccPolyline.h>
#include <ccPointCloud.h>
#include <ccHObject.h>
#include <stack>
#include <pcl/common/pca.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <QDialog>
#include <QVBoxLayout>


#include <QDialog>
#include <QVBoxLayout>
#include <QPainter>
#include <QMouseEvent>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <vector>

using namespace roadmarking;

GridProcess::GridProcess(float resolution) :
	gridSize(resolution) {}

void GridProcess::perform_orthogonal_grid_mapping(ccCloudPtr roadCloud)
{
	roadCloud->getBoundingBox(minCorner, maxCorner);
	// 拓展边界,5个格子（避免搜索点云边界时误判）
	const float expansion = 5 * gridSize;
	minCorner.x -= expansion;
	minCorner.y -= expansion;
	minCorner.z -= expansion;
	maxCorner.x += expansion;
	maxCorner.y += expansion;
	maxCorner.z += expansion;

	const float Xmin = minCorner.x;
	const float Ymin = minCorner.y;
	const float Xmax = maxCorner.x;
	const float Ymax = maxCorner.y;

	const int numRows = static_cast<int>((Xmax - Xmin) / gridSize) + 1;
	const int numCols = static_cast<int>((Ymax - Ymin) / gridSize) + 1;

	// 初始化网格，NaN表示空
	IGrid.resize(numRows, std::vector<float>(numCols, NAN));
	ZGrid.resize(numRows, std::vector<float>(numCols, NAN));
	Grid totalWeight(numRows, std::vector<float>(numCols, 0.0f));  // 权重累加器

	// 获取强度字段
	const int intensitySFIndex = PointCloudIO::get_intensity_idx(roadCloud);;
	ccScalarField* intensitySF = intensitySFIndex >= 0 ?
		dynamic_cast<ccScalarField*>(roadCloud->getScalarField(intensitySFIndex)) : nullptr;

	// 预处理强度值（IDW需要）
	float intensityOffset = 0.0f;
	if (intensitySF)
	{
		const float minIntensity = intensitySF->getMin();
		intensityOffset = (minIntensity < 0) ? (-minIntensity + 1.0f) : 0.0f;
	}

	// IDW插值处理
	for (size_t i = 0; i < roadCloud->size(); ++i)
	{
		CCVector3 point = *roadCloud->getPoint(i);

		const int N = static_cast<int>((point.x - Xmin) / gridSize);
		const int M = static_cast<int>((point.y - Ymin) / gridSize);

		if (N >= 0 && N < numRows && M >= 0 && M < numCols)
		{
			// 获取当前点强度值
			const float intensityVal = intensitySF ?
				(intensitySF->getValue(i) + intensityOffset) : 1.0f;

			// 计算对邻近3x3网格的影响
			for (int iOffset = -1; iOffset <= 1; ++iOffset)
			{
				for (int jOffset = -1; jOffset <= 1; ++jOffset)
				{
					const int neighborN = N + iOffset;
					const int neighborM = M + jOffset;

					if (neighborN >= 0 && neighborN < numRows &&
						neighborM >= 0 && neighborM < numCols)
					{
						// 计算网格中心坐标
						const float centerX = Xmin + (N + 0.5f) * gridSize;
						const float centerY = Ymin + (M + 0.5f) * gridSize;
						const float neighborCenterX = Xmin + (neighborN + 0.5f) * gridSize;
						const float neighborCenterY = Ymin + (neighborM + 0.5f) * gridSize;

						// 计算距离权重（IDW）
						const float dx = centerX - neighborCenterX;
						const float dy = centerY - neighborCenterY;
						const float dist = sqrt(dx * dx + dy * dy) + 0.001f; // 避免除零
						const float weight = 1.0f / (dist * dist);        // 平方反比权重

						// 初始化或累加强度值
						if (std::isnan(IGrid[neighborN][neighborM]))
						{
							IGrid[neighborN][neighborM] = intensityVal * weight;
						}
						else
						{
							IGrid[neighborN][neighborM] += intensityVal * weight;
						}

						// 累加总权重
						totalWeight[neighborN][neighborM] += weight;

						// 更新高程值（直接取最大值）
						if (point.z > ZGrid[neighborN][neighborM] ||
							std::isnan(ZGrid[neighborN][neighborM]))
						{
							ZGrid[neighborN][neighborM] = point.z;
						}
					}
				}
			}
		}
	}

	// 应用IDW计算最终强度值
	float Imin = FLT_MAX, Imax = -FLT_MAX;
	for (int row = 0; row < numRows; ++row)
	{
		for (int col = 0; col < numCols; ++col)
		{
			if (totalWeight[row][col] > 0.0f)
			{
				IGrid[row][col] /= totalWeight[row][col];
			}
			else
			{
				IGrid[row][col] = NAN; // 无数据区域保持NaN
			}
		}
	}
}

void GridProcess::perform_orthogonal_grid_mapping(PCLCloudXYZIPtr roadCloud)
{
	// 获取点云的包围盒
	pcl::PointXYZI minPt, maxPt;
	pcl::getMinMax3D(*roadCloud, minPt, maxPt);

	minCorner = { minPt.x, minPt.y, minPt.z };
	maxCorner = { maxPt.x, maxPt.y, maxPt.z };

	// 扩展边界，避免搜索点云边界时误判
	const float expansion = 5 * gridSize;
	minCorner.x -= expansion;
	minCorner.y -= expansion;
	minCorner.z -= expansion;
	maxCorner.x += expansion;
	maxCorner.y += expansion;
	maxCorner.z += expansion;

	// 计算网格的行数和列数
	const float Xmin = minCorner.x;
	const float Ymin = minCorner.y;
	const float Xmax = maxCorner.x;
	const float Ymax = maxCorner.y;

	const int numRows = static_cast<int>((Xmax - Xmin) / gridSize) + 1;
	const int numCols = static_cast<int>((Ymax - Ymin) / gridSize) + 1;

	// 初始化网格，NaN表示空
	IGrid.resize(numRows, std::vector<float>(numCols, NAN));
	ZGrid.resize(numRows, std::vector<float>(numCols, NAN));
	Grid totalWeight(numRows, std::vector<float>(numCols, 0.0f));  // 权重累加器

	// 遍历点云进行插值处理
	for (const auto& point : roadCloud->points)
	{
		const int N = static_cast<int>((point.x - Xmin) / gridSize);
		const int M = static_cast<int>((point.y - Ymin) / gridSize);

		if (N >= 0 && N < numRows && M >= 0 && M < numCols)
		{
			// 计算当前点的强度值（此处假设强度值存储在点的 'intensity' 字段中）
			const float intensityVal = point.intensity;

			// 计算对邻近3x3网格的影响
			for (int iOffset = -1; iOffset <= 1; ++iOffset)
			{
				for (int jOffset = -1; jOffset <= 1; ++jOffset)
				{
					const int neighborN = N + iOffset;
					const int neighborM = M + jOffset;

					if (neighborN >= 0 && neighborN < numRows &&
						neighborM >= 0 && neighborM < numCols)
					{
						// 计算网格中心坐标
						const float centerX = Xmin + (N + 0.5f) * gridSize;
						const float centerY = Ymin + (M + 0.5f) * gridSize;
						const float neighborCenterX = Xmin + (neighborN + 0.5f) * gridSize;
						const float neighborCenterY = Ymin + (neighborM + 0.5f) * gridSize;

						// 计算距离权重（IDW）
						const float dx = centerX - neighborCenterX;
						const float dy = centerY - neighborCenterY;
						const float dist = sqrt(dx * dx + dy * dy) + 0.001f; // 避免除零
						const float weight = 1.0f / (dist * dist);        // 平方反比权重

						// 初始化或累加强度值
						if (std::isnan(IGrid[neighborN][neighborM]))
						{
							IGrid[neighborN][neighborM] = intensityVal * weight;
						}
						else
						{
							IGrid[neighborN][neighborM] += intensityVal * weight;
						}

						// 累加总权重
						totalWeight[neighborN][neighborM] += weight;

						// 更新高程值（直接取最大值）
						if (point.z > ZGrid[neighborN][neighborM] ||
							std::isnan(ZGrid[neighborN][neighborM]))
						{
							ZGrid[neighborN][neighborM] = point.z;
						}
					}
				}
			}
		}
	}

	// 应用IDW计算最终强度值
	for (int row = 0; row < numRows; ++row)
	{
		for (int col = 0; col < numCols; ++col)
		{
			if (totalWeight[row][col] > 0.0f)
			{
				IGrid[row][col] /= totalWeight[row][col];
			}
			else
			{
				IGrid[row][col] = NAN; // 无数据区域保持NaN
			}
		}
	}
}

void GridProcess::restore_from_grid_to_cloud(PCLCloudPtr markingCloud)
{
	int numRows = IGrid.size();
	int numCols = IGrid.front().size();

	// 恢复每个网格点到原始点云
	for (int N = 0; N < numRows; ++N)
	{
		for (int M = 0; M < numCols; ++M)
		{
			// 如果是路标
			if (!std::isnan(IGrid[N][M]))
			{
				// 计算点在原始点云坐标系中的位置
				float x = minCorner.x + N * gridSize;
				float y = minCorner.y + M * gridSize;
				float z = ZGrid[N][M];

				// 创建新的点云点
				PCLPoint restoredPoint(x, y, z);
				markingCloud->push_back(restoredPoint);
			}
		}
	}
}

void GridProcess::restore_from_grid_to_cloud(ccCloudPtr markingCloud)
{
	int numRows = IGrid.size();
	int numCols = IGrid.front().size();


	// 遍历网格，恢复点云数据
	for (int N = 0; N < numRows; ++N)
	{
		for (int M = 0; M < numCols; ++M)
		{
			// 如果是路标
			if (!std::isnan(IGrid[N][M]))
			{
				// 计算点在原始点云坐标系中的位置
				float x = minCorner.x + N * gridSize;
				float y = minCorner.y + M * gridSize;
				float z = ZGrid[N][M];
				markingCloud->addPoint({ x,y,z });
			}
		}
	}
}

void GridProcess::restore_from_grid_to_cloud(PCLCloudXYZIPtr markingCloud)
{
	int numRows = IGrid.size();
	int numCols = IGrid.front().size();

	// 恢复每个网格点到原始点云
	for (int N = 0; N < numRows; ++N)
	{
		for (int M = 0; M < numCols; ++M)
		{
			// 如果是路标
			if (!std::isnan(IGrid[N][M]))
			{
				// 计算点在原始点云坐标系中的位置
				float x = minCorner.x + N * gridSize;
				float y = minCorner.y + M * gridSize;
				float z = ZGrid[N][M];
				float i = IGrid[N][M];

				// 创建新的点云点
				PCLPointXYZI restoredPoint(x, y, z, i);
				markingCloud->push_back(restoredPoint);
			}
		}
	}
}

void GridProcess::div_by_adaptive_intensity_threshold()
{
	if (!IGrid.size() || !IGrid[0].size())return;

	int n = IGrid.size();
	int m = IGrid[0].size();

	Grid& grid = IGrid;
	// 1. 归一化到0~255范围
	normalize(grid);

	// 2. 计算Otsu阈值
	const int otsuThreshold = GridProcess::get_otsu_threshold(grid);
	if (otsuThreshold < 0) return; // 无效阈值处理

	const float globalThreshold = otsuThreshold;

	for (auto& row : grid) {
		for (float& val : row) {
			if (!std::isnan(val) && val < globalThreshold)
			{
				val = NAN; // 低于阈值设为空
			}
		}
	}

	/// <summary>
	/// 搜索
	/// </summary>
	const int size_threshold = 10000;
	std::vector<std::vector<int>>is_visit(n, std::vector<int>(m));
	const int dx[8] = { 1, 1, 1, 0, 0, -1, -1, -1 };
	const int dy[8] = { -1, 0, 1, 1, -1, -1, 0, 1 };

	int current_iterations = 0; // 当前迭代次数
	bool is_continue = true; // 是否继续迭代
	auto bfs_div_by_threshold = [&](Point pos)
	{
		std::vector<float>vis_intensity;
		std::vector<Point>vis_pos;

		std::queue<Point>q;
		q.push(pos);

		vis_intensity.push_back(grid[pos.x][pos.y]);
		vis_pos.push_back(pos);
		is_visit[pos.x][pos.y] = true;

		while (!q.empty())
		{
			auto& pos = q.front();
			q.pop();

			for (int i = 0; i < 8; i++)
			{
				int xx = pos.x + dx[i], yy = pos.y + dy[i];

				if (xx < 0 || xx >= n || yy < 0 || yy >= m)continue;

				if (is_visit[xx][yy] == current_iterations && !std::isnan(grid[xx][yy]))
				{
					is_visit[xx][yy]++;
					vis_intensity.push_back(grid[xx][yy]);
					vis_pos.push_back({ xx , yy });
					q.push({ xx, yy });
				}
			}
		}

		if (vis_pos.size() < 10)//零散点
		{
			for (int i = 0; i < vis_pos.size(); i++)
			{
				grid[vis_pos[i].x][vis_pos[i].y] = NAN;
			}
			return;
		}

		if (vis_pos.size() < size_threshold)return;

		is_continue = true;
		normalize(vis_intensity);
		const int otsu_threshold = GridProcess::get_otsu_threshold(vis_intensity);

		for (int i = 0; i < vis_pos.size(); i++)
		{
			if (vis_intensity[i] < otsu_threshold)
			{
				grid[vis_pos[i].x][vis_pos[i].y] = NAN;
			}
		}

		/*for (auto& pos : vis_pos)
		{
			if (grid[pos.x][pos.y] < otsu_threshold)
			{
				grid[pos.x][pos.y] = NAN;
			}
		}*/
	};

	for (; is_continue; current_iterations++)
	{
		is_continue = false;
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < m; j++)
			{
				if (is_visit[i][j] == current_iterations && !std::isnan(grid[i][j]))
				{
					bfs_div_by_threshold({ i, j });
				}
			}
		}
	}
}

void GridProcess::calculate_mean(const std::vector<float>& grid, float& mean)
{
	float sum = 0.0f;
	int count = 0;
	for (float value : grid)
	{
		if (std::isnan(value)) continue; // 跳过NaN
		sum += value;
		count++;
	}
	mean = (count > 0) ? sum / count : NAN; // 无有效数据时返回NaN
}

void GridProcess::calculate_mean(const Grid& grid, float& mean)
{
	std::vector<float> line;
	for (const auto& row : grid)
	{
		for (float value : row)
		{
			line.push_back(value);
		}
	}
	calculate_mean(line, mean);
}

int GridProcess::get_otsu_threshold(const Grid& gridI)
{
	std::vector<float> line;
	for (const auto& row : gridI)
	{
		for (float value : row)
		{
			if (std::isnan(value)) continue; // 跳过NaN
			line.push_back(value);
		}
	}
	return get_otsu_threshold(line);
}

int GridProcess::get_otsu_threshold(const std::vector<float>& gridI)
{
	const int L = 256;  // 灰度级数
	std::vector<int> histogram(L, 0);
	int validPixels = 0;

	// 计算有效像素的直方图
	for (float value : gridI)
	{
		if (std::isnan(value)) continue; // 跳过NaN

		// 将浮点强度值映射到0-255整数
		int gray = static_cast<int>(std::clamp(value, 0.0f, 255.0f));
		histogram[gray]++;
		validPixels++;
	}


	// 无有效数据时返回-1
	if (validPixels == 0) return -1;

	// 计算全局均值
	float sumTotal = 0.0f;
	for (int i = 0; i < L; ++i)
	{
		sumTotal += i * histogram[i];
	}
	const float mG = sumTotal / validPixels;

	// Otsu算法核心
	float maxVariance = 0.0f;
	int optimalThreshold = 0;
	float sumBackground = 0.0f;
	int countBackground = 0;

	for (int k = 0; k < L; ++k)
	{
		countBackground += histogram[k];
		sumBackground += k * histogram[k];

		if (countBackground == 0) continue;

		const int countForeground = validPixels - countBackground;
		if (countForeground == 0) break;

		const float m1 = sumBackground / countBackground;
		const float m2 = (sumTotal - sumBackground) / countForeground;

		// 根据背景和前景的样本数量以及先验权重计算类间方差
		const float w1 = countBackground;
		const float w2 = countForeground; // 前景加权

		// 计算加权的类间方差
		const float variance = w1 * (m1 - mG) * (m1 - mG) + w2 * (m2 - mG) * (m2 - mG);

		if (variance > maxVariance)
		{
			maxVariance = variance;
			optimalThreshold = k;
		}
	}
	return optimalThreshold;
}

void GridProcess::normalize(Grid& grid)
{
	std::vector<float> line;
	for (const auto& row : grid)
	{
		for (float value : row)
		{
			line.push_back(value);
		}
	}
	normalize(line);
	size_t idx = 0;
	for (auto& row : grid)
	{
		for (auto& value : row)
		{
			value = line[idx++];  // 把归一化后的值回写到Grid中
		}
	}
}

void GridProcess::normalize(std::vector<float>& line)
{
	float minVal = FLT_MAX;
	float maxVal = FLT_MIN;

	// 计算最大强度阈值。超过是异常，出现概率1/100000
	float maxIntensityThreshold = std::numeric_limits<float>::infinity();
	if (line.size() > 100000) {
		std::vector<float> validData;
		for (float pixel : line) {
			if (!std::isnan(pixel)) {
				validData.push_back(pixel);
			}
		}
		std::sort(validData.begin(), validData.end());

		// 计算Q1和Q3
		int N = validData.size() - 1;
		int Index = N - N / 30000;
		float Q1 = validData[Index];

		maxIntensityThreshold = Q1;
	}

	// 去除超高异常强度点
	for (float& pixel : line)
	{
		if (!std::isnan(pixel))
		{
			if (pixel > maxIntensityThreshold)
			{
				pixel = NAN; // 超过阈值的点标记为NaN
			}
			else
			{
				minVal = std::min(minVal, pixel);
				maxVal = std::max(maxVal, pixel);
			}
		}
	}

	for (auto& pixel : line)
	{
		if (!std::isnan(pixel))
		{
			pixel = (pixel - minVal) / (maxVal - minVal) * 255;
		}
	}
}

void GridProcess::process_grid_to_polylines(ccHObject* polylineContainer)
{
	div_by_adaptive_intensity_threshold();
	Lines extractedLines;
	find_boundary(extractedLines);

	for (auto& line : extractedLines)
	{
		fit_lines(line);
		// 存入 CloudCompare 折线
		ccPolyline* polyline = create_polyline_from_line(line);
		if (polyline)
		{
			polylineContainer->addChild(polyline);
		}
	}
}

ccPolyline* GridProcess::create_polyline_from_line(Line line)
{
	ccPointCloud* cloud = new ccPointCloud("Polyline Points");
	if (!cloud || line.empty()) return nullptr;

	for (const auto& p : line)
	{
		float x = minCorner.x + p.x * gridSize;
		float y = minCorner.y + p.y * gridSize;
		float z = ZGrid[p.x][p.y];
		cloud->addPoint(CCVector3(x, y, z));
	}

	ccPolyline* polyline = new ccPolyline(cloud);
	if (!polyline) {
		delete cloud;
		return nullptr;
	}

	polyline->addChild(cloud);
	polyline->reserve(static_cast<unsigned>(line.size()));

	for (size_t i = 0; i < line.size(); ++i)
	{
		polyline->addPointIndex(static_cast<unsigned>(i));
	}

	if (line.front().x == line.back().x && line.front().y == line.back().y)
	{
		polyline->setClosed(true);
	}
	else {
		polyline->setClosed(false);
	}

	polyline->setName("Extracted Boundary Polyline");
	return polyline;
}

void GridProcess::find_boundary(Lines& boundaries)
{
	if (IGrid.empty() || IGrid[0].empty()) return;

	const int dx[8] = { 1, 1, 1, 0, 0, -1, -1, -1 };
	const int dy[8] = { 1, -1, 0, 1, -1, 1, 0, -1 };

	const int rows = IGrid.size();
	const int cols = IGrid[0].size();

	std::vector<std::vector<int>> mark(rows, std::vector<int>(cols, 0));
	std::set<std::pair<int, int>> unorderedBoundary;
	std::vector<Point> boundaryPoints;

	// 标记边界点
	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			if (fabs(IGrid[i][j]) < std::numeric_limits<float>::epsilon()) continue;

			for (int z = 0; z < 8; z++)
			{
				int xx = i + dx[z];
				int yy = j + dy[z];
				if (xx < 0 || xx >= rows || yy < 0 || yy >= cols || fabs(IGrid[xx][yy]) < std::numeric_limits<float>::epsilon())
				{
					unorderedBoundary.insert({ i, j });
					mark[i][j] = 1;
					break;
				}
			}
		}
	}

	if (unorderedBoundary.empty()) return;

	std::function<void(int, int, Line&)> dfs =
		[&](int x, int y, Line& boundary)
	{
		std::stack<Point> stack;
		stack.push(Point(x, y));

		mark[x][y] = 0;
		boundary.push_back(Point(x, y));

		while (!stack.empty())
		{
			Point current = stack.top();
			stack.pop();

			int cx = current.x, cy = current.y;
			for (int i = 0; i < 8; i++)
			{
				int nx = cx + dx[i], ny = cy + dy[i];
				if (nx < 0 || nx >= rows || ny < 0 || ny >= cols || !mark[nx][ny]) continue;
				mark[nx][ny] = 0;
				stack.push(Point(nx, ny));
				boundary.push_back(Point(nx, ny));
			}
		}
	};

	boundaries.clear();
	for (auto& p : unorderedBoundary)
	{
		if (mark[p.first][p.second])
		{
			boundaries.push_back({});
			dfs(p.first, p.second, boundaries.back());

			if (!boundaries.back().empty())
			{
				boundaries.back().push_back(boundaries.back().front());
			}
		}
	}
}

void GridProcess::restore_2D_boundary_to_3D_cloud(const Lines& boundaries, const Grid& ZGrid, PCLCloudPtr markingCloud)
{
	markingCloud->clear();
	for (const auto& boundary : boundaries)
	{
		for (const auto& p : boundary)
		{
			float x = minCorner.x + p.x * gridSize;
			float y = minCorner.y + p.y * gridSize;

			float z = ZGrid[p.x][p.y];
			markingCloud->push_back(pcl::PointXYZ(p.x, p.y, z));
		}
	}
}

double GridProcess::get_distance_from_point_to_segment(const Point& p, const Point& p1, const Point& p2)
{
	double dx = p2.x - p1.x, dy = p2.y - p1.y;
	if (dx == 0 && dy == 0) return hypot(p.x - p1.x, p.y - p1.y);

	double t = ((p.x - p1.x) * dx + (p.y - p1.y) * dy) / (dx * dx + dy * dy);
	t = std::max(0.0, std::min(1.0, t));
	double projX = p1.x + t * dx, projY = p1.y + t * dy;
	return hypot(p.x - projX, p.y - projY);
}

void GridProcess::Douglas_Peucker(Line& points, double epsilon) {
	if (points.size() < 3) return;

	// 处理闭合边界
	bool isClosed = (points.front().x == points.back().x && points.front().y == points.back().y);
	if (isClosed) points.pop_back();

	double maxDist = 0.0;
	int index = 0;
	for (size_t i = 1; i < points.size() - 1; ++i) {
		double dist = get_distance_from_point_to_segment(points[i], points.front(), points.back());
		if (dist > maxDist) {
			maxDist = dist;
			index = i;
		}
	}

	if (maxDist > epsilon) {
		Line left(points.begin(), points.begin() + index + 1);
		Line right(points.begin() + index, points.end());
		Douglas_Peucker(left, epsilon);
		Douglas_Peucker(right, epsilon);

		points.clear();
		points.insert(points.end(), left.begin(), left.end());
		points.insert(points.end(), right.begin() + 1, right.end());
	}
	else {
		points = { points.front(), points.back() };
	}

	// 重新闭合边界
	if (isClosed) points.push_back(points.front());
}

void GridProcess::fit_lines(Line& points)
{
	if (points.size() < 2) return;
	int midIndex = points.size() / 2;

	Line firstHalf(points.begin(), points.begin() + midIndex + 1);
	Line secondHalf(points.begin() + midIndex, points.end());

	Douglas_Peucker(firstHalf, 0.2 / gridSize);
	Douglas_Peucker(secondHalf, 0.2 / gridSize);

	points.clear();
	points.insert(points.end(), firstHalf.begin(), firstHalf.end());
	points.insert(points.end(), secondHalf.begin() + 1, secondHalf.end());

	if (points.front().x == points.back().x && points.front().y == points.back().y)
	{
		points.push_back(points.front());
	}
}

void GridProcess::export_2D_image(const Grid& IGrid, const QString& filename)
{
	//// 创建一个 OpenCV 图像对象
	//cv::Mat image(IGrid.size(), IGrid[0].size(), CV_32F);

	//for (size_t i = 0; i < IGrid.size(); ++i)
	//{
	//	for (size_t j = 0; j < IGrid[i].size(); ++j)
	//	{
	//		image.at<float>(i, j) = IGrid[i][j];
	//	}
	//}


	//// 确保文件路径正确
	//QString filePath = filename;
	//filePath = QDir::toNativeSeparators(filePath);  // 转换为本地文件分隔符

	//// 保存图像
	//cv::imwrite(filePath.toStdString(), image);
}
