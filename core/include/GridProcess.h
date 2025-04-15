#pragma once
#pragma execution_character_set("utf-8")

#include <vector>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <PointCloudIO.h>

namespace roadmarking
{
	struct Point
	{
		int x, y;
		Point(int x, int y) : x(x), y(y) {}
	};

	using Line = std::vector<Point>;
	using Lines = std::vector<Line>;
	using Grid = std::vector<std::vector<float>>;

	class GridProcess
	{
	public:
		GridProcess(float resolution = 0.03);

		void performOrthogonalGridMapping(ccCloudPtr roadCloud);
		void performOrthogonalGridMapping(PCLCloudXYZIPtr roadCloud);
		void restoreFromGridToPointCloud(PCLCloudPtr markingCloud);
		void restoreFromGridToPointCloud(ccCloudPtr markingCloud);
		void restoreFromGridToPointCloud(PCLCloudXYZIPtr markingCloud);
		void divByDoubleAdaptiveIntensityThreshold();

		void calculateMean(const std::vector<float>& grid, float& mean);

		void processGridToPolylineCloud(ccHObject* polylineContainer);

		void export2DImage(const Grid& intensityGrid, const QString& filename);

	private:
		int otsuThreshold(const Grid& gridI);
		int otsuThreshold(const std::vector<float>& gridI);

		void calculateMean(const Grid& grid, float& mean);
		void normalize(Grid& grid);
		void normalize(std::vector<float>& line);
		void findBoundary(Lines& boundaries);
		void restoreToPolyCloud(const Lines& boundaries, const Grid& ZGrid, PCLCloudPtr markingCloud);
		void fitLines(Line& points);
		void douglasPeucker(Line& points, double epsilon);

		double pointToSegmentDistance(const Point& p, const Point& p1, const Point& p2);

		ccPolyline* createPolylineFromLine(Line line);

		float gridSize = 0.03;
		CCVector3 minCorner, maxCorner;
	public:
		Grid IGrid;
		Grid ZGrid;
	};
};
