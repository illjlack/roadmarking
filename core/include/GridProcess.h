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

		void perform_orthogonal_grid_mapping(ccCloudPtr roadCloud);
		void perform_orthogonal_grid_mapping(PCLCloudXYZIPtr roadCloud);
		void restore_from_grid_to_cloud(PCLCloudPtr markingCloud);
		void restore_from_grid_to_cloud(ccCloudPtr markingCloud);
		void restore_from_grid_to_cloud(PCLCloudXYZIPtr markingCloud);
		void div_by_adaptive_intensity_threshold();


		void process_grid_to_polylines(ccHObject* polylineContainer);

		void export_2D_image(const Grid& intensityGrid, const QString& filename);

	private:
		int get_otsu_threshold(const Grid& gridI);
		int get_otsu_threshold(const std::vector<float>& gridI);
		void calculate_mean(const std::vector<float>& grid, float& mean);
		void calculate_mean(const Grid& grid, float& mean);
		void normalize(Grid& grid);
		void normalize(std::vector<float>& line);
		void find_boundary(Lines& boundaries);
		void restore_2D_boundary_to_3D_cloud(const Lines& boundaries, const Grid& ZGrid, PCLCloudPtr markingCloud);
		void fit_lines(Line& points);
		void Douglas_Peucker(Line& points, double epsilon);

		double get_distance_from_point_to_segment(const Point& p, const Point& p1, const Point& p2);

		ccPolyline* create_polyline_from_line(Line line);

		float gridSize = 0.03;
		CCVector3 minCorner, maxCorner;
	public:
		Grid IGrid;
		Grid ZGrid;
	};
};
