#include "CloudFilterDlg.h"

#include <QPainter>
#include <QMouseEvent>
#include <algorithm>

#include "CloudProcess.h"
#include <QApplication>
using namespace roadmarking;

ThresholdHistogramWidget::ThresholdHistogramWidget(QWidget* parent)
	: QWidget(parent), lowerThreshold(50), upperThreshold(200) {}

ThresholdHistogramWidget::~ThresholdHistogramWidget()
{
	if (pointCloud)pointCloud->setUpperAndLowerThreshold(false);
}

void ThresholdHistogramWidget::setPointCloud(ccPointCloud* pointCloud) {
	this->pointCloud = pointCloud;
	computeIntensityRange(); // 计算强度的最小值和最大值
	updateHistogramData();   // 生成直方图数据
	calculateSum();          // 计算阈值下方和上方的强度和
	update();                // 更新显示
}

void ThresholdHistogramWidget::setUpperAndLowerThreshold(bool is_has_threshold, float lowerThreshold, float upperThreshold)
{
	if (is_has_threshold) {
		// 如果传入了阈值，使用传入的值
		this->lowerThreshold = lowerThreshold;
		this->upperThreshold = upperThreshold;
	}
	else {
		// 如果没有传入阈值，使用点云数据的最小和最大强度值作为阈值
		this->lowerThreshold = minIntensity;
		this->upperThreshold = maxIntensity;
	}
	if (pointCloud)
	{
		pointCloud->setUpperAndLowerThreshold(is_has_threshold,
			lowerThreshold / 256 * (maxIntensity - minIntensity) + minIntensity,
			upperThreshold / 256 * (maxIntensity - minIntensity) + minIntensity);
		pointCloud->getDisplay()->redraw(false, true);
		QApplication::processEvents();
	}
	update(); // 更新显示

}

void ThresholdHistogramWidget::paintEvent(QPaintEvent* event)
{
	if (!pointCloud)return;
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	drawHistogram(painter); // 绘制直方图
}

void ThresholdHistogramWidget::mousePressEvent(QMouseEvent* event) {
	int debug_x = event->x();

	if (event->x() >= lowerThresholdX && event->x() <= lowerThresholdX + 10) {
		draggingLower = true; // 如果点击在下阈值附近，开始拖动下阈值
	}
	else if (event->x() >= upperThresholdX && event->x() <= upperThresholdX + 10) {
		draggingUpper = true; // 如果点击在上阈值附近，开始拖动上阈值
	}
}

void ThresholdHistogramWidget::mouseMoveEvent(QMouseEvent* event) {
	float histWidth = width() - 40;
	float newThreshold = (event->x() - 20) / (histWidth / 256);
	if (draggingLower)
	{
		if (newThreshold >= 0 && newThreshold <= 255 && newThreshold < upperThreshold - 5)
		{
			setUpperAndLowerThreshold(true, newThreshold, upperThreshold);
			update(); // 更新显示
		}
	}
	else if (draggingUpper)
	{
		if (newThreshold >= 0 && newThreshold <= 255 && newThreshold > lowerThreshold + 5)
		{
			setUpperAndLowerThreshold(true, lowerThreshold, newThreshold);
			update(); // 更新显示
		}
	}
}

void ThresholdHistogramWidget::mouseReleaseEvent(QMouseEvent* event) {
	draggingLower = draggingUpper = false; // 释放拖动标志
}

void ThresholdHistogramWidget::calculateSum() {
	belowThresholdSum = 0;
	aboveThresholdSum = 0;
	// 计算阈值下方和上方的强度和
	for (int i = 0; i < histogram.size(); ++i) {
		if (i <= lowerThreshold) {
			belowThresholdSum += histogram[i];
		}
		else if (i >= upperThreshold) {
			aboveThresholdSum += histogram[i];
		}
	}
}

void ThresholdHistogramWidget::drawHistogram(QPainter& painter) {
	float histWidth = width() - 40;
	float histHeight = height() - 40;
	float binWidth = histWidth / 256;

	painter.fillRect(20, 20, histWidth, histHeight, QBrush(Qt::white));

	int maxBinHeight = *std::max_element(histogram.begin(), histogram.end());
	for (int i = 0; i < 256; i++) {
		int binHeight = histogram[i];
		int normalizedHeight = binHeight * histHeight / maxBinHeight;

		QColor color;
		if (i <= lowerThreshold) {
			color = QColor(0, 255, 0);
		}
		else if (i >= upperThreshold) {
			color = QColor(0, 0, 255);
		}
		else {
			color = QColor(255, 255, 0);
		}

		QRect rect(20 + i * binWidth, histHeight - normalizedHeight + 20, binWidth, normalizedHeight);
		painter.fillRect(rect, QBrush(color));
	}

	QFont font = painter.font();
	font.setPointSize(6);
	painter.setFont(font);
	for (int i = 0; i <= 255; i += 32) {
		painter.drawText(20 + i * binWidth, height() - 5, QString::number(i));
	}

	painter.drawText(20, 15, "Normalized Histogram");

	lowerThresholdX = 20 + lowerThreshold * binWidth;
	upperThresholdX = 20 + upperThreshold * binWidth;

	painter.setPen(QPen(Qt::red, 3));
	painter.drawLine(lowerThresholdX, 20, lowerThresholdX, histHeight + 20);
	painter.drawLine(upperThresholdX, 20, upperThresholdX, histHeight + 20);

	painter.setPen(QPen(Qt::black));
	font.setPointSize(10);
	painter.drawText(lowerThresholdX + 10, histHeight + 15, QString("Lower: %1").arg(lowerThreshold));
	painter.drawText(upperThresholdX + 10, histHeight + 15, QString("Upper: %1").arg(upperThreshold));

	painter.drawText(40, histHeight + 60, QString("Below Threshold Sum: %1").arg(belowThresholdSum));
	painter.drawText(40, histHeight + 90, QString("Above Threshold Sum: %1").arg(aboveThresholdSum));
}

void ThresholdHistogramWidget::computeIntensityRange() {
	if (pointCloud)
	{
		minIntensity = std::numeric_limits<float>::max();
		maxIntensity = std::numeric_limits<float>::lowest();

		int sfIdx = PointCloudIO::getIntensityIdx(pointCloud);

		auto sf = pointCloud->getScalarField(sfIdx);

		if (!sf)return;

		// 遍历点云数据，计算强度的最小值和最大值
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float intensity = sf->getValue(i);
			minIntensity = std::min(minIntensity, intensity);
			maxIntensity = std::max(maxIntensity, intensity);
		}
	}
}

void ThresholdHistogramWidget::updateHistogramData()
{
	histogram.resize(256, 0); // 初始化直方图数据
	int sfIdx = PointCloudIO::getIntensityIdx(pointCloud);
	auto sf = pointCloud->getScalarField(sfIdx);
	if (!sf)return;
	if (pointCloud)
	{
		// 遍历点云数据，根据强度值更新直方图
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float intensity = sf->getValue(i);
			int bin = (int)(255 * (intensity - minIntensity) / (maxIntensity - minIntensity)); // 映射到0-255区间
			bin = std::min(std::max(bin, 0), 255); // 确保bin在有效范围内
			++histogram[bin]; // 增加相应bin的计数
		}
	}
}
