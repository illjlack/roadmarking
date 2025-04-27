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
	computeIntensityRange(); // ����ǿ�ȵ���Сֵ�����ֵ
	updateHistogramData();   // ����ֱ��ͼ����
	calculateSum();          // ������ֵ�·����Ϸ���ǿ�Ⱥ�
	update();                // ������ʾ
}

void ThresholdHistogramWidget::setUpperAndLowerThreshold(bool is_has_threshold, float lowerThreshold, float upperThreshold)
{
	if (is_has_threshold) {
		// �����������ֵ��ʹ�ô����ֵ
		this->lowerThreshold = lowerThreshold;
		this->upperThreshold = upperThreshold;
	}
	else {
		// ���û�д�����ֵ��ʹ�õ������ݵ���С�����ǿ��ֵ��Ϊ��ֵ
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
	update(); // ������ʾ

}

void ThresholdHistogramWidget::paintEvent(QPaintEvent* event)
{
	if (!pointCloud)return;
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	drawHistogram(painter); // ����ֱ��ͼ
}

void ThresholdHistogramWidget::mousePressEvent(QMouseEvent* event) {
	int debug_x = event->x();

	if (event->x() >= lowerThresholdX && event->x() <= lowerThresholdX + 10) {
		draggingLower = true; // ������������ֵ��������ʼ�϶�����ֵ
	}
	else if (event->x() >= upperThresholdX && event->x() <= upperThresholdX + 10) {
		draggingUpper = true; // ������������ֵ��������ʼ�϶�����ֵ
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
			update(); // ������ʾ
		}
	}
	else if (draggingUpper)
	{
		if (newThreshold >= 0 && newThreshold <= 255 && newThreshold > lowerThreshold + 5)
		{
			setUpperAndLowerThreshold(true, lowerThreshold, newThreshold);
			update(); // ������ʾ
		}
	}
}

void ThresholdHistogramWidget::mouseReleaseEvent(QMouseEvent* event) {
	draggingLower = draggingUpper = false; // �ͷ��϶���־
}

void ThresholdHistogramWidget::calculateSum() {
	belowThresholdSum = 0;
	aboveThresholdSum = 0;
	// ������ֵ�·����Ϸ���ǿ�Ⱥ�
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

		// �����������ݣ�����ǿ�ȵ���Сֵ�����ֵ
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
	histogram.resize(256, 0); // ��ʼ��ֱ��ͼ����
	int sfIdx = PointCloudIO::getIntensityIdx(pointCloud);
	auto sf = pointCloud->getScalarField(sfIdx);
	if (!sf)return;
	if (pointCloud)
	{
		// �����������ݣ�����ǿ��ֵ����ֱ��ͼ
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float intensity = sf->getValue(i);
			int bin = (int)(255 * (intensity - minIntensity) / (maxIntensity - minIntensity)); // ӳ�䵽0-255����
			bin = std::min(std::max(bin, 0), 255); // ȷ��bin����Ч��Χ��
			++histogram[bin]; // ������Ӧbin�ļ���
		}
	}
}
