#include "CloudFilterDlg.h"

#include <QPainter>
#include <QMouseEvent>
#include <algorithm>

#include "CloudProcess.h"
#include <QApplication>
using namespace roadmarking;

ThresholdHistogramWidget::ThresholdHistogramWidget(QWidget* parent)
	: QWidget(parent), lowerThreshold(50), upperThreshold(200)
{
	this->setGeometry(100, 100, 600, 400);
	// 初始化UI布局和控件
	confirmButton = new QPushButton(tr("确定"), this);
	// 使用垂直布局将按钮放在直方图下方
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->addStretch();  // 上方弹性区域，占据直方图绘制区域
	QHBoxLayout* buttonLayout = new QHBoxLayout();
	buttonLayout->addStretch();             // 左侧弹性区域，使按钮靠右
	buttonLayout->addWidget(confirmButton); // 添加确定按钮
	mainLayout->addLayout(buttonLayout);    // 将水平布局添加到主布局底部

	// 连接按钮点击信号到槽函数
	connect(confirmButton, &QPushButton::clicked, this, &ThresholdHistogramWidget::onConfirmButtonClicked);

}

ThresholdHistogramWidget::~ThresholdHistogramWidget()
{
	if (pointCloud)pointCloud->setUpperAndLowerThreshold(false);
}

void ThresholdHistogramWidget::setPointCloud(ccPointCloud* pointCloud) {
	this->pointCloud = pointCloud;
	computeIntensityRange(); // 计算强度的最小值和最大值
	updateHistogramData();   // 生成直方图数据
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

void ThresholdHistogramWidget::drawHistogram(QPainter& painter) {
	float histWidth = width() - 40;
	float histHeight = height() - 40;
	float binWidth = histWidth / 256;

	painter.fillRect(20, 20, histWidth, histHeight, QBrush(Qt::white));

	int maxBinHeight = *std::max_element(histogram.begin(), histogram.end());
	int inRangeCount = 0;
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
			inRangeCount += histogram[i];
		}

		QRect rect(20 + i * binWidth, histHeight - normalizedHeight + 20, binWidth, normalizedHeight);
		painter.fillRect(rect, QBrush(color));
	}

	QFont font = painter.font();
	font.setPointSize(12);
	painter.setFont(font);
	for (int i = 0; i <= 255; i += 32) {
		painter.drawText(20 + i * binWidth, height() - 5, QString::number(i));
	}

	painter.drawText(20, 15, "Normalized Histogram");

	lowerThresholdX = 20 + lowerThreshold * binWidth;
	upperThresholdX = 20 + upperThreshold * binWidth;

	painter.setPen(QPen(Qt::red, 6));
	painter.drawLine(lowerThresholdX, 20, lowerThresholdX, histHeight + 20);
	painter.drawLine(upperThresholdX, 20, upperThresholdX, histHeight + 20);

	painter.setPen(QPen(Qt::black));
	font.setPointSize(10);
	painter.drawText(lowerThresholdX + 10, histHeight + 15, QString("Lower: %1").arg(lowerThreshold));
	painter.drawText(upperThresholdX + 10, histHeight + 15, QString("Upper: %1").arg(upperThreshold));

	// 区间内点数（右上角）
	QString countText = QString("区间内点数: %1 , 总点数: %2")
		.arg(inRangeCount)
		.arg(static_cast<int>(pointCloud->size()));
	int textWidth = painter.fontMetrics().horizontalAdvance(countText);
	painter.drawText(
		width() - textWidth - 10,  // 右侧留 10px
		15,                        // 距离顶部 15px
		countText);
}

void ThresholdHistogramWidget::computeIntensityRange() {
	if (pointCloud)
	{
		minIntensity = std::numeric_limits<float>::max();
		maxIntensity = std::numeric_limits<float>::lowest();

		int sfIdx = PointCloudIO::get_intensity_idx(pointCloud);

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
	int sfIdx = PointCloudIO::get_intensity_idx(pointCloud);
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


void ThresholdHistogramWidget::onConfirmButtonClicked()
{
	// 点击“确定”按钮处理：
	// 先禁用点云的阈值过滤模式，然后生成新的点云
	if (pointCloud) {
		pointCloud->setUpperAndLowerThreshold(false);
	}

	ccPointCloud* cloud = new ccPointCloud;
	CloudProcess::filter_cloud_by_intensity(pointCloud,
		lowerThreshold / 256 * (maxIntensity - minIntensity) + minIntensity,
		upperThreshold / 256 * (maxIntensity - minIntensity) + minIntensity,
		cloud);
	cloud->setName("filtedCloud");
	emit addCloudToDB(cloud);
	this->hide();
}


void ThresholdHistogramWidget::closeEvent(QCloseEvent* event)
{
	setUpperAndLowerThreshold(false);
	hide();
	event->accept();
}
