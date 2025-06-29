#include "CloudFilterDlg.h"

#include <QPainter>
#include <QMouseEvent>
#include <algorithm>

#include "CloudProcess.h"
#include <QApplication>
using namespace roadmarking;

ThresholdHistogramWidget::ThresholdHistogramWidget(QWidget* parent)
	: QWidget(parent), lowerPos(50), upperPos(200)
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
	if (pointCloud) {
		pointCloud->resetVisibilityArray();
		pointCloud->getDisplay()->redraw(false, true);
	}
}

void ThresholdHistogramWidget::setPointCloud(ccPointCloud* pointCloud, bool isfilterIntensity)
{
	this->pointCloud = pointCloud;
	
	// 保存原始显示状态
	if (pointCloud) {
		originalDisplayedSFIndex = pointCloud->getCurrentDisplayedScalarFieldIndex();
		originalShowSF = pointCloud->sfShown();
	}
	
	if (isfilterIntensity)
	{
		computeIntensityRangeAndHistogramData();
		this->isfilterIntensity = isfilterIntensity;
	}
	else
	{
		computeZRangeAndHistogramData();
		this->isfilterIntensity = isfilterIntensity;
		
		// 过滤高程时，设置高程为显示标量
		PointCloudIO::apply_height_as_scalar(pointCloud);
	}
	update();                // 更新显示
}

void ThresholdHistogramWidget::setUpperAndLowerThreshold(bool is_has_threshold, float lowerPos, float upperPos)
{
	if (is_has_threshold) {
		// 如果传入了阈值，使用传入的值
		this->lowerPos = lowerPos;
		this->upperPos = upperPos;
	}
	else {
		// 如果没有传入阈值，使用点云数据的最小和最大强度值作为阈值
		this->lowerPos = minScalar;
		this->upperPos = maxScalar;
	}
	if (pointCloud)
	{
		if(isfilterIntensity) {
			// 强度过滤：隐藏不在阈值范围内的点
			pointCloud->hidePointsByScalarValue(getValue(lowerPos), getValue(upperPos));
		} else {
			// 高程过滤：隐藏不在阈值范围内的点
			pointCloud->hidePointsByScalarValue(getValue(lowerPos), getValue(upperPos));
		}
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

	if (event->x() >= lowerPosX - 5 && event->x() <= lowerPosX + 5) {
		draggingLower = true; // 如果点击在下阈值附近，开始拖动下阈值
	}
	else if (event->x() >= upperPosX -5 && event->x() <= upperPosX + 5) {
		draggingUpper = true; // 如果点击在上阈值附近，开始拖动上阈值
	}
}

void ThresholdHistogramWidget::mouseMoveEvent(QMouseEvent* event)
{
	int newPos = getPos(event->x());
	if (draggingLower)
	{
		if (newPos >= 0 && newPos <= 255 && newPos < upperPos - 5)
		{
			setUpperAndLowerThreshold(true, newPos, upperPos);
			update(); // 更新显示
		}
	}
	else if (draggingUpper)
	{
		if (newPos >= 0 && newPos <= 255 && newPos > lowerPos + 5)
		{
			setUpperAndLowerThreshold(true, lowerPos, newPos);
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
		if (i <= lowerPos) {
			color = QColor(0, 255, 0);
		}
		else if (i <= upperPos) {
			color = QColor(255, 255, 0);
			inRangeCount += histogram[i];
		}
		else {
			color = QColor(0, 0, 255);
		}

		QRect rect(getPosX(i), histHeight - normalizedHeight + 20, binWidth, normalizedHeight);
		painter.fillRect(rect, QBrush(color));
	}

	QFont font = painter.font();
	font.setPointSize(12);
	painter.setFont(font);
	for (int i = 0; i <= 255; i += 32)
	{
		painter.drawText(getPosX(i), height() - 5, QString::number(getValue(i)));
	}

	painter.drawText(20, 15, "Normalized Histogram");

	lowerPosX = getPosX(lowerPos);
	upperPosX = getPosX(upperPos);

	painter.setPen(QPen(Qt::red, 6));
	painter.drawLine(lowerPosX, 20, lowerPosX, histHeight + 20);
	painter.drawLine(upperPosX, 20, upperPosX, histHeight + 20);

	painter.setPen(QPen(Qt::black));
	font.setPointSize(10);
	painter.drawText(lowerPosX + 10, histHeight + 15, QString("Lower: %1").arg(getValue(lowerPos)));
	painter.drawText(upperPosX + 10, histHeight + 15, QString("Upper: %1").arg(getValue(upperPos)));

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

void ThresholdHistogramWidget::computeIntensityRangeAndHistogramData()
{
	if (pointCloud)
	{
		minScalar = std::numeric_limits<float>::max();
		maxScalar = std::numeric_limits<float>::lowest();

		int sfIdx = PointCloudIO::get_intensity_idx(pointCloud);

		auto sf = pointCloud->getScalarField(sfIdx);

		if (!sf)return;

		// 遍历点云数据，计算强度的最小值和最大值
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float intensity = sf->getValue(i);
			minScalar = std::min(minScalar, intensity);
			maxScalar = std::max(maxScalar, intensity);
		}
		std::vector<int>(256, 0).swap(histogram); // 初始化直方图数据
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float intensity = sf->getValue(i);
			int bin = (int)(255 * (intensity - minScalar) / (maxScalar - minScalar)); // 映射到0-255区间
			bin = std::min(std::max(bin, 0), 255); // 确保bin在有效范围内
			++histogram[bin]; // 增加相应bin的计数
		}
	}
}

void ThresholdHistogramWidget::computeZRangeAndHistogramData()
{
	if (pointCloud)
	{
		minScalar = std::numeric_limits<float>::max();
		maxScalar = std::numeric_limits<float>::lowest();

		// 遍历点云数据，计算强度的最小值和最大值
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float z = pointCloud->getPoint(i)->z;
			minScalar = std::min(minScalar, z);
			maxScalar = std::max(maxScalar, z);
		}

		histogram.resize(256, 0); // 初始化直方图数据
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float z = pointCloud->getPoint(i)->z;
			int bin = (int)(255 * (z - minScalar) / (maxScalar - minScalar)); // 映射到0-255区间
			bin = std::min(std::max(bin, 0), 255);
			++histogram[bin];
		}
	}
}

void ThresholdHistogramWidget::onConfirmButtonClicked()
{
	// 点击"确定"按钮处理：
	// 先恢复点云的显示状态，然后生成新的点云
	if (pointCloud) {
		pointCloud->resetVisibilityArray();
	}

	ccPointCloud* cloud = new ccPointCloud;
	if (isfilterIntensity)
	{
		CloudProcess::filter_cloud_by_intensity(pointCloud,
			getValue(lowerPos),
			getValue(upperPos),
			cloud);
		cloud->setName("filtedCloud_by_intensity");
	}
	else
	{
		CloudProcess::filter_cloud_by_z(pointCloud,
			getValue(lowerPos),
			getValue(upperPos),
			cloud);
		cloud->setName("filtedCloud_by_Z");
		
		// 恢复原始显示状态
		if (originalDisplayedSFIndex >= 0) {
			pointCloud->setCurrentDisplayedScalarField(originalDisplayedSFIndex);
		}
		pointCloud->showSF(originalShowSF);
		pointCloud->getDisplay()->redraw(false, true);
	}
	emit addCloudToDB(cloud);
	this->hide();
}


void ThresholdHistogramWidget::closeEvent(QCloseEvent* event)
{
	// 恢复点云显示状态
	if (pointCloud) {
		pointCloud->resetVisibilityArray();
	}
	
	// 恢复原始显示状态
	if (pointCloud && !isfilterIntensity) {
		// 只有在过滤高程时才需要恢复，因为强度过滤不会改变显示状态
		if (originalDisplayedSFIndex >= 0) {
			pointCloud->setCurrentDisplayedScalarField(originalDisplayedSFIndex);
		}
		pointCloud->showSF(originalShowSF);
		pointCloud->getDisplay()->redraw(false, true);
	}
	
	hide();
	event->accept();
}

float ThresholdHistogramWidget::getValue(float pos)
{
	return pos / 256 * (maxScalar - minScalar) + minScalar;
}

int ThresholdHistogramWidget::getPos(float posX)
{
	// 窗口左右边界留20宽度
	return (posX - 20) / ((width() - 40) / 256);
}

float ThresholdHistogramWidget::getPosX(float pos)
{
	float binWidth = (width() - 40) / 256;
	return 20 + pos * binWidth;
}
