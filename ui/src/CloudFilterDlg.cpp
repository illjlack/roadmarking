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
	// ��ʼ��UI���ֺͿؼ�
	confirmButton = new QPushButton(tr("ȷ��"), this);
	// ʹ�ô�ֱ���ֽ���ť����ֱ��ͼ�·�
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->addStretch();  // �Ϸ���������ռ��ֱ��ͼ��������
	QHBoxLayout* buttonLayout = new QHBoxLayout();
	buttonLayout->addStretch();             // ��൯������ʹ��ť����
	buttonLayout->addWidget(confirmButton); // ���ȷ����ť
	mainLayout->addLayout(buttonLayout);    // ��ˮƽ������ӵ������ֵײ�

	// ���Ӱ�ť����źŵ��ۺ���
	connect(confirmButton, &QPushButton::clicked, this, &ThresholdHistogramWidget::onConfirmButtonClicked);
}

ThresholdHistogramWidget::~ThresholdHistogramWidget()
{
	if (pointCloud)pointCloud->setUpperAndLowerThreshold(false);
}

void ThresholdHistogramWidget::setPointCloud(ccPointCloud* pointCloud, bool isfilterIntensity)
{
	this->pointCloud = pointCloud;
	if (isfilterIntensity)
	{
		computeIntensityRangeAndHistogramData();
		this->isfilterIntensity = isfilterIntensity;
	}
	else
	{
		computeZRangeAndHistogramData();
		this->isfilterIntensity = isfilterIntensity;
	}
	update();                // ������ʾ
}

void ThresholdHistogramWidget::setUpperAndLowerThreshold(bool is_has_threshold, float lowerPos, float upperPos)
{
	if (is_has_threshold) {
		// �����������ֵ��ʹ�ô����ֵ
		this->lowerPos = lowerPos;
		this->upperPos = upperPos;
	}
	else {
		// ���û�д�����ֵ��ʹ�õ������ݵ���С�����ǿ��ֵ��Ϊ��ֵ
		this->lowerPos = minScalar;
		this->upperPos = maxScalar;
	}
	if (pointCloud)
	{
		if(isfilterIntensity)pointCloud->setUpperAndLowerThreshold(is_has_threshold, getValue(lowerPos), getValue(upperPos), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max());
		else pointCloud->setUpperAndLowerThreshold(is_has_threshold, std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max(), getValue(lowerPos), getValue(upperPos));
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

	if (event->x() >= lowerPosX - 5 && event->x() <= lowerPosX + 5) {
		draggingLower = true; // ������������ֵ��������ʼ�϶�����ֵ
	}
	else if (event->x() >= upperPosX -5 && event->x() <= upperPosX + 5) {
		draggingUpper = true; // ������������ֵ��������ʼ�϶�����ֵ
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
			update(); // ������ʾ
		}
	}
	else if (draggingUpper)
	{
		if (newPos >= 0 && newPos <= 255 && newPos > lowerPos + 5)
		{
			setUpperAndLowerThreshold(true, lowerPos, newPos);
			update(); // ������ʾ
		}
	}
}

void ThresholdHistogramWidget::mouseReleaseEvent(QMouseEvent* event) {
	draggingLower = draggingUpper = false; // �ͷ��϶���־
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

	// �����ڵ��������Ͻǣ�
	QString countText = QString("�����ڵ���: %1 , �ܵ���: %2")
		.arg(inRangeCount)
		.arg(static_cast<int>(pointCloud->size()));
	int textWidth = painter.fontMetrics().horizontalAdvance(countText);
	painter.drawText(
		width() - textWidth - 10,  // �Ҳ��� 10px
		15,                        // ���붥�� 15px
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

		// �����������ݣ�����ǿ�ȵ���Сֵ�����ֵ
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float intensity = sf->getValue(i);
			minScalar = std::min(minScalar, intensity);
			maxScalar = std::max(maxScalar, intensity);
		}
		std::vector<int>(256, 0).swap(histogram); // ��ʼ��ֱ��ͼ����
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float intensity = sf->getValue(i);
			int bin = (int)(255 * (intensity - minScalar) / (maxScalar - minScalar)); // ӳ�䵽0-255����
			bin = std::min(std::max(bin, 0), 255); // ȷ��bin����Ч��Χ��
			++histogram[bin]; // ������Ӧbin�ļ���
		}
	}
}

void ThresholdHistogramWidget::computeZRangeAndHistogramData()
{
	if (pointCloud)
	{
		minScalar = std::numeric_limits<float>::max();
		maxScalar = std::numeric_limits<float>::lowest();

		// �����������ݣ�����ǿ�ȵ���Сֵ�����ֵ
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float z = pointCloud->getPoint(i)->z;
			minScalar = std::min(minScalar, z);
			maxScalar = std::max(maxScalar, z);
		}

		histogram.resize(256, 0); // ��ʼ��ֱ��ͼ����
		for (unsigned i = 0; i < pointCloud->size(); ++i)
		{
			const float z = pointCloud->getPoint(i)->z;
			int bin = (int)(255 * (z - minScalar) / (maxScalar - minScalar)); // ӳ�䵽0-255����
			bin = std::min(std::max(bin, 0), 255);
			++histogram[bin];
		}
	}
}

void ThresholdHistogramWidget::onConfirmButtonClicked()
{
	// �����ȷ������ť����
	// �Ƚ��õ��Ƶ���ֵ����ģʽ��Ȼ�������µĵ���
	if (pointCloud) {
		pointCloud->setUpperAndLowerThreshold(false);
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
	}
	emit addCloudToDB(cloud);
	this->hide();
}


void ThresholdHistogramWidget::closeEvent(QCloseEvent* event)
{
	setUpperAndLowerThreshold(false);
	hide();
	event->accept();
}

float ThresholdHistogramWidget::getValue(float pos)
{
	return pos / 256 * (maxScalar - minScalar) + minScalar;
}

int ThresholdHistogramWidget::getPos(float posX)
{
	// �������ұ߽���20���
	return (posX - 20) / ((width() - 40) / 256);
}

float ThresholdHistogramWidget::getPosX(float pos)
{
	float binWidth = (width() - 40) / 256;
	return 20 + pos * binWidth;
}
