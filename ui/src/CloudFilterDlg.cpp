#include "CloudFilterDlg.h"

#include <QPainter>
#include <QMouseEvent>
#include <algorithm>

ThresholdHistogramWidget::ThresholdHistogramWidget(QWidget* parent)
	: QWidget(parent), lowerThreshold(50), upperThreshold(200) {}

void ThresholdHistogramWidget::setData(const std::vector<int>& histogram) {
	this->histogram = histogram;
	calculateSum();
}

void ThresholdHistogramWidget::setLowerThreshold(int value) {
	lowerThreshold = value;
	update();
}

void ThresholdHistogramWidget::setUpperThreshold(int value) {
	upperThreshold = value;
	update();
}

void ThresholdHistogramWidget::paintEvent(QPaintEvent* event) {
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	drawHistogram(painter);
}

void ThresholdHistogramWidget::mousePressEvent(QMouseEvent* event) {
	if (event->x() >= lowerThresholdX && event->x() <= lowerThresholdX + 10) {
		draggingLower = true;
	}
	else if (event->x() >= upperThresholdX && event->x() <= upperThresholdX + 10) {
		draggingUpper = true;
	}
}

void ThresholdHistogramWidget::mouseMoveEvent(QMouseEvent* event) {
	if (draggingLower) {
		int newThreshold = event->x() - 20;
		if (newThreshold >= 0 && newThreshold <= upperThreshold - 10) {
			lowerThreshold = newThreshold;
			update();
		}
	}
	else if (draggingUpper) {
		int newThreshold = event->x() - 20;
		if (newThreshold <= width() && newThreshold >= lowerThreshold + 10) {
			upperThreshold = newThreshold;
			update();
		}
	}
}

void ThresholdHistogramWidget::mouseReleaseEvent(QMouseEvent* event) {
	draggingLower = draggingUpper = false;
}

void ThresholdHistogramWidget::calculateSum() {
	belowThresholdSum = 0;
	aboveThresholdSum = 0;
	for (int i = 0; i < 256; ++i) {
		if (i <= lowerThreshold) {
			belowThresholdSum += histogram[i];
		}
		else if (i >= upperThreshold) {
			aboveThresholdSum += histogram[i];
		}
	}
}

void ThresholdHistogramWidget::drawHistogram(QPainter& painter) {
	int histWidth = width() - 40;
	int histHeight = height() - 40;
	int binWidth = histWidth / 256;

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

void showThresholdHistogram(const std::vector<int>& data, int lowerThreshold, int upperThreshold) {
	QDialog histogramWindow;
	histogramWindow.setWindowTitle("Threshold Histogram");

	ThresholdHistogramWidget* histogramWidget = new ThresholdHistogramWidget(&histogramWindow);
	histogramWidget->setData(data);

	histogramWidget->setLowerThreshold(lowerThreshold);
	histogramWidget->setUpperThreshold(upperThreshold);

	QVBoxLayout* layout = new QVBoxLayout(&histogramWindow);
	layout->addWidget(histogramWidget);

	histogramWindow.setLayout(layout);
	histogramWindow.resize(520, 380);
	histogramWindow.exec();
}
