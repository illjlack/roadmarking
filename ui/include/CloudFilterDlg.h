#pragma once
#pragma execution_character_set("utf-8")

#include <QDialog>
#include <QVBoxLayout>
#include <QPainter>
#include <QMouseEvent>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <vector>

class ThresholdHistogramWidget : public QWidget {
	Q_OBJECT

public:
	explicit ThresholdHistogramWidget(QWidget* parent = nullptr);
	void setData(const std::vector<int>& histogram);
	void setLowerThreshold(int value);
	void setUpperThreshold(int value);

protected:
	void paintEvent(QPaintEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;

private:
	std::vector<int> histogram;
	int lowerThreshold, upperThreshold;
	int belowThresholdSum = 0, aboveThresholdSum = 0;
	int lowerThresholdX = 0, upperThresholdX = 0;
	bool draggingLower = false, draggingUpper = false;

	void calculateSum();
	void drawHistogram(QPainter& painter);
};

void showThresholdHistogram(const std::vector<int>& data, int lowerThreshold, int upperThreshold);
