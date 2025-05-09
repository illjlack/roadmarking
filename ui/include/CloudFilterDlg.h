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

#include <ccPointCloud.h>

class ThresholdHistogramWidget : public QWidget {
	Q_OBJECT

public:
	// 构造函数，初始化阈值
	explicit ThresholdHistogramWidget(QWidget* parent = nullptr);

	~ThresholdHistogramWidget();
	// 设置点云数据，计算强度范围并更新直方图
	void setPointCloud(ccPointCloud* pointCloud, bool isfilterIntensity = true);

	// 设置上下阈值，若没有传入阈值，则根据点云数据自动计算
	void setUpperAndLowerThreshold(bool is_has_threshold = false, float lowerThreshold = 0.0, float upperThreshold = 0.0);

protected:
	// 重写绘制事件，负责绘制直方图
	void paintEvent(QPaintEvent* event) override;

	// 鼠标按下事件，开始拖动阈值
	void mousePressEvent(QMouseEvent* event) override;

	// 鼠标移动事件，更新阈值位置
	void mouseMoveEvent(QMouseEvent* event) override;

	// 鼠标释放事件，停止拖动阈值
	void mouseReleaseEvent(QMouseEvent* event) override;

	void closeEvent(QCloseEvent* event) override;

signals:
	void addCloudToDB(ccPointCloud* cloud);

private slots:
	// 槽函数：处理点击确定按钮的动作
	void onConfirmButtonClicked();

private:
	QPushButton* confirmButton;       // 确定按钮（确认阈值选择）
	ccPointCloud* pointCloud = nullptr; // 点云数据指针
	std::vector<int> histogram; // 存储直方图数据
	float lowerPos, upperPos; // 下限和上限阈值
	int lowerPosX = 0, upperPosX = 0; // 阈值在直方图中的位置
	bool draggingLower = false, draggingUpper = false; // 是否正在拖动阈值
	float minScalar = std::numeric_limits<float>::max(), maxScalar = std::numeric_limits<float>::lowest(); // 最小和最大强度值

	bool isfilterIntensity;

	// 绘制直方图
	void drawHistogram(QPainter& painter);

	// 计算点云中的最小和最大强度值
	void computeIntensityRangeAndHistogramData();

	void computeZRangeAndHistogramData();

	float getValue(float pos);

	int getPos(float posX);
	float getPosX(float pos);
};
