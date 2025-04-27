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
	// ���캯������ʼ����ֵ
	explicit ThresholdHistogramWidget(QWidget* parent = nullptr);

	~ThresholdHistogramWidget();
	// ���õ������ݣ�����ǿ�ȷ�Χ������ֱ��ͼ
	void setPointCloud(ccPointCloud* pointCloud);

	// ����������ֵ����û�д�����ֵ������ݵ��������Զ�����
	void setUpperAndLowerThreshold(bool is_has_threshold = false, float lowerThreshold = 0.0, float upperThreshold = 0.0);

protected:
	// ��д�����¼����������ֱ��ͼ
	void paintEvent(QPaintEvent* event) override;

	// ��갴���¼�����ʼ�϶���ֵ
	void mousePressEvent(QMouseEvent* event) override;

	// ����ƶ��¼���������ֵλ��
	void mouseMoveEvent(QMouseEvent* event) override;

	// ����ͷ��¼���ֹͣ�϶���ֵ
	void mouseReleaseEvent(QMouseEvent* event) override;

	void closeEvent(QCloseEvent* event) override;
signals:
	void addCloudToDB(ccPointCloud* cloud);

private slots:
	// �ۺ�����������ȷ����ť�Ķ���
	void onConfirmButtonClicked();

private:
	QPushButton* confirmButton;       // ȷ����ť��ȷ����ֵѡ��
	ccPointCloud* pointCloud = nullptr; // ��������ָ��
	std::vector<int> histogram; // �洢ֱ��ͼ����
	float lowerThreshold, upperThreshold; // ���޺�������ֵ
	int lowerThresholdX = 0, upperThresholdX = 0; // ��ֵ��ֱ��ͼ�е�λ��
	bool draggingLower = false, draggingUpper = false; // �Ƿ������϶���ֵ
	float minIntensity = std::numeric_limits<float>::max(), maxIntensity = std::numeric_limits<float>::lowest(); // ��С�����ǿ��ֵ



	// ����ֱ��ͼ
	void drawHistogram(QPainter& painter);

	// ��������е���С�����ǿ��ֵ
	void computeIntensityRange();

	// ����ֱ��ͼ����
	void updateHistogramData();

};
