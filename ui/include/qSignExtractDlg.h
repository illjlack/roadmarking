#pragma once
#pragma execution_character_set("utf-8")

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolButton>
#include <QGroupBox>
#include <QListWidget>
#include <QFrame>
#include <QSplitter>
#include <QLabel>
#include <QSettings>
#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QButtonGroup>
#include <QTreeWidget>
#include <QMouseEvent>
#include <QEvent>

#include "ccGLWindow.h"
#include "ccGLWindowSignalEmitter.h"
#include "ccMainAppInterface.h"
#include "ccBox.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#include "comm.h"
#include "CloudFilterDlg.h"

#include "PointCloudSelector.h"
#include "PointCloudDrawing.h"
#include "CloudObjectTreeWidget.h"

// 选择模式枚举
enum SelectionMode
{
	ENTITY_SELECTION,  // 实体选择状态
	POINT_SELECTION,   // 点选择模式
	DRAW_SELECTION,    // 多边形框选模式
	DRAW_MODEL	       // 模板绘制
};

/// <summary>
/// qSignExtractDlg 类用于点云提取对话框处理
/// </summary>
class qSignExtractDlg : public QDialog
{
	Q_OBJECT

public:
	qSignExtractDlg(ccMainAppInterface* app);
	~qSignExtractDlg();

	bool setCloud(std::vector<ccHObject*> cloud);
protected:
	void keyPressEvent(QKeyEvent* event) override;

private slots:
	void onAutoExtract();        // 全自动提取
	void onBoxSelectExtract();   // 框选提取
	void onPointGrowExtract();   // 点生长提取
	void onBoxClip();            // 框选截取
	void onMatchTemplateDirect();
	void onMatchTemplateByBox();
	void onMatchTemplate();
	void onRectClip();
	void onMakeModel();
	void onFilteCloudByIntensity();         // 过滤点云
	void onFilteCloudByZ();
	void onFilteGround();
	void onZebraExtract();

	//void onMatchTemplateByIntensity();
	//void onZebraExtractByIntensity();
	//void onPointGrowExtractByIntensity();

	void onItemPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3&, const CCVector3d&);
	void onItemPickedFast(ccHObject* entity, int subEntityID, int x, int y);
	void onLeftButtonClicked(int x, int y);
	void onLeftButtonDoubleClicked(int x, int y);
	void onRightButtonDoubleClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onButtonReleased();
	void onEntitySelectionChanged(ccHObject* entity);
	void onMouseWheelRotated(int delta);

	void addCloudToDB(ccPointCloud* cloud);

private:
	void showThresholdHistogram(ccPointCloud* pointCloud, bool isfilterIntensity, bool is_has_threshold = true, float lowerThreshold = 50, float upperThreshold = 250);

	bool m_selecting = false;          // 是否正在选择
	ccHObject* p_select_cloud = nullptr; // 选择的点云对象
	ThresholdHistogramWidget* histogramWidget;
	QWidget* m_glWidget = nullptr;      // OpenGL Widget
	ccMainAppInterface* m_app = nullptr; // 主程序接口
	ccGLWindowInterface* m_glWindow = nullptr;  // OpenGL 窗口接口

	CloudObjectTreeWidget* m_objectTree = nullptr; // 对象树
	SelectionMode m_selectionMode = ENTITY_SELECTION; // 选择模式

	cc_interact::PointCloudSelector* m_pointCloudSelector; // 前景折线编辑器
	cc_drawing::PointCloudDrawer* m_pointCloudDrawer;

	std::function<void(ccHObject*, unsigned)> m_pick_callback; // 选点任务的回调函数
};
