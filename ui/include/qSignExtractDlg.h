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
// ǰ������
class CloudObjectTreeWidget;

// ѡ��ģʽö��
enum SelectionMode
{
	ENTITY_SELECTION,  // ʵ��ѡ��״̬
	POINT_SELECTION,   // ��ѡ��ģʽ
	DRAW_SELECTION,    // ����ο�ѡģʽ
	DRAW_MODEL	       // ģ�����
};

/// <summary>
/// qSignExtractDlg �����ڵ�����ȡ�Ի�����
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
	void onAutoExtract();        // ȫ�Զ���ȡ
	void onBoxSelectExtract();   // ��ѡ��ȡ
	void onPointGrowExtract();   // ��������ȡ
	void onBoxClip();            // ��ѡ��ȡ
	void onRectClip();
	void onMakeModel();
	void onFilteCloudByIntensity();         // ���˵���
	void onFilteCloudByZ();
	void onZebraExtract();

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

	bool m_selecting = false;          // �Ƿ�����ѡ��
	ccHObject* p_select_cloud = nullptr; // ѡ��ĵ��ƶ���
	ThresholdHistogramWidget* histogramWidget;
	QWidget* m_glWidget = nullptr;      // OpenGL Widget
	ccMainAppInterface* m_app = nullptr; // ������ӿ�
	ccGLWindowInterface* m_glWindow = nullptr;  // OpenGL ���ڽӿ�

	CloudObjectTreeWidget* m_objectTree = nullptr; // ������
	SelectionMode m_selectionMode = ENTITY_SELECTION; // ѡ��ģʽ

	cc_interact::PointCloudSelector* m_pointCloudSelector; // ǰ�����߱༭��
	cc_drawing::PointCloudDrawer* m_pointCloudDrawer;
};

/// <summary>
/// CloudObjectTreeWidget �����ڹ�����ƶ���Ŀ¼��
/// </summary>
class CloudObjectTreeWidget : public QTreeWidget
{
	Q_OBJECT

public:
	explicit CloudObjectTreeWidget(QWidget* parent = nullptr);

	/// <summary>
	/// ��ʼ��Ŀ¼�����󶨴��ں͸��ڵ����
	/// </summary>
	void initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud, const std::vector<ccHObject*>& objects);

	void addCloud(ccPointCloud* cloud, ccHObject* parent = nullptr);  // ��ӵ���

	void relase();	 // �ͷŵ��Ƶ�ԭ����

	void getAllPointClouds(std::vector<ccPointCloud*>& pointClouds);  // ��ȡ���е���

signals:
	void async_refresh();

public slots:
	void refresh();

protected:
	void contextMenuEvent(QContextMenuEvent* event) override;

	void getAllPointCloudsRecursive(ccHObject* object, std::vector<ccPointCloud*>& pointClouds); // �ݹ��ȡ���е���

	void loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem); // �������ڵ�

private:
	ccHObject* root = nullptr;  // ���ڵ�
	ccHObject** pp_select_cloud = nullptr;  // ѡ��ĵ��ƶ���ָ��
	ccMainAppInterface* m_app = nullptr;  // ������ӿ�
	ccGLWindowInterface* m_glWindow = nullptr;  // OpenGL ���ڽӿ�

	ccGenericGLDisplay* originalDisplay;  // ԭʼ��ʾ����


};
