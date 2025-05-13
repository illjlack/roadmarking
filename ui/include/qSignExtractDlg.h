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

#define INF 0x3f3f3f3f

// ǰ������
class CloudObjectTreeWidget;
class ForegroundPolylineEditor;

// ѡ��ģʽö��
enum SelectionMode
{
	ENTITY_SELECTION,  // ʵ��ѡ��״̬
	POINT_SELECTION,   // ��ѡ��ģʽ
	DRAW_SELECTION     // ����ο�ѡģʽ
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
	void onFilteCloudByIntensity();         // ���˵���
	void onFilteCloudByZ();

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

	ForegroundPolylineEditor* m_foregroundPolylineEditor; // ǰ�����߱༭��
};

/// <summary>
/// ForegroundPolylineEditor �����ڱ༭ǰ������
/// </summary>
class ForegroundPolylineEditor : public QObject
{
	Q_OBJECT

public:
	ForegroundPolylineEditor(ccGLWindowInterface* glWindow);
	~ForegroundPolylineEditor();

	void setSelectCloudPtr(ccHObject** select_cloud);
	void setCallbackfunc(std::function<void()> callback);
	void closeLine();
	void startDraw();

	void onLeftButtonClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onDoubleLeftButtonClicked(int x, int y);
	void onDoubleRightButtonClicked(int x, int y);
	void onMouseWheelRotated(int delta);

	void onKeyPressEvent(QKeyEvent* event);

	void getPoints(std::vector<CCVector3d>& polyline);

	void setDraw(int max_points_num, bool isClosed);

	void resetDraw();
private:
	void updatePoly();
	void finishDraw(bool doAction);

	ccGLWindowInterface* m_glWindow;      // ������ʾ��OpenGL����
	ccPointCloud* m_pointCloud;           // ���ڰ����ߵĵ��ƶ���(2D��Ļ������)
	ccPolyline* m_foregroundPolyline;     // ǰ�����߶���
	std::vector<CCVector3d> m_3DPoints;   // ���ڱ�����λ��

	// ���ݱ��
	ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
	ccGLWindowInterface::PICKING_MODE picking_mode_backup;

	ccHObject** pp_select_cloud = nullptr;  // ѡ��ĵ��ƶ���ָ��
	std::function<void()> m_callback;       // �ص�����

	bool isFreezeUI = false;  // �Ƿ񶳽�UI

	int max_points_num = INF;
	bool isClosed = true;

signals:
	void draw_start();  // ��ʼ����
	void draw_finish(); // �������
	void update_tree(); // ��������ͼ
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
