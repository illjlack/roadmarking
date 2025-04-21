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

class CloudBackup;
class CloudObjectTreeWidget;
class ForegroundPolylineEditor;

// ѡ��ģʽö��
enum SelectionMode
{
	ENTITY_SELECTION,  // ʵ��ѡ��״̬
	POINT_SELECTION,   // ��ѡ��ģʽ
	DRAW_SELECTION     // ����ο�ѡģʽ
};

class qSignExtractDlg : public QDialog
{
	Q_OBJECT

public:
	qSignExtractDlg(ccMainAppInterface* app);
	~qSignExtractDlg();

	/// <summary>
	/// ����Ŀ�����
	/// </summary>
	/// <param name="cloud">Ŀ�����</param>
	/// <returns>�Ƿ�ɹ�</returns>
	bool setCloud(ccCloudPtr cloud);

protected:


	
	CCVector3 screenToWorld(int x, int y);

private slots:
	void onAutoExtract();        // ȫ�Զ���ȡ
	void onBoxSelectExtract();   // ��ѡ��ȡ
	void onPointGrowExtract();   // ��������ȡ
	void onBoxClip();            // ��ѡ��ȡ

	void onItemPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3&, const CCVector3d&);
	void onItemPickedFast(ccHObject* entity, int subEntityID, int x, int y);
	void onLeftButtonClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onButtonReleased();
	void onEntitySelectionChanged(ccHObject* entity);
	void onMouseWheelRotated(int delta);

private:
	bool m_selecting = false;

	ccHObject* p_select_cloud = nullptr;

	QWidget* m_glWidget = nullptr;
	ccMainAppInterface* m_app = nullptr;
	ccGLWindowInterface* m_glWindow = nullptr;
	
	std::shared_ptr<CloudBackup> m_cloudBackup;
	CloudObjectTreeWidget* m_objectTree = nullptr;
	SelectionMode m_selectionMode = ENTITY_SELECTION;



	ForegroundPolylineEditor* foregroundPolylineEditor;


};

// ================================================================== ForegroundPolylineEditor
class ForegroundPolylineEditor: public QObject
{
	Q_OBJECT
public:
	// ���캯��������gl���ڡ����ƶ���Ȳ���
	ForegroundPolylineEditor(ccGLWindowInterface* glWindow);

	// ��������������ʱ������Դ
	~ForegroundPolylineEditor();

	void setSelectCloudPtr(ccHObject** select_cloud);
	void setCallbackfunc(std::function<void()> callback);
	void startDraw();

	void onLeftButtonClicked(int x, int y);
	void onMouseMoved(int x, int y, Qt::MouseButtons button);
	void onMouseWheelRotated(int delta);

private:
	void updatePoly();
	void finishDraw();

	ccGLWindowInterface* m_glWindow;             // ������ʾ��OpenGL����
	ccPointCloud* m_pointCloud;                  // ���ڰ����ߵĵ��ƶ���(2D��Ļ������)
	ccPolyline* m_foregroundPolyline;            // ǰ�����߶���
	std::vector<CCVector3d> m_3DPoints;		 // ���ڱ�����λ��

	// ���ݱ��
	ccGLWindowInterface::INTERACTION_FLAGS interaction_flags_backup;
	ccGLWindowInterface::PICKING_MODE picking_mode_backup;

	ccHObject** pp_select_cloud = nullptr;
	std::function<void()> m_callback;            // �ص�����
signals:
	void draw_start();
	void draw_finish();
};


/// <summary>
/// ����Ŀ¼��������ѡ�е��ƣ����õ��ƿɼ��ԣ�ɾ������
/// </summary>
class CloudObjectTreeWidget : public QTreeWidget
{
	Q_OBJECT

public:
	explicit CloudObjectTreeWidget(QWidget * parent = nullptr);

	/// <summary>
	/// ��ʼ�����󶨴��ڣ����ڵ����
	/// </summary>
	/// <param name="win">���ڽӿ�</param>
	/// <param name="app">������ӿ�</param>
	/// <param name="select_cloud">���ڵ����</param>
	void initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud);

	/// <summary>
	/// ˢ��Ŀ¼
	/// </summary>
	void refresh();
protected:
	void contextMenuEvent(QContextMenuEvent* event) override;
	void loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem);
private:
	ccHObject* root = nullptr;
	ccHObject** pp_select_cloud = nullptr;
	ccMainAppInterface* m_app = nullptr;
	ccGLWindowInterface* m_glWindow = nullptr;
};


/// <summary>
/// ����״̬������ָ�������
/// ���ڲ������ʱ�޸ĵ������Ժ�֧��һ����ԭ(������Ҫ�������Լ������к��ӵ�display����)
/// </summary>
class CloudBackup
{
public:
	CloudBackup();
	~CloudBackup();

	/// <summary>
	/// ���ݵ���״̬
	/// </summary>
	/// <param name="cloud">����Ŀ�����</param>
	void backup(ccCloudPtr cloud);

	/// <summary>
	/// �ָ�����״̬
	/// </summary>
	void restore();

	/// <summary>
	/// ������
	/// </summary>
	void clear();

private:
	int displayedSFIndex;
	bool wasVisible;
	bool wasEnabled;
	bool wasSelected;
	bool colorsWereDisplayed;
	bool sfWasDisplayed;

	ccCloudPtr ref;
	ccGenericGLDisplay* originalDisplay;
};
