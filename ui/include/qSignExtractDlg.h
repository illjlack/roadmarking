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

/// <summary>
/// ����״̬������ָ�������
/// ���ڲ������ʱ�޸ĵ������Ժ�֧��һ����ԭ
/// </summary>
struct CloudBackup
{
	CloudBackup();
	~CloudBackup();

	/// ����״̬
	void backup(ccCloudPtr cloud);

	/// �ָ�״̬
	void restore();

	/// ������Դ
	void clear();

private:
	ccCloudPtr ref;

	// ԭʼ״̬����
	ccGenericGLDisplay* originalDisplay;
	bool wasVisible;
	bool wasEnabled;
	bool wasSelected;
	bool colorsWereDisplayed;
	bool sfWasDisplayed;
	int displayedSFIndex;
	bool hadColors;

	RGBAColorsTableType* backupColors;
};


class CloudObjectTreeWidget;

// ѡ��ģʽö��
enum SelectionMode
{
	NO_SELECTION,      // ��ѡ��״̬
	POINT_SELECTION,   // ��ѡ��ģʽ
	RECT_SELECTION,    // ���ο�ѡģʽ
	POLY_SELECTION     // ����ο�ѡģʽ
};

class qSignExtractDlg : public QDialog
{
	Q_OBJECT

public:
	qSignExtractDlg(ccMainAppInterface* app);
	~qSignExtractDlg();

	/// ���ò���ʾĿ�����
	bool setCloud(ccCloudPtr cloud);

protected:


	// ��Ļ����ת�������꣨z=0 ƽ�棩
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
private:
	ccMainAppInterface* m_app = nullptr;
	ccGLWindowInterface* m_glWindow = nullptr;
	QWidget* m_glWidget = nullptr;

	// Ŀ¼�� 
	CloudObjectTreeWidget* m_objectTree = nullptr;

	// ���Ʊ���
	CloudBackup m_cloudBackup;

	SelectionMode m_selectionMode = POLY_SELECTION;

	bool m_selecting = false;
	QPoint m_selectionStart;
	QPoint m_mousePos;


	ccHObject* p_select_cloud = nullptr;
};


/// <summary>
/// �ڲ��ࣺ����Ŀ¼��������ѡ�е��ƣ����õ��ƿɼ��ԣ�ɾ������
/// </summary>
class CloudObjectTreeWidget : public QTreeWidget
{
	Q_OBJECT

public:
	explicit CloudObjectTreeWidget(QWidget * parent = nullptr);

	/// ��ʼ��Ŀ¼�����󶨴��ں� app��
	void initialize(ccGLWindowInterface* win, ccMainAppInterface* app, ccHObject** select_cloud);

	// ˢ��Ŀ¼�����¼�������
	void refresh();


protected:
	// �Ҽ��˵�, �ṩɾ��ѡ��
	void contextMenuEvent(QContextMenuEvent* event) override;
	// �����ļ�������Ŀ¼
	void loadTreeItem(ccHObject* object, QTreeWidgetItem* parentItem);
private:
	ccGLWindowInterface* m_glWindow = nullptr;
	ccMainAppInterface* m_app = nullptr;
	ccHObject** pp_select_cloud = nullptr;
	ccHObject* root = nullptr;
};
