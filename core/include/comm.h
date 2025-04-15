#pragma once
#pragma execution_character_set("utf-8")

#define DEBUG

#include <ccMainAppInterface.h>
extern ccMainAppInterface* GApp;

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <Eigen/Core>
#include <QString.h>
#include <vector>
#include <sstream>
#include <pcl/octree/octree.h>

// ======================================================= ����

/// <summary>
/// �����ͷ�����Ȩ(���Զ�����)������ָ��
/// </summary>
/// <typeparam name="T">��������</typeparam>
template <typename T>
class MovableSharedPtr : public std::shared_ptr<T> {
private:
	std::shared_ptr<std::atomic<bool>> m_is_move;  // �����ת�Ʊ��

public:
	// Ĭ�Ϲ��죬֧��ԭ��ָ�룬���Զ���ɾ����
	template <typename Deleter = std::default_delete<T>>
	explicit MovableSharedPtr(T* ptr = nullptr, Deleter deleter = Deleter())
	{
		auto moveFlag = std::make_shared<std::atomic<bool>>(false);
		m_is_move = moveFlag;

		std::shared_ptr<T>::operator=(
			std::shared_ptr<T>(ptr, [moveFlag, deleter](T* p) {
				if (!*moveFlag) {
					deleter(p);
				}
				})
			);
	}

	// nullptr ���ݹ��캯��
	MovableSharedPtr(std::nullptr_t)
		: std::shared_ptr<T>(nullptr),
		m_is_move(std::make_shared<std::atomic<bool>>(false))
	{}

	// ���ƹ���
	MovableSharedPtr(const MovableSharedPtr& other)
		: std::shared_ptr<T>(other),
		m_is_move(other.m_is_move)
	{}

	// ��ֵ�����
	MovableSharedPtr& operator=(const MovableSharedPtr& other) {
		if (this != &other) {
			std::shared_ptr<T>::operator=(other);
			m_is_move = other.m_is_move;
		}
		return *this;
	}

	/// <summary>
	/// ��������Ȩ
	/// ��ԭ���˹���ָ���ڲ�����һ������ָ��ָ����,�˱�Ǳ�ʾ�Ƿ��ͷ�������Ȩ��
	/// </summary>
	/// <returns>��ָͨ��</returns>
	T* release() {
		*m_is_move = true;  // ������ж�����ת��
		return this->get();
	}

	/// <summary>
	/// �жϹ���ָ���Ƿ񽻳�����Ȩ
	/// </summary>
	/// <returns>�Ƿ񽻳�����Ȩ</returns>
	bool isMoved() const {
		return *m_is_move;
	}
};

using ccCloudPtr = MovableSharedPtr<ccPointCloud>;

using PCLPoint = pcl::PointXYZ;
using PCLCloud = pcl::PointCloud<pcl::PointXYZ>;
using PCLCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

using PCLPointXYZI = pcl::PointXYZI;
using PCLCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PCLCloudXYZIPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;


using PCLOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;
using PCLOctreePtr = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr;

// ============================================pcl��ʾ����
template <typename... PointClouds>
void visualizePointClouds(
	const QString& title,
	const std::vector<QString>& dynamic_text = {},  // ��̬�ı�
	PointClouds&&... clouds  // �۵����������ڽ��ն������
)
{
	pcl::visualization::PCLVisualizer viewer(title.toStdString());

	std::vector<std::string> cloud_ids;
	Eigen::Vector4f overall_centroid(0.0f, 0.0f, 0.0f, 0.0f);
	int total_points = 0;

	// ʹ���۵�������Ӷ������
	size_t cloud_index = 0;
	// ʹ���۵����ʽչ������ӵ���
	([&] {
		// Ϊÿ���������ò�ͬ����ɫ
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(
			std::forward<PointClouds>(clouds),
			255 - (cloud_index * 50),
			0,
			cloud_index * 50
		);

		std::stringstream id;
		id << "cloud_" << cloud_index;
		cloud_ids.push_back(id.str());

		// ��������ӵ���ͼ��
		viewer.addPointCloud(std::forward<PointClouds>(clouds), color, cloud_ids[cloud_index]);

		// ����ÿ�����Ƶ�����
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*std::forward<PointClouds>(clouds), centroid);
		overall_centroid += centroid;  // �ۼ�
		total_points += clouds->size();

		++cloud_index;
		}(), ...);  // ʹ���۵����ʽչ���������

		// �������е��Ƶ���������
	overall_centroid /= cloud_index;

	// �������ϵ
	viewer.addCoordinateSystem(1.0);
	viewer.setBackgroundColor(0, 0, 0);  // ���ñ�����ɫΪ��ɫ

	// ��������ӽǣ��۽������е��Ƶ�����
	viewer.setCameraPosition(
		overall_centroid[0], overall_centroid[1], overall_centroid[2] + 10,
		overall_centroid[0], overall_centroid[1], overall_centroid[2],
		0, -1, 0  // �۲�Ŀ��㡢�ӽǺ��Ϸ���
	);

	// ��ʾ���Ƹ���
	std::stringstream cloud_count_text;
	cloud_count_text << "Total Points: " << total_points;
	viewer.addText(cloud_count_text.str(), 10, 10, 1.0, 1.0, 1.0, "cloud_count_text");

	// ��ʾ��̬�ı�
	int y_offset = 30;
	for (const auto& text : dynamic_text) {
		viewer.addText(text.toStdString(), 10, y_offset, 1.0, 1.0, 1.0, "dynamic_text_" + std::to_string(y_offset));
		y_offset += 20;  // ÿ���ı�����20����
	}

	// ������ʾֱ���رմ���
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

// =====================================================��־
#define _SILENCE_CXX17_BYTE_DEPRECATION_WARNING
#include <QDebug>
#include <QFile>
#include <QString>
#include <sstream>
#include <filesystem>
#include <QCoreApplication>
#include <QMetaObject>
#include <QDateTime>
class LogHelper
{
public:
	inline LogHelper(const char* file, int line)
		: file_(file), line_(line) {}

	template <typename T>
	LogHelper& operator<<(const T& value)
	{
		stream_ << value;
		return *this;
	}

	// �ػ����� QString ����
	inline LogHelper& operator<<(const QString& value)
	{
		stream_ << value.toStdString();
		return *this;
	}

	// �ػ����� std::string ����
	inline LogHelper& operator<<(const std::string& value)
	{
		stream_ << value;
		return *this;
	}

	// �ػ����� const char* ����
	inline LogHelper& operator<<(const char* value)
	{
		stream_ << value;
		return *this;
	}

	// �ػ����� int ����
	inline LogHelper& operator<<(int value)
	{
		stream_ << value;
		return *this;
	}

	// �ػ����� double ����
	inline LogHelper& operator<<(double value)
	{
		stream_ << value;
		return *this;
	}

	inline ~LogHelper()
	{
#ifdef DEBUG
		QString string = QString() + "Log:" + stream_.str().c_str() + ", file:" +
			QString::fromStdString(std::filesystem::path(file_).filename().string())
			+ ", line:" + QString::fromStdString(std::to_string(line_));

		//GApp->dispToConsole(string);
		//QCoreApplication::processEvents();
		//GApp->refreshAll();

		logToFile(string);

#endif // DEBUG
	}

private:
	std::ostringstream stream_;
	const char* file_;
	int line_;


	// �ļ�·���������ѡ���Լ��趨·����
	const QString logFilePath = "logfile.txt";

	// �����־���ļ�
	inline void logToFile(const QString& logMessage)
	{
		QFile file(logFilePath);
		if (file.open(QIODevice::Append | QIODevice::Text)) {
			QTextStream out(&file);
			out << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") << " - " << logMessage << endl;
			file.close();
		}
	}
};

// ��ʽ�궨��
#define UILog LogHelper(__FILE__, __LINE__)


// ===================================================== ��ʱ��
#include <QElapsedTimer>
#include <ccMainAppInterface.h>
#include <QCoreApplication>
class Timer
{
	QString moduleNames;
	QElapsedTimer timer;
	qint64 totalTime = 0;
public:
	inline Timer(const char* name) : moduleNames(QString("[%1]").arg(name)) { timer.start(); }
	~Timer() = default;

	inline void restart(ccMainAppInterface* app, const char* description)
	{
		totalTime += timer.elapsed();
		if (app)
		{
			app->dispToConsole(QString("%1 %2: %3 ms.").arg(moduleNames).arg(description).arg(timer.restart()));
			QCoreApplication::processEvents();
			app->refreshAll();
		}
	}

	inline void elapsed(ccMainAppInterface* app, const char* description)
	{
		if (app)
		{
			app->dispToConsole(QString("%1 %2: %3 s.").arg(moduleNames).arg(description).arg((timer.elapsed()+ totalTime) / 1000.0, 0, 'f', 1));
			QCoreApplication::processEvents();
			app->refreshAll();
		}
	}
};

// ===================================================== ������
#include <QProgressDialog>
class ProgressDialog
{
private:
	QProgressDialog progressDialog;

public:
	inline ProgressDialog(const char* title, const char* text, int max)
	{
		progressDialog.setWindowTitle(title),
		progressDialog.setLabelText(text);
		progressDialog.setRange(0, max);
		progressDialog.show();
	}

	~ProgressDialog() = default;

	inline bool setValue(int pos)
	{
		progressDialog.setValue(pos);
		QCoreApplication::processEvents();

		if (progressDialog.wasCanceled())
		{
			progressDialog.close();
			QCoreApplication::processEvents();
			return false;
		}
		return true;
	}
};

// ======================================================= ���ô���
#include <QDialog>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QFormLayout>
#include <QVariant>
#include <QMap>
#include <QDebug>
#include <QIntValidator>
#include <QDoubleValidator>

class SettingsDialog : public QDialog {
	Q_OBJECT

public:
	enum class ComponentType {
		String,
		StringList,
		Int,
		Double,
		Float
	};

	inline explicit SettingsDialog(QWidget* parent = nullptr)
		: QDialog(parent) {
		setWindowTitle("Settings");

		// ���ò���
		layout = new QFormLayout(this);

		// ȷ�ϰ�ť
		QPushButton* okButton = new QPushButton("OK", this);
		layout->addRow(okButton);
		connect(okButton, &QPushButton::clicked, this, &SettingsDialog::onOkClicked);
	}

	// ���ô��ڱ���
	inline void setWindowTitle(const char* title) {
		QDialog::setWindowTitle(title);  // ���ø�������ñ��⺯��
	}

	// ���������ı�
	inline void setDescription(const char* description) {
		QLabel* descriptionLabel = new QLabel(description, this);
		descriptionLabel->setWordWrap(true);  // �����ı��Զ�����
		layout->addRow(descriptionLabel);  // ��������ǩ��ӵ�������
	}

	// ע��ؼ�������QLineEdit��QComboBox�ȣ�
	template<typename T>
	inline void registerComponent(const QString& label, const QString& name, const T& defaultValue) {
		QWidget* widget = nullptr;
		ComponentType type;

		if constexpr (std::is_same<T, QString>::value) {
			widget = new QLineEdit(defaultValue, this);
			type = ComponentType::String;
		}
		else if constexpr (std::is_same<T, QStringList>::value) {
			widget = new QComboBox(this);
			for (const auto& item : defaultValue) {
				static_cast<QComboBox*>(widget)->addItem(item);
			}
			type = ComponentType::StringList;
		}
		else if constexpr (std::is_same<T, int>::value) {
			QLineEdit* lineEdit = new QLineEdit(QString::number(defaultValue), this);
			QIntValidator* validator = new QIntValidator(this);
			lineEdit->setValidator(validator);
			widget = lineEdit;
			type = ComponentType::Int;
		}
		else if constexpr (std::is_same<T, double>::value || std::is_same<T, float>::value) {
			QLineEdit* lineEdit = new QLineEdit(QString::number(defaultValue), this);
			QDoubleValidator* validator = new QDoubleValidator(this);
			lineEdit->setValidator(validator);
			widget = lineEdit;
			type = ComponentType::Double;
		}

		if (widget) {
			layout->addRow(new QLabel(label, this), widget);
			components[name] = widget;  // ����ؼ���������
			types[name] = type;  // ����ؼ�������
			defaultValues[name] = QVariant::fromValue(defaultValue); // ����Ĭ��ֵ
		}
	}

	// ��ȡ�û�����Ĳ���
	inline QMap<QString, QVariant> getParameters() const {
		QMap<QString, QVariant> result;
		for (auto it = components.begin(); it != components.end(); ++it) {
			const auto& component = it.value();
			const ComponentType type = types[it.key()];

			if (auto lineEdit = dynamic_cast<QLineEdit*>(component)) {
				QString text = lineEdit->text();
				const QVariant& defaultValue = defaultValues[it.key()];  // ��ȡ�ؼ���Ĭ��ֵ

				if (text.isEmpty()) {
					result[it.key()] = defaultValue.isNull() ? QVariant() : defaultValue;  // ���Ϊ�գ�����Ĭ��ֵ
				}
				else {
					switch (type) {
					case ComponentType::Int: {
						bool ok;
						result[it.key()] = text.toInt(&ok);
						break;
					}
					case ComponentType::Double: {
						bool ok;
						result[it.key()] = text.toDouble(&ok);
						break;
					}
					case ComponentType::String:
					case ComponentType::StringList: {
						result[it.key()] = text;
						break;
					}
					}
				}
			}
		}
		return result;
	}

private slots:
	inline void onOkClicked() {
		accept(); // ȷ�Ϻ�رմ���
	}

private:
	QFormLayout* layout;
	QMap<QString, QWidget*> components;  // �洢�ؼ��������Ƶ�ӳ��
	QMap<QString, QVariant> defaultValues; // �洢Ĭ��ֵ
	QMap<QString, ComponentType> types;  // �洢�ؼ�������
};
