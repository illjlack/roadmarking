#pragma once
#pragma execution_character_set("utf-8")

// #define DEBUG

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

// ======================================================= 通用

/// <summary>
/// 可移动共享所有权(自动管理)的智能指针
/// </summary>
/// <typeparam name="T">数据类型</typeparam>
template <typename T>
class MovableSharedPtr : public std::shared_ptr<T> {
private:
	std::shared_ptr<std::atomic<bool>> m_is_move;  // 所有权转移标志

public:
	// 默认构造，支持原始指针，自动管理删除器
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

	// nullptr 拷贝构造函数
	MovableSharedPtr(std::nullptr_t)
		: std::shared_ptr<T>(nullptr),
		m_is_move(std::make_shared<std::atomic<bool>>(false))
	{}

	// 拷贝构造
	MovableSharedPtr(const MovableSharedPtr& other)
		: std::shared_ptr<T>(other),
		m_is_move(other.m_is_move)
	{}

	// 赋值操作
	MovableSharedPtr& operator=(const MovableSharedPtr& other) {
		if (this != &other) {
			std::shared_ptr<T>::operator=(other);
			m_is_move = other.m_is_move;
		}
		return *this;
	}

	/// <summary>
	/// 释放所有权
	/// 将原始共享指针内部转移到一个新的指针指向，此标志表示是否释放原始所有权
	/// </summary>
	/// <returns>普通指针</returns>
	T* release() {
		*m_is_move = true;  // 标记为所有权转移
		return this->get();
	}

	/// <summary>
	/// 判断共享指针是否已交出所有权
	/// </summary>
	/// <returns>是否已交出所有权</returns>
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

template <typename... PointClouds>
void visualizePointClouds(
	const QString& title,
	const std::vector<QString>& dynamic_text = {},  // 动态文本
	PointClouds&&... clouds  // 折叠表达式用于接收多个参数
)
{
	pcl::visualization::PCLVisualizer viewer(title.toStdString());

	std::vector<std::string> cloud_ids;
	Eigen::Vector4f overall_centroid(0.0f, 0.0f, 0.0f, 0.0f);
	int total_points = 0;

	size_t cloud_index = 0;
	([&] {
		// 为每个点云设置不同的颜色
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(
			std::forward<PointClouds>(clouds),
			255 - (cloud_index * 50),
			0,
			cloud_index * 50
		);

		std::stringstream id;
		id << "cloud_" << cloud_index;
		cloud_ids.push_back(id.str());

		// 将点云添加到可视化
		viewer.addPointCloud(std::forward<PointClouds>(clouds), color, cloud_ids[cloud_index]);

		// 计算每个点云的质心
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*std::forward<PointClouds>(clouds), centroid);
		overall_centroid += centroid;  // 累积
		total_points += clouds->size();

		++cloud_index;
		}(), ...);  // 使用折叠表达式展开参数

		// 计算所有点云的平均质心
	overall_centroid /= cloud_index;

	// 添加坐标系
	viewer.addCoordinateSystem(1.0);
	viewer.setBackgroundColor(0, 0, 0);  // 设置背景色为黑色

	// 设置相机视角，聚焦在所有点云的质心
	viewer.setCameraPosition(
		overall_centroid[0], overall_centroid[1], overall_centroid[2] + 10,
		overall_centroid[0], overall_centroid[1], overall_centroid[2],
		0, -1, 0  // 观察目标点、视角和上方向
	);

	// 显示点云个数
	std::stringstream cloud_count_text;
	cloud_count_text << "Total Points: " << total_points;
	viewer.addText(cloud_count_text.str(), 10, 10, 1.0, 1.0, 1.0, "cloud_count_text");

	// 显示动态文本
	int y_offset = 30;
	for (const auto& text : dynamic_text) {
		viewer.addText(text.toStdString(), 10, y_offset, 1.0, 1.0, 1.0, "dynamic_text_" + std::to_string(y_offset));
		y_offset += 20;  // 每个文本间隔20像素
	}

	// 循环显示直到关闭窗口
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}


inline void visualizePointCloudWithNormals(
	const QString& title,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, // 点云数据
	const pcl::PointCloud<pcl::Normal>::Ptr& normals,  // 对应的法向量
	const std::vector<QString>& dynamic_text = {}      // 动态文本
)
{
	pcl::visualization::PCLVisualizer viewer(title.toStdString());

	std::string cloud_id = "cloud_0"; // 只有一个点云
	std::string normals_id = "normals_0";

	// 设置点云的颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 255, 0, 0);

	// 添加点云到可视化
	viewer.addPointCloud(cloud, color, cloud_id);

	// 添加法向量
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.05, normals_id);

	// 计算点云的质心
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	// 添加坐标系
	viewer.addCoordinateSystem(1.0);
	viewer.setBackgroundColor(0, 0, 0);  // 设置背景色为黑色

	// 设置相机视角，聚焦在点云的质心
	viewer.setCameraPosition(
		centroid[0], centroid[1], centroid[2] + 10,
		centroid[0], centroid[1], centroid[2],
		0, -1, 0  // 观察目标点、视角和上方向
	);

	// 显示点云个数
	std::stringstream cloud_count_text;
	cloud_count_text << "Total Points: " << cloud->size();
	viewer.addText(cloud_count_text.str(), 10, 10, 1.0, 1.0, 1.0, "cloud_count_text");

	// 显示动态文本
	int y_offset = 30;
	for (const auto& text : dynamic_text) {
		viewer.addText(text.toStdString(), 10, y_offset, 1.0, 1.0, 1.0, "dynamic_text_" + std::to_string(y_offset));
		y_offset += 20;  // 每个文本间隔20像素
	}

	// 循环显示直到关闭窗口
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

// ===================================================== 日志
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

	// 转换为 QString 类型
	inline LogHelper& operator<<(const QString& value)
	{
		stream_ << value.toStdString();
		return *this;
	}

	// 转换为 std::string 类型
	inline LogHelper& operator<<(const std::string& value)
	{
		stream_ << value;
		return *this;
	}

	// 转换为 const char* 类型
	inline LogHelper& operator<<(const char* value)
	{
		stream_ << value;
		return *this;
	}

	// 转换为 int 类型
	inline LogHelper& operator<<(int value)
	{
		stream_ << value;
		return *this;
	}

	// 转换为 double 类型
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


	// 文件路径选择，可以设置路径
	const QString logFilePath = "logfile.txt";

	// 写入日志文件
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

// 格式化宏定义
#define UILog LogHelper(__FILE__, __LINE__)


// ===================================================== 计时
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

// ===================================================== 进度条
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

// ======================================================= 设置对话框
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
#include <QCheckBox>
class SettingsDialog : public QDialog {
	Q_OBJECT

public:
	enum class ComponentType {
		String,
		StringList,
		Int,
		Double,
		Float,
		Bool
	};

	inline explicit SettingsDialog(QWidget* parent = nullptr)
		: QDialog(parent) {
		setWindowTitle("Settings");

		// 添加部件
		layout = new QFormLayout(this);

		// 确定按钮
		QPushButton* okButton = new QPushButton("OK", this);
		layout->addRow(okButton);
		connect(okButton, &QPushButton::clicked, this, &SettingsDialog::onOkClicked);
	}

	// 设置对话框标题
	inline void setWindowTitle(const char* title) {
		QDialog::setWindowTitle(title);  // 调用父类的方法
	}

	// 设置描述文本
	inline void setDescription(const char* description) {
		QLabel* descriptionLabel = new QLabel(description, this);
		descriptionLabel->setWordWrap(true);  // 自动换行
		layout->addRow(descriptionLabel);  // 添加描述标签到布局
	}

	// 注意关键部件，如QLineEdit和QComboBox等，
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
		else if constexpr (std::is_same<T, bool>::value) {
			// For boolean values, use a QCheckBox
			QCheckBox* checkBox = new QCheckBox(this);
			checkBox->setChecked(defaultValue);  // Set default checked state
			widget = checkBox;
			type = ComponentType::Bool;
		}


		if (widget) {
			layout->addRow(new QLabel(label, this), widget);
			components[name] = widget;  // 存储关键部件和它们的映射
			types[name] = type;  // 存储关键部件类型
			defaultValues[name] = QVariant::fromValue(defaultValue); // 存储默认值
		}
	}

	// 获取用户输入的参数
	inline QMap<QString, QVariant> getParameters() const {
		QMap<QString, QVariant> result;
		for (auto it = components.begin(); it != components.end(); ++it) {
			const auto& component = it.value();
			const ComponentType type = types[it.key()];

			if (auto lineEdit = dynamic_cast<QLineEdit*>(component)) {
				QString text = lineEdit->text();
				const QVariant& defaultValue = defaultValues[it.key()];  // 获取关键部件默认值

				if (text.isEmpty()) {
					result[it.key()] = defaultValue.isNull() ? QVariant() : defaultValue;  // 如果为空，返回默认值
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
			else if (auto checkBox = dynamic_cast<QCheckBox*>(component)) {
				// For boolean values, get the checked state of the checkbox
				result[it.key()] = checkBox->isChecked();
			}
		}
		return result;
	}

private slots:
	inline void onOkClicked() {
		accept(); // 确定关闭对话框
	}

private:
	QFormLayout* layout;
	QMap<QString, QWidget*> components;  // 存储关键部件和它们的映射
	QMap<QString, QVariant> defaultValues; // 存储默认值
	QMap<QString, ComponentType> types;  // 存储关键部件类型
};
