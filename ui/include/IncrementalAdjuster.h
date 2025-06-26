#pragma once
#pragma execution_character_set("utf-8")

#include <QObject>
#include <QKeyEvent>
#include <vector>
#include <map>
#include <memory>

#include <ccPointCloud.h>
#include <ccHObject.h>

// 前向声明
class ccGLWindowInterface;

// ==================== 数据结构 ====================

// 选择模式枚举
enum class SelectionMode {
    NONE,
    ELEVATION,    // 高程选择
    INTENSITY,    // 强度选择
    DENSITY       // 密度选择
};

// ==================== 主类定义 ====================

class IncrementalAdjuster : public QObject {
    Q_OBJECT

public:
    explicit IncrementalAdjuster(QObject* parent = nullptr);
    ~IncrementalAdjuster();

    // ==================== 基础设置 ====================
    void setCurrentPointCloud(ccPointCloud* cloud);
    void setGLWindow(ccGLWindowInterface* window);
    inline void setSearchRadius(float radius) { searchRadius = radius; }
    void setSelectionMode(SelectionMode mode);

    // ==================== 事件处理 ====================
    bool handleKeyEvent(QKeyEvent* event);
    void handlePointPicked(ccHObject* select_cloud, unsigned idx);

signals:
    void modeChanged(SelectionMode mode);
    void selectionConfirmed(ccPointCloud* mergedCloud);
    void updateBin();

private:
    // 核心对象
    ccPointCloud* currentCloud;
    ccGLWindowInterface* glWindow;
    SelectionMode currentMode;
    
    // 显示控制
    ccHObject* binContainer;

    // 分层管理
	std::map<int, ccPointCloud*> binClouds;
	int currentBinId;
	int binRange;// 例如正负5，显示上下5个bin
    float binMinValue;
    float binMaxValue;
    float binSize;
    void createBins(); // 分割，根据不同模式，确定每个bin的上下阈值
    void updateBinVisibility(); // binRange改变时，改变点云可见性
    int getBinIdFromValue(float value); // 根据值映射到对应的binId
    

	// 调整
	void adjustBinUp();
	void adjustBinDown();
	void confirmSelection();
	void cancelSelection();
	ccPointCloud* mergeSelectedBins(); // 结束后，合并可见的bin，
    
    // 邻域搜索
	float searchRadius;
    void calculateAverageAndBinId(const CCVector3& clickedPoint);
    float getValueAtPoint(unsigned pointIndex);

	// 密度计算
	std::vector<unsigned> densityArray;
	bool densityCalculated;
	void calculateDensityArray();
	void clearDensityArray();

	// 键盘处理
	bool handleArrowUp();
	bool handleArrowDown();
	bool handleEnter();
	bool handleEscape();
}; 
