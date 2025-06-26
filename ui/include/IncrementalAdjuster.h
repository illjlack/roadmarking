#pragma once
#pragma execution_character_set("utf-8")

#include <QObject>
#include <QKeyEvent>
#include <vector>
#include <memory>

#include <ccPointCloud.h>
#include <ccHObject.h>

// 前向声明
class ccGLWindowInterface;

// 分层数据结构
struct BinLayer {
    float lowerBound;
    float upperBound;
    ccPointCloud* filteredCloud;
    int pointCount;
    bool isSelected;  // 是否被选中显示
    
    BinLayer(float lower, float upper) 
        : lowerBound(lower), upperBound(upper), filteredCloud(nullptr), pointCount(0), isSelected(false) {}
};

// 选择模式枚举
enum class SelectionMode {
    NONE,
    ELEVATION,    // 高程选择 (Ctrl+Z)
    INTENSITY,    // 强度选择 (Ctrl+I)
    DENSITY       // 密度选择 (Ctrl+D)
};

class IncrementalAdjuster : public QObject {
    Q_OBJECT

public:
    explicit IncrementalAdjuster(QObject* parent = nullptr);
    ~IncrementalAdjuster();

    // 设置当前点云
    void setCurrentPointCloud(ccPointCloud* cloud);
    
    // 设置GL窗口
    void setGLWindow(ccGLWindowInterface* window);
    
    // 设置搜索半径
    void setSearchRadius(float radius) { searchRadius = radius; }
    
    // 获取搜索半径
    float getSearchRadius() const { return searchRadius; }
    
    // 处理键盘事件
    bool handleKeyEvent(QKeyEvent* event);
    
    // 处理点云拾取事件
    void handlePointPicked(unsigned pointIndex, const CCVector3& point);
    
    // 设置选择模式
    void setSelectionMode(SelectionMode mode);
    
    // 获取当前选择模式
    SelectionMode getCurrentMode() const { return currentMode; }
    
    // 切换到下一个/上一个点云
    void nextPointCloud();
    void previousPointCloud();
    void nextSubPointCloud();
    void previousParentPointCloud();
    
    // 增量调整阈值
    void adjustThresholdUp();
    void adjustThresholdDown();
    
    // 重新计算分层
    void recalculateBins();
    
    // 确认选择并合并点云
    void confirmSelection();
    
    // 取消选择
    void cancelSelection();
    
    // 获取当前选中的点云
    ccPointCloud* getSelectedCloud() const;

signals:
    void modeChanged(SelectionMode mode);
    void thresholdChanged(float lower, float upper);
    void pointCloudChanged(ccPointCloud* cloud);
    void selectionConfirmed(ccPointCloud* mergedCloud);

private:
    ccPointCloud* currentCloud;
    ccGLWindowInterface* glWindow;
    SelectionMode currentMode;
    
    // 分层数据
    std::vector<BinLayer> bins;
    int currentBinIndex;
    float binRange; // 每个bin的范围，默认5
    
    // 当前阈值
    float currentLowerThreshold;
    float currentUpperThreshold;
    
    // 点云列表管理
    std::vector<ccPointCloud*> pointCloudList;
    int currentCloudIndex;
    
    // 分层显示管理
    ccHObject* binContainer;  // 包含所有bin点云的容器
    bool isInSelectionMode;   // 是否处于选择模式
    float searchRadius;       // 邻域搜索半径
    
    // 私有方法
    void createBins();
    void filterCloudByThreshold(float lower, float upper);
    void applyThresholdToCloud();
    
    // 分层显示相关方法
    void displayBins();
    void hideBins();
    void updateBinVisibility();
    void selectBinRange(int startBin, int endBin);
    ccPointCloud* mergeSelectedBins();
    
    // 邻域搜索相关方法
    void calculateNeighborhoodAverage(unsigned pointIndex, const CCVector3& clickedPoint);
    float getValueAtPoint(unsigned pointIndex);
    void selectBinByValue(float value);
    void visualizeNeighborhood(const CCVector3& center, float radius, const std::vector<unsigned>& neighborIndices);
    
    // 键盘快捷键处理
    bool handleCtrlZ(); // 高程选择
    bool handleCtrlI(); // 强度选择
    bool handleCtrlD(); // 密度选择
    bool handleTab();   // 下一个点云
    bool handleShiftTab(); // 上一个点云
    bool handleAltTab(); // 子点云
    bool handleAltShiftTab(); // 父点云
    bool handleArrowUp(); // 上箭头
    bool handleArrowDown(); // 下箭头
    bool handleEnter(); // 确认选择
    bool handleEscape(); // 取消选择
}; 
