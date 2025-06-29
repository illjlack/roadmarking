#pragma once
#pragma execution_character_set("utf-8")

#include <QObject>
#include <QKeyEvent>
#include <vector>

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
    void setSelectionMode(SelectionMode mode);

    // ==================== 事件处理 ====================
    bool handleKeyEvent(QKeyEvent* event);
    void handlePointPicked(ccHObject* select_cloud, unsigned idx);

signals:
    void modeChanged(SelectionMode mode);
    void selectionConfirmed(ccPointCloud* selectedCloud);
    void updateRange();

private:
    // 核心对象
    ccPointCloud* currentCloud;
    ccGLWindowInterface* glWindow;
    SelectionMode currentMode;

    // 阈值范围控制
    float currentValue;
    float rangeSize;
    float minValue;
    float maxValue;
    
    // 密度计算
    std::vector<unsigned> densityArray;
    bool densityCalculated;

    // ==================== 核心方法 ====================
    void initializeRange(); // 初始化阈值范围
    void updatePointVisibility(); // 根据阈值范围更新点云可见性
    float getValueAtPoint(unsigned pointIndex);
    
    // ==================== 调整方法 ====================
    void adjustValueUp(); // 增加当前值
    void adjustValueDown(); // 减少当前值
    void adjustRangeUp(); // 增加范围大小
    void adjustRangeDown(); // 减少范围大小
    void confirmSelection(); // 确认选择
    void cancelSelection(); // 取消选择
    ccPointCloud* createSelectedCloud(); // 创建选中的点云
    
    // ==================== 密度计算 ====================
    void calculateDensityArray();
    void clearDensityArray();

    // ==================== 键盘处理 ====================
    bool handleArrowUp();
    bool handleArrowDown();
    bool handleEnter();
    bool handleEscape();
}; 
