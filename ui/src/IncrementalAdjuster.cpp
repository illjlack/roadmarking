#include "IncrementalAdjuster.h"
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <ccHObject.h>
#include <ccScalarField.h>
#include <ccOctree.h>
#include <ccLog.h>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>
#include <limits>
#include "PointCloudIO.h"
#include "CloudProcess.h"

using namespace roadmarking;

// 获取点云中每个点在指定层级单元格中的密度信息
std::vector<unsigned> getPointDensityAtLevel(ccPointCloud* cloud, unsigned char level)
{
    std::vector<unsigned> densityArray;
    
    if (!cloud || !cloud->getOctree()) {
        return densityArray;
    }
    
    ccOctree::Shared octree = cloud->getOctree();
    
    // 获取指定层级的所有单元格代码和索引
    CCCoreLib::DgmOctree::cellsContainer cellCodesAndIndexes;
    bool truncatedCodes = false;
    bool success = octree->getCellCodesAndIndexes(level, cellCodesAndIndexes, truncatedCodes);
    
    if (!success) {
        ccLog::Warning("[IncrementalAdjuster] Failed to get cell codes and indexes at level %d", level);
        return densityArray;
    }
    
    // 初始化密度数组，大小为点云的点数
    densityArray.resize(cloud->size(), 0);
    
    // 创建临时容器用于获取单元格中的点
    CCCoreLib::ReferenceCloud* subset = new CCCoreLib::ReferenceCloud(cloud);
    
    // 遍历每个单元格
    for (const auto& cell : cellCodesAndIndexes) {
        unsigned cellIndex = cell.theIndex;
        
        // 获取当前单元格中的所有点
        success = octree->getPointsInCellByCellIndex(subset, cellIndex, level);
        if (success && subset->size() > 0) {
            // 计算当前单元格中的点数
            unsigned pointCount = static_cast<unsigned>(subset->size());
            
            // 将该密度值赋给单元格中的所有点
            for (unsigned i = 0; i < subset->size(); ++i) {
                unsigned globalIndex = subset->getPointGlobalIndex(i);
                if (globalIndex < densityArray.size()) {
                    densityArray[globalIndex] = pointCount;
                }
            }
        }
    }
    
    // 清理临时容器
    delete subset;
    
    return densityArray;
}

// ==================== 构造函数和析构函数 ====================

IncrementalAdjuster::IncrementalAdjuster(QObject* parent)
    : QObject(parent)
    , currentCloud(nullptr)
    , glWindow(nullptr)
    , currentMode(SelectionMode::NONE)
    , currentValue(0.0f)
    , rangeSize(1.0f)
    , minValue(0.0f)
    , maxValue(1.0f)
    , densityCalculated(false)
    , originalScalarFieldIndex(-1)
    , originalSFVisible(false)
{
}

IncrementalAdjuster::~IncrementalAdjuster()
{
    // 清理密度数组
    clearDensityArray();
}

// ==================== 基础设置 ====================

void IncrementalAdjuster::setCurrentPointCloud(ccPointCloud* cloud)
{
    currentCloud = cloud;
}

void IncrementalAdjuster::setGLWindow(ccGLWindowInterface* window)
{
    glWindow = window;
}

void IncrementalAdjuster::setSelectionMode(SelectionMode mode)
{
    if (currentMode != mode) {
        // 如果从密度模式切换到其他模式，清理密度数组
        if (currentMode == SelectionMode::DENSITY && mode != SelectionMode::DENSITY) {
            clearDensityArray();
        }
        
        // 如果切换到NONE模式，恢复点云的完整显示
        if (mode == SelectionMode::NONE && currentCloud) {
            // 恢复所有点的可见性
            currentCloud->setVisible(true);
            currentCloud->unallocateVisibilityArray();
            
            // 恢复原始标量字段状态
            if (originalScalarFieldIndex >= 0) {
                currentCloud->setCurrentDisplayedScalarField(originalScalarFieldIndex);
                currentCloud->showSF(originalSFVisible);
            } else {
                // 如果没有保存的原始状态，尝试恢复强度标量字段显示（如果存在）
                int intensityIdx = PointCloudIO::get_intensity_idx(currentCloud);
                if (intensityIdx >= 0) {
                    currentCloud->setCurrentDisplayedScalarField(intensityIdx);
                    currentCloud->showSF(true);
                } else {
                    // 如果没有强度字段，隐藏标量字段显示
                    currentCloud->showSF(false);
                }
            }
            
            // 重置原始状态
            originalScalarFieldIndex = -1;
            originalSFVisible = false;
            
            // 重绘窗口
            if (glWindow) {
                glWindow->redraw(true, false);
            }
        }
        
        currentMode = mode;
        emit modeChanged(mode);
    }

    // 设置初始范围大小
    rangeSize = 5.0f;

    emit updateRange();
}

// ==================== 事件处理 ====================

bool IncrementalAdjuster::handleKeyEvent(QKeyEvent* event)
{
    if (currentMode == SelectionMode::NONE) {
        return false;
    }
    
    switch (event->key()) {
    case Qt::Key_Up:
        return handleArrowUp();
    case Qt::Key_Down:
        return handleArrowDown();
    case Qt::Key_PageUp:
        adjustRangeUp();
        return true;
    case Qt::Key_PageDown:
        adjustRangeDown();
        return true;
    case Qt::Key_Return:
    case Qt::Key_Enter:
        return handleEnter();
    case Qt::Key_Escape:
        return handleEscape();
    default:
        return false;
    }
}

void IncrementalAdjuster::handlePointPicked(ccHObject* select_cloud, unsigned idx)
{
    if (currentMode == SelectionMode::NONE) {
        return;
    }
    
    // 验证输入参数
    if (!select_cloud || !dynamic_cast<ccPointCloud*>(select_cloud)) {
        ccLog::Error(QString("[IncrementalAdjuster] 请选择有效的点云对象"));
        return;
    }
    
    ccPointCloud* cloud = static_cast<ccPointCloud*>(select_cloud);
    if (idx >= cloud->size()) {
        ccLog::Error(QString("[IncrementalAdjuster] 请选择有效点"));
        return;
    }
    
    // 如果当前点云与传入的点云不同，则更新当前点云
    if (currentCloud != cloud) {
        setCurrentPointCloud(cloud);
        // 如果当前模式是密度模式，需要重新计算密度数组
        if (currentMode == SelectionMode::DENSITY) {
            clearDensityArray();
            calculateDensityArray();
        }
    }
    
    // 保存原始状态（在第一次进入增量调整模式时）
    if (originalScalarFieldIndex == -1) {
        originalScalarFieldIndex = currentCloud->getCurrentDisplayedScalarFieldIndex();
        originalSFVisible = currentCloud->sfShown();
    }
    
    // 初始化阈值范围
    initializeRange();
    
    // 设置当前值为选中点的值
    currentValue = getValueAtPoint(idx);
    
    // 更新点云可见性
    updatePointVisibility();
}

// ==================== 核心方法 ====================

void IncrementalAdjuster::initializeRange()
{
    if (!currentCloud) {
        return;
    }
    
    // 根据模式确定阈值范围
    minValue = 0.0f;
    maxValue = 1.0f;
    
    switch (currentMode) {
    case SelectionMode::ELEVATION:
    {
        // 计算点云的实际高程范围
        float minZ = std::numeric_limits<float>::max();
        float maxZ = std::numeric_limits<float>::lowest();
        
        for (unsigned i = 0; i < currentCloud->size(); ++i) {
            const CCVector3* point = currentCloud->getPoint(i);
            float z = static_cast<float>(point->z);
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
        }
        
        minValue = minZ;
        maxValue = maxZ;
        
        // 确保高程标量字段存在并正确设置
        PointCloudIO::apply_height_as_scalar(currentCloud);
        
        // 验证高程标量字段是否正确设置
        int heightIdx = PointCloudIO::get_height_idx(currentCloud);
        if (heightIdx >= 0) {
            auto* heightField = currentCloud->getScalarField(heightIdx);
            if (heightField) {
                // 使用标量字段的实际范围
                minValue = static_cast<float>(heightField->getMin());
                maxValue = static_cast<float>(heightField->getMax());
            }
        }

		PointCloudIO::apply_height_as_scalar(currentCloud);
        // 确保高程标量字段可见
        currentCloud->showSF(true);
    }
    break;
    
    case SelectionMode::INTENSITY:
    {
        int intensityIdx = PointCloudIO::get_intensity_idx(currentCloud);
        if (intensityIdx >= 0) {
            auto* intensityField = currentCloud->getScalarField(intensityIdx);
            if (intensityField) {
                minValue = static_cast<float>(intensityField->getMin());
                maxValue = static_cast<float>(intensityField->getMax());
            }
        }
        
        // 设置强度为显示标量
        PointCloudIO::apply_intensity(currentCloud);
    }
    break;
    
    case SelectionMode::DENSITY:
    {
        // 计算密度数组
        if (!densityCalculated) {
            calculateDensityArray();
        }
        
        // 使用密度数组的最大最小值
        if (!densityArray.empty()) {
            auto minMax = std::minmax_element(densityArray.begin(), densityArray.end());
            minValue = static_cast<float>(*minMax.first);
            maxValue = static_cast<float>(*minMax.second);
            
            // 将密度保存为标量字段
            PointCloudIO::save_density_as_scalar(currentCloud, densityArray);
            PointCloudIO::apply_density_as_scalar(currentCloud);
        }
        else {
            // 如果密度数组为空，使用默认值
            minValue = 0.0f;
            maxValue = 100.0f;
        }
    }
    break;
    
    default:
        return;
    }
    
    // 计算范围大小（初始为总范围的10%）
    rangeSize = (maxValue - minValue) * 0.1f;

    // 确保点云可见
    currentCloud->setVisible(true);
}

void IncrementalAdjuster::updatePointVisibility()
{
    if (!currentCloud) {
        return;
    }

    // 计算当前范围
    float startValue = currentValue - rangeSize * 0.5f;
    float endValue = currentValue + rangeSize * 0.5f;
    if (startValue < minValue) startValue = minValue;
    if (endValue > maxValue) endValue = maxValue;

    // 隐藏不在当前范围内的点
    currentCloud->hidePointsByScalarValue(startValue, endValue);
    currentCloud->setVisible(true);

    // 重绘窗口
    if (glWindow) {
        glWindow->redraw(true, false);
    }

    emit updateRange();
}

float IncrementalAdjuster::getValueAtPoint(unsigned pointIndex)
{
    if (!currentCloud || pointIndex >= currentCloud->size()) {
        return 0.0f;
    }
    
    switch (currentMode) {
    case SelectionMode::ELEVATION:
        {
            // 优先从高程标量字段获取值
            int heightIdx = PointCloudIO::get_height_idx(currentCloud);
            if (heightIdx >= 0) {
                auto heightField = currentCloud->getScalarField(heightIdx);
                if (heightField) {
                    return static_cast<float>(heightField->getValue(pointIndex));
                }
            }
            
            // 如果没有高程标量字段，直接从点坐标获取Z值
            const CCVector3* point = currentCloud->getPoint(pointIndex);
            return static_cast<float>(point->z);
        }
        
    case SelectionMode::INTENSITY:
        {
            int intensityIdx = PointCloudIO::get_intensity_idx(currentCloud);
            if (intensityIdx >= 0) {
                auto intensityField = currentCloud->getScalarField(intensityIdx);
                if (intensityField) {
                    return static_cast<float>(intensityField->getValue(pointIndex));
                }
            }
            return 0.0f;
        }
        
    case SelectionMode::DENSITY:
        {
            // 返回该点的密度值
            if (pointIndex < densityArray.size()) {
                return static_cast<float>(densityArray[pointIndex]);
            }
            return 0.0f;
        }
        
    default:
        return 0.0f;
    }
}

// ==================== 调整方法 ====================

void IncrementalAdjuster::adjustValueUp()
{
    if (currentMode == SelectionMode::NONE) {
        return;
    }
    
    // 增加当前值
    float step = (maxValue - minValue) * 0.01f; // 1%的步长
    currentValue += step;
    if (currentValue > maxValue) {
        currentValue = maxValue;
    }
    
    updatePointVisibility();
}

void IncrementalAdjuster::adjustValueDown()
{
    if (currentMode == SelectionMode::NONE) {
        return;
    }
    
    // 减少当前值
    float step = (maxValue - minValue) * 0.01f; // 1%的步长
    currentValue -= step;
    if (currentValue < minValue) {
        currentValue = minValue;
    }
    
    updatePointVisibility();
}

void IncrementalAdjuster::adjustRangeUp()
{
    if (currentMode == SelectionMode::NONE) {
        return;
    }
    
    // 增加范围大小
    rangeSize += (maxValue - minValue) * 0.01f;
    updatePointVisibility();
}

void IncrementalAdjuster::adjustRangeDown()
{
    if (currentMode == SelectionMode::NONE) {
        return;
    }
    
    // 减少范围大小
    float step = (maxValue - minValue) * 0.01f;
    rangeSize -= step;
    if (rangeSize < step) {
        rangeSize = step; // 最小范围
    }
    
    updatePointVisibility();
}

void IncrementalAdjuster::confirmSelection()
{
    ccPointCloud* selectedCloud = createSelectedCloud();
    if (selectedCloud) {
        emit selectionConfirmed(selectedCloud);
    }
    
    // 重置模式
    setSelectionMode(SelectionMode::NONE);
}

void IncrementalAdjuster::cancelSelection()
{
    // 重置模式
    setSelectionMode(SelectionMode::NONE);
}

ccPointCloud* IncrementalAdjuster::createSelectedCloud()
{
    if (!currentCloud) {
        return nullptr;
    }
    
    // 创建选中的点云
    ccPointCloud* selectedCloud = new ccPointCloud();
    
    // 根据模式生成有意义的名称
    QString cloudName;
    switch (currentMode) {
    case SelectionMode::ELEVATION:
        cloudName = QString("高程选择_%1±%2").arg(currentValue, 0, 'f', 2).arg(rangeSize * 0.5f, 0, 'f', 2);
        break;
    case SelectionMode::INTENSITY:
        cloudName = QString("强度选择_%1±%2").arg(currentValue, 0, 'f', 2).arg(rangeSize * 0.5f, 0, 'f', 2);
        break;
    case SelectionMode::DENSITY:
        cloudName = QString("密度选择_%1±%2").arg(currentValue, 0, 'f', 0).arg(rangeSize * 0.5f, 0, 'f', 0);
        break;
    default:
        cloudName = "选择结果";
        break;
    }
    
    selectedCloud->setName(cloudName);
    
    // 计算当前范围
    float startValue = currentValue - rangeSize * 0.5f;
    float endValue = currentValue + rangeSize * 0.5f;
    if (startValue < minValue) startValue = minValue;
    if (endValue > maxValue) endValue = maxValue;
    
    // 收集范围内的点
    for (unsigned i = 0; i < currentCloud->size(); ++i) {
        float value = getValueAtPoint(i);
        if (value >= startValue && value <= endValue) {
            const CCVector3* point = currentCloud->getPoint(i);
            selectedCloud->addPoint(*point);
        }
    }
    
    if (selectedCloud->size() == 0) {
        delete selectedCloud;
        return nullptr;
    }
    
    // 复制标量字段
    if (currentCloud->hasScalarFields()) {
        // 复制强度标量字段
        int intensityIdx = PointCloudIO::get_intensity_idx(currentCloud);
        if (intensityIdx >= 0) {
            selectedCloud->addScalarField("intensity");
            selectedCloud->setCurrentInScalarField(0);
            
            CCCoreLib::ScalarField* sourceSF = currentCloud->getScalarField(intensityIdx);
            CCCoreLib::ScalarField* targetSF = selectedCloud->getScalarField(0);
            
            if (sourceSF && targetSF) {
                unsigned targetIndex = 0;
                for (unsigned i = 0; i < currentCloud->size(); ++i) {
                    float value = getValueAtPoint(i);
                    if (value >= startValue && value <= endValue) {
                        ScalarType scalarValue = sourceSF->getValue(i);
                        targetSF->addElement(scalarValue);
                        targetIndex++;
                    }
                }
            }
            
            // 设置标量字段显示
            PointCloudIO::apply_intensity(selectedCloud);
        }
    }
    
    return selectedCloud;
}

// ==================== 密度计算 ====================

void IncrementalAdjuster::calculateDensityArray()
{
    if (!currentCloud || !currentCloud->getOctree()) {
        ccLog::Warning("[IncrementalAdjuster] 无法计算密度：点云或八叉树为空");
        return;
    }
    
    ccLog::Print("[IncrementalAdjuster] 开始计算密度数组...");
    
    // 计算密度数组
    densityArray = getPointDensityAtLevel(currentCloud, 11);
    densityCalculated = true;
    
    ccLog::Print(QString("[IncrementalAdjuster] 密度数组计算完成，共 %1 个点").arg(densityArray.size()));
}

void IncrementalAdjuster::clearDensityArray()
{
    densityArray.clear();
    densityCalculated = false;
    ccLog::Print("[IncrementalAdjuster] 密度数组已清理");
}

// ==================== 键盘处理 ====================

bool IncrementalAdjuster::handleArrowUp()
{
    if (currentMode != SelectionMode::NONE) {
        adjustValueUp();
        return true;
    }
    return false;
}

bool IncrementalAdjuster::handleArrowDown()
{
    if (currentMode != SelectionMode::NONE) {
        adjustValueDown();
        return true;
    }
    return false;
}

bool IncrementalAdjuster::handleEnter()
{
    if (currentMode != SelectionMode::NONE) {
        confirmSelection();
        return true;
    }
    return false;
}

bool IncrementalAdjuster::handleEscape()
{
    if (currentMode != SelectionMode::NONE) {
        cancelSelection();
        return true;
    }
    return false;
}
