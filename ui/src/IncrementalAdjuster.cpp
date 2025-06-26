#include "IncrementalAdjuster.h"
#include "CloudFilterDlg.h"
#include <QApplication>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>
#include "PointCloudIO.h"
#include <ccGLWindowInterface.h>
#include <ccColorTypes.h>
#include <ccLog.h>
#include <QTimer>

using namespace roadmarking;

IncrementalAdjuster::IncrementalAdjuster(QObject* parent)
    : QObject(parent)
    , currentCloud(nullptr)
    , glWindow(nullptr)
    , currentMode(SelectionMode::NONE)
    , currentBinIndex(0)
    , binRange(5.0f)
    , currentLowerThreshold(0.0f)
    , currentUpperThreshold(0.0f)
    , currentCloudIndex(0)
    , binContainer(nullptr)
    , isInSelectionMode(false)
    , searchRadius(0.5f)
{
}

IncrementalAdjuster::~IncrementalAdjuster()
{
    // 清理分层数据
    for (auto& bin : bins) {
        if (bin.filteredCloud) {
            delete bin.filteredCloud;
        }
    }
    bins.clear();
    
    // 清理bin容器
    if (binContainer) {
        delete binContainer;
        binContainer = nullptr;
    }
}

void IncrementalAdjuster::setCurrentPointCloud(ccPointCloud* cloud)
{
    currentCloud = cloud;
    
    // 如果点云不在列表中，添加它
    if (cloud) {
        bool found = false;
        for (auto* existingCloud : pointCloudList) {
            if (existingCloud == cloud) {
                found = true;
                break;
            }
        }
        if (!found) {
            pointCloudList.push_back(cloud);
            currentCloudIndex = pointCloudList.size() - 1;
        }
        
        if (isInSelectionMode) {
            recalculateBins();
            displayBins();
        }
    }
}

void IncrementalAdjuster::setGLWindow(ccGLWindowInterface* window)
{
    glWindow = window;
}

bool IncrementalAdjuster::handleKeyEvent(QKeyEvent* event)
{
    if (!event) return false;

    // 检查Ctrl组合键
    if (event->modifiers() & Qt::ControlModifier) {
        switch (event->key()) {
        case Qt::Key_Z:
            return handleCtrlZ();
        case Qt::Key_I:
            return handleCtrlI();
        case Qt::Key_D:
            return handleCtrlD();
        }
    }
    
    // 检查Alt组合键
    if (event->modifiers() & Qt::AltModifier) {
        if (event->key() == Qt::Key_Tab) {
            if (event->modifiers() & Qt::ShiftModifier) {
                return handleAltShiftTab();
            } else {
                return handleAltTab();
            }
        }
    }
    
    // 检查Tab键
    if (event->key() == Qt::Key_Tab) {
        if (event->modifiers() & Qt::ShiftModifier) {
            return handleShiftTab();
        } else {
            return handleTab();
        }
    }
    
    // 检查方向键
    if (event->key() == Qt::Key_Up) {
        return handleArrowUp();
    } else if (event->key() == Qt::Key_Down) {
        return handleArrowDown();
    }
    
    // 检查Enter键（确认选择）
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        return handleEnter();
    }
    
    // 检查Escape键（取消选择）
    if (event->key() == Qt::Key_Escape) {
        return handleEscape();
    }
    
    return false;
}

void IncrementalAdjuster::handlePointPicked(unsigned pointIndex, const CCVector3& point)
{
    if (!currentCloud || currentMode == SelectionMode::NONE || !isInSelectionMode) {
        return;
    }
    
    // 直接使用拾取的点来计算邻域平均值
    calculateNeighborhoodAverage(pointIndex, point);
}

void IncrementalAdjuster::setSelectionMode(SelectionMode mode)
{
    if (currentMode != mode) {
        // 如果之前处于选择模式，先清理
        if (isInSelectionMode) {
            hideBins();
        }
        
        currentMode = mode;
        emit modeChanged(mode);
        
        // 如果选择了有效模式，开始分层显示
        if (mode != SelectionMode::NONE && currentCloud) {
            isInSelectionMode = true;
            
            // 输出调试信息
            QString modeName;
            switch (mode) {
            case SelectionMode::ELEVATION:
                modeName = "高程";
                break;
            case SelectionMode::INTENSITY:
                modeName = "强度";
                break;
            case SelectionMode::DENSITY:
                modeName = "密度";
                break;
            default:
                modeName = "未知";
                break;
            }
            ccLog::Print(QString("进入%1分层选择模式，使用方向键调整选择范围，Enter确认，Esc取消").arg(modeName));
            
            recalculateBins();
            displayBins();
        } else {
            isInSelectionMode = false;
        }
    }
}

void IncrementalAdjuster::nextPointCloud()
{
    if (pointCloudList.empty()) return;
    
    currentCloudIndex = (currentCloudIndex + 1) % pointCloudList.size();
    currentCloud = pointCloudList[currentCloudIndex];
    emit pointCloudChanged(currentCloud);
    
    if (currentCloud) {
        recalculateBins();
    }
}

void IncrementalAdjuster::previousPointCloud()
{
    if (pointCloudList.empty()) return;
    
    currentCloudIndex = (currentCloudIndex - 1 + pointCloudList.size()) % pointCloudList.size();
    currentCloud = pointCloudList[currentCloudIndex];
    emit pointCloudChanged(currentCloud);
    
    if (currentCloud) {
        recalculateBins();
    }
}

void IncrementalAdjuster::nextSubPointCloud()
{
    // 这里需要根据实际的点云层次结构来实现
    // 暂时实现为下一个点云
    nextPointCloud();
}

void IncrementalAdjuster::previousParentPointCloud()
{
    // 这里需要根据实际的点云层次结构来实现
    // 暂时实现为上一个点云
    previousPointCloud();
}

void IncrementalAdjuster::adjustThresholdUp()
{
    if (currentMode == SelectionMode::NONE || !isInSelectionMode) return;
    
    // 找到当前选中的bin范围
    int startBin = -1, endBin = -1;
    for (size_t i = 0; i < bins.size(); ++i) {
        if (bins[i].isSelected) {
            if (startBin == -1) startBin = i;
            endBin = i;
        }
    }
    
    // 向上扩展选择范围
    if (endBin < static_cast<int>(bins.size()) - 1) {
        selectBinRange(startBin, endBin + 1);
    }
}

void IncrementalAdjuster::adjustThresholdDown()
{
    if (currentMode == SelectionMode::NONE || !isInSelectionMode) return;
    
    // 找到当前选中的bin范围
    int startBin = -1, endBin = -1;
    for (size_t i = 0; i < bins.size(); ++i) {
        if (bins[i].isSelected) {
            if (startBin == -1) startBin = i;
            endBin = i;
        }
    }
    
    // 向下扩展选择范围
    if (startBin > 0) {
        selectBinRange(startBin - 1, endBin);
    }
}

void IncrementalAdjuster::recalculateBins()
{
    if (!currentCloud) return;
    
    // 清理现有分层
    for (auto& bin : bins) {
        if (bin.filteredCloud) {
            delete bin.filteredCloud;
        }
    }
    bins.clear();
    
    // 创建新的分层
    createBins();
}

void IncrementalAdjuster::createBins()
{
    if (!currentCloud) return;
    
    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::lowest();
    int sfIdx = -1; // 将sfIdx声明移到switch外部
    
    // 根据当前模式获取相应的值范围
    switch (currentMode) {
    case SelectionMode::ELEVATION:
        // 获取Z坐标范围
        for (unsigned i = 0; i < currentCloud->size(); ++i) {
            const CCVector3* point = currentCloud->getPoint(i);
            minValue = std::min(minValue, point->z);
            maxValue = std::max(maxValue, point->z);
        }
        break;
        
    case SelectionMode::INTENSITY:
        // 获取强度范围
        sfIdx = PointCloudIO::get_intensity_idx(currentCloud);
        if (sfIdx >= 0) {
            auto sf = currentCloud->getScalarField(sfIdx);
            if (sf) {
                for (unsigned i = 0; i < currentCloud->size(); ++i) {
                    float intensity = sf->getValue(i);
                    minValue = std::min(minValue, intensity);
                    maxValue = std::max(maxValue, intensity);
                }
            }
        }
        break;
        
    case SelectionMode::DENSITY:
        // 计算密度（基于点间距的简单密度估计）
        // 这里使用一个简化的方法：基于点的Z坐标变化来估计密度
        minValue = 0.0f;
        maxValue = 100.0f; // 假设密度范围
        
        // 如果点云有足够的点，可以计算一个更准确的密度范围
        if (currentCloud->size() > 100) {
            // 计算点云的标准差作为密度变化的参考
            float meanZ = 0.0f;
            for (unsigned i = 0; i < currentCloud->size(); ++i) {
                const CCVector3* point = currentCloud->getPoint(i);
                meanZ += point->z;
            }
            meanZ /= currentCloud->size();
            
            float variance = 0.0f;
            for (unsigned i = 0; i < currentCloud->size(); ++i) {
                const CCVector3* point = currentCloud->getPoint(i);
                float diff = point->z - meanZ;
                variance += diff * diff;
            }
            variance /= currentCloud->size();
            
            float stdDev = sqrt(variance);
            maxValue = stdDev * 10.0f; // 使用标准差的10倍作为最大密度值
        }
        break;
        
    default:
        return;
    }
    
    // 创建分层
    int numBins = static_cast<int>((maxValue - minValue) / binRange) + 1;
    
    // 限制bin的数量，避免创建过多
    const int maxBins = 20;
    if (numBins > maxBins) {
        binRange = (maxValue - minValue) / maxBins;
        numBins = maxBins;
        ccLog::Print(QString("调整bin范围至 %1，创建 %2 个分层").arg(binRange, 0, 'f', 2).arg(numBins));
    }
    
    bins.reserve(numBins);
    
    for (int i = 0; i < numBins; ++i) {
        float lower = minValue + i * binRange;
        float upper = minValue + (i + 1) * binRange;
        bins.emplace_back(lower, upper);
        
        // 为每个bin创建过滤点云
        filterCloudByThreshold(lower, upper);
    }
    
    currentBinIndex = 0;
}

void IncrementalAdjuster::filterCloudByThreshold(float lower, float upper)
{
    if (!currentCloud) return;
	int sfIdx = -1;
    // 找到对应的bin
    int binIndex = -1;
    for (size_t i = 0; i < bins.size(); ++i) {
        if (std::abs(bins[i].lowerBound - lower) < 1e-6 && std::abs(bins[i].upperBound - upper) < 1e-6) {
            binIndex = i;
            break;
        }
    }
    
    if (binIndex == -1) return;
    
    // 创建过滤后的点云
    ccPointCloud* filteredCloud = new ccPointCloud();
    filteredCloud->reserve(currentCloud->size() / 10); // 预留一些空间
    
    switch (currentMode) {
    case SelectionMode::ELEVATION:
        // 按高程过滤
        for (unsigned i = 0; i < currentCloud->size(); ++i) {
            const CCVector3* point = currentCloud->getPoint(i);
            if (point->z >= lower && point->z <= upper) {
                filteredCloud->addPoint(*point);
            }
        }
        break;
        
    case SelectionMode::INTENSITY:
        // 按强度过滤
        sfIdx = PointCloudIO::get_intensity_idx(currentCloud);
        if (sfIdx >= 0) {
            auto sf = currentCloud->getScalarField(sfIdx);
            if (sf) {
                for (unsigned i = 0; i < currentCloud->size(); ++i) {
                    float intensity = sf->getValue(i);
                    if (intensity >= lower && intensity <= upper) {
                        const CCVector3* point = currentCloud->getPoint(i);
                        filteredCloud->addPoint(*point);
                    }
                }
            }
        }
        break;
        
    case SelectionMode::DENSITY:
        // 按密度过滤（简化实现）
        // 这里需要根据实际的密度计算方法来实现
        break;
        
    default:
        break;
    }
    
    // 更新当前bin的数据
    if (bins[binIndex].filteredCloud) {
        delete bins[binIndex].filteredCloud;
    }
    bins[binIndex].filteredCloud = filteredCloud;
    bins[binIndex].pointCount = static_cast<int>(filteredCloud->size());
}

void IncrementalAdjuster::applyThresholdToCloud()
{
    if (!currentCloud) return;
    
    // 应用阈值到点云
    filterCloudByThreshold(currentLowerThreshold, currentUpperThreshold);
    
    // 发送阈值变化信号
    emit thresholdChanged(currentLowerThreshold, currentUpperThreshold);
}

// 键盘快捷键处理实现
bool IncrementalAdjuster::handleCtrlZ()
{
    setSelectionMode(SelectionMode::ELEVATION);
    return true;
}

bool IncrementalAdjuster::handleCtrlI()
{
    setSelectionMode(SelectionMode::INTENSITY);
    return true;
}

bool IncrementalAdjuster::handleCtrlD()
{
    setSelectionMode(SelectionMode::DENSITY);
    return true;
}

bool IncrementalAdjuster::handleTab()
{
    nextPointCloud();
    return true;
}

bool IncrementalAdjuster::handleShiftTab()
{
    previousPointCloud();
    return true;
}

bool IncrementalAdjuster::handleAltTab()
{
    nextSubPointCloud();
    return true;
}

bool IncrementalAdjuster::handleAltShiftTab()
{
    previousParentPointCloud();
    return true;
}

bool IncrementalAdjuster::handleArrowUp()
{
    adjustThresholdUp();
    return true;
}

bool IncrementalAdjuster::handleArrowDown()
{
    adjustThresholdDown();
    return true;
}

bool IncrementalAdjuster::handleEnter()
{
    confirmSelection();
    return true;
}

bool IncrementalAdjuster::handleEscape()
{
    cancelSelection();
    return true;
}

// 分层显示相关方法实现
void IncrementalAdjuster::displayBins()
{
    if (!glWindow || !currentCloud) return;
    
    // 创建bin容器
    if (!binContainer) {
        binContainer = new ccHObject();
        binContainer->setName("分层点云");
        glWindow->addToOwnDB(binContainer);
    }
    
    // 为每个bin创建点云并添加到GL窗口
    for (size_t i = 0; i < bins.size(); ++i) {
        if (bins[i].filteredCloud && bins[i].filteredCloud->size() > 0) {
            // 设置点云名称和颜色
            QString modeName;
            switch (currentMode) {
            case SelectionMode::ELEVATION:
                modeName = "高程";
                break;
            case SelectionMode::INTENSITY:
                modeName = "强度";
                break;
            case SelectionMode::DENSITY:
                modeName = "密度";
                break;
            default:
                modeName = "未知";
                break;
            }
            
            QString binName = QString("%1_Bin_%2_[%3-%4]")
                .arg(modeName)
                .arg(i)
                .arg(bins[i].lowerBound, 0, 'f', 2)
                .arg(bins[i].upperBound, 0, 'f', 2);
            bins[i].filteredCloud->setName(binName);
            
            // 根据bin索引设置不同颜色，使用彩虹色方案
            ccColor::Rgb color;
            float hue = (i * 360.0f) / bins.size();
            if (hue < 60.0f) {
                color.r = 255;
                color.g = static_cast<unsigned char>(255 * hue / 60.0f);
                color.b = 0;
            } else if (hue < 120.0f) {
                color.r = static_cast<unsigned char>(255 * (120.0f - hue) / 60.0f);
                color.g = 255;
                color.b = 0;
            } else if (hue < 180.0f) {
                color.r = 0;
                color.g = 255;
                color.b = static_cast<unsigned char>(255 * (hue - 120.0f) / 60.0f);
            } else if (hue < 240.0f) {
                color.r = 0;
                color.g = static_cast<unsigned char>(255 * (240.0f - hue) / 60.0f);
                color.b = 255;
            } else if (hue < 300.0f) {
                color.r = static_cast<unsigned char>(255 * (hue - 240.0f) / 60.0f);
                color.g = 0;
                color.b = 255;
            } else {
                color.r = 255;
                color.g = 0;
                color.b = static_cast<unsigned char>(255 * (360.0f - hue) / 60.0f);
            }
            
            bins[i].filteredCloud->setColor(color);
            bins[i].filteredCloud->showColors(true);
            bins[i].filteredCloud->setVisible(true);
            
            // 添加到容器
            binContainer->addChild(bins[i].filteredCloud);
        }
    }
    
    // 设置原始点云为半透明，保持可见以便点击
    if (currentCloud) {
        currentCloud->setVisible(true);
        currentCloud->setEnabled(true);
    }
    
    // 默认选择第一个bin
    if (!bins.empty()) {
        selectBinRange(0, 0);
    }
    
    ccLog::Print(QString("创建了 %1 个分层点云，每个分层显示不同颜色").arg(bins.size()));
    
    // 显示每个bin的统计信息
    for (size_t i = 0; i < bins.size(); ++i) {
        if (bins[i].filteredCloud && bins[i].filteredCloud->size() > 0) {
            ccLog::Print(QString("  Bin %1: [%2, %3] - %4 个点")
                .arg(i)
                .arg(bins[i].lowerBound, 0, 'f', 2)
                .arg(bins[i].upperBound, 0, 'f', 2)
                .arg(bins[i].filteredCloud->size()));
        }
    }
    
    glWindow->redraw();
}

void IncrementalAdjuster::hideBins()
{
    if (!glWindow) return;
    
    // 显示原始点云并恢复透明度
    if (currentCloud) {
        currentCloud->setVisible(true);
        currentCloud->setEnabled(true);
    }
    
    // 移除bin容器及其所有子对象
    if (binContainer) {
        glWindow->removeFromOwnDB(binContainer);
        delete binContainer;
        binContainer = nullptr;
    }
    
    glWindow->redraw();
}

void IncrementalAdjuster::updateBinVisibility()
{
    if (!binContainer) return;
    
    for (size_t i = 0; i < bins.size(); ++i) {
        if (bins[i].filteredCloud) {
            bins[i].filteredCloud->setVisible(bins[i].isSelected);
            
            // 为选中的bin添加高亮效果
            if (bins[i].isSelected) {
                // 增加点的大小以突出显示
                bins[i].filteredCloud->setPointSize(bins[i].filteredCloud->getPointSize() * 1.5f);
                // 设置为高亮状态
                bins[i].filteredCloud->setSelected(true);
            } else {
                // 恢复正常的点大小
                bins[i].filteredCloud->setPointSize(bins[i].filteredCloud->getPointSize() / 1.5f);
                // 取消高亮状态
                bins[i].filteredCloud->setSelected(false);
            }
        }
    }
    
    if (glWindow) {
        glWindow->redraw();
    }
}

void IncrementalAdjuster::selectBinRange(int startBin, int endBin)
{
    // 清除之前的选择
    for (auto& bin : bins) {
        bin.isSelected = false;
    }
    
    // 选择指定范围
    for (int i = startBin; i <= endBin && i < static_cast<int>(bins.size()); ++i) {
        bins[i].isSelected = true;
    }
    
    // 更新显示
    updateBinVisibility();
    
    // 更新当前阈值
    if (startBin < static_cast<int>(bins.size()) && endBin < static_cast<int>(bins.size())) {
        currentLowerThreshold = bins[startBin].lowerBound;
        currentUpperThreshold = bins[endBin].upperBound;
        
        // 输出当前选择范围
        QString modeName;
        switch (currentMode) {
        case SelectionMode::ELEVATION:
            modeName = "高程";
            break;
        case SelectionMode::INTENSITY:
            modeName = "强度";
            break;
        case SelectionMode::DENSITY:
            modeName = "密度";
            break;
        default:
            modeName = "未知";
            break;
        }
        
        int selectedCount = endBin - startBin + 1;
        int totalPoints = 0;
        for (int i = startBin; i <= endBin && i < static_cast<int>(bins.size()); ++i) {
            totalPoints += bins[i].pointCount;
        }
        
        ccLog::Print(QString("当前%1选择范围: [%2, %3]，选中%4个分层，共%5个点").arg(modeName)
            .arg(currentLowerThreshold, 0, 'f', 2)
            .arg(currentUpperThreshold, 0, 'f', 2)
            .arg(selectedCount)
            .arg(totalPoints));
        
        emit thresholdChanged(currentLowerThreshold, currentUpperThreshold);
    }
}

ccPointCloud* IncrementalAdjuster::getSelectedCloud() const
{
    if (!isInSelectionMode) return nullptr;
    
    // 创建合并后的点云
    ccPointCloud* mergedCloud = new ccPointCloud();
    if (currentCloud) {
        mergedCloud->setName(currentCloud->getName() + "_selected");
    }
    
    // 统计总点数
    int totalPoints = 0;
    for (const auto& bin : bins) {
        if (bin.isSelected && bin.filteredCloud) {
            totalPoints += bin.filteredCloud->size();
        }
    }
    
    if (totalPoints == 0) {
        delete mergedCloud;
        return nullptr;
    }
    
    mergedCloud->reserve(totalPoints);
    
    // 合并选中的点云
    for (const auto& bin : bins) {
        if (bin.isSelected && bin.filteredCloud) {
            for (unsigned i = 0; i < bin.filteredCloud->size(); ++i) {
                const CCVector3* point = bin.filteredCloud->getPoint(i);
                mergedCloud->addPoint(*point);
            }
        }
    }
    
    return mergedCloud;
}

void IncrementalAdjuster::confirmSelection()
{
    if (!isInSelectionMode) return;
    
    // 合并选中的点云
    ccPointCloud* mergedCloud = getSelectedCloud();
    if (mergedCloud) {
        // 设置合并后点云的颜色和名称
        mergedCloud->setColor(ccColor::green);
        mergedCloud->showColors(true);
        ccLog::Print(QString("确认选择，合并后的点云包含 %1 个点").arg(mergedCloud->size()));
        emit selectionConfirmed(mergedCloud);
    } else {
        ccLog::Warning("没有选中任何点云");
    }
    
    // 退出选择模式
    cancelSelection();
}

void IncrementalAdjuster::cancelSelection()
{
    if (!isInSelectionMode) return;
    
    ccLog::Print("取消选择，退出分层模式");
    
    // 隐藏分层显示
    hideBins();
    
    // 重置状态
    isInSelectionMode = false;
    currentMode = SelectionMode::NONE;
    
    // 清理分层数据
    for (auto& bin : bins) {
        if (bin.filteredCloud) {
            delete bin.filteredCloud;
            bin.filteredCloud = nullptr;
        }
        bin.isSelected = false;
    }
    bins.clear();
    
    // 恢复正常的拾取模式
    if (glWindow) {
        glWindow->setPickingMode(ccGLWindowInterface::ENTITY_PICKING);
    }
    
    emit modeChanged(SelectionMode::NONE);
}

// 邻域搜索相关方法实现
float IncrementalAdjuster::getValueAtPoint(unsigned pointIndex)
{
    if (!currentCloud || pointIndex >= currentCloud->size()) {
        return std::numeric_limits<float>::max();
    }
    
    int sfIdx = -1; // 将sfIdx声明移到switch外部
    
    switch (currentMode) {
    case SelectionMode::ELEVATION:
        // 返回Z坐标
        return currentCloud->getPoint(pointIndex)->z;
        
    case SelectionMode::INTENSITY:
        // 返回强度值
        sfIdx = PointCloudIO::get_intensity_idx(currentCloud);
        if (sfIdx >= 0) {
            auto sf = currentCloud->getScalarField(sfIdx);
            if (sf) {
                return sf->getValue(pointIndex);
            }
        }
        return std::numeric_limits<float>::max();
        
    case SelectionMode::DENSITY:
        // 返回密度值（使用Z坐标的绝对值作为代理）
        return abs(currentCloud->getPoint(pointIndex)->z);
        
    default:
        return std::numeric_limits<float>::max();
    }
}

void IncrementalAdjuster::selectBinByValue(float value)
{
    if (bins.empty()) return;
    
    // 找到包含该值的bin
    for (size_t i = 0; i < bins.size(); ++i) {
        if (value >= bins[i].lowerBound && value <= bins[i].upperBound) {
            selectBinRange(i, i);
            ccLog::Print(QString("根据值 %1 选择了Bin %2").arg(value, 0, 'f', 2).arg(i));
            return;
        }
    }
    
    ccLog::Warning(QString("值 %1 不在任何bin范围内").arg(value, 0, 'f', 2));
}

void IncrementalAdjuster::calculateNeighborhoodAverage(unsigned pointIndex, const CCVector3& clickedPoint)
{
    if (!currentCloud || pointIndex >= currentCloud->size()) {
        ccLog::Warning("无效的点索引");
        return;
    }
    
    // 收集邻域内的点
    std::vector<unsigned> neighborIndices;
    std::vector<float> neighborValues;
    
    for (unsigned i = 0; i < currentCloud->size(); ++i) {
        const CCVector3* point = currentCloud->getPoint(i);
        float distance = (*point - clickedPoint).norm();
        
        if (distance <= searchRadius) {
            neighborIndices.push_back(i);
            float value = getValueAtPoint(i);
            if (value != std::numeric_limits<float>::max()) {
                neighborValues.push_back(value);
            }
        }
    }
    
    if (neighborValues.empty()) {
        ccLog::Warning("在搜索半径内没有找到有效的邻域点");
        return;
    }
    
    // 计算邻域平均值
    float sum = 0.0f;
    for (float value : neighborValues) {
        sum += value;
    }
    float averageValue = sum / neighborValues.size();
    
    // 输出邻域信息
    QString modeName;
    switch (currentMode) {
    case SelectionMode::ELEVATION:
        modeName = "高程";
        break;
    case SelectionMode::INTENSITY:
        modeName = "强度";
        break;
    case SelectionMode::DENSITY:
        modeName = "密度";
        break;
    default:
        modeName = "未知";
        break;
    }
    
    ccLog::Print(QString("点击位置%1值: %2").arg(modeName).arg(getValueAtPoint(pointIndex), 0, 'f', 2));
    ccLog::Print(QString("邻域内找到 %1 个点，平均%2值: %3").arg(neighborValues.size()).arg(modeName).arg(averageValue, 0, 'f', 2));
    
    // 可视化邻域搜索结果
    visualizeNeighborhood(clickedPoint, searchRadius, neighborIndices);
    
    // 根据平均值选择对应的bin
    selectBinByValue(averageValue);
}

void IncrementalAdjuster::visualizeNeighborhood(const CCVector3& center, float radius, const std::vector<unsigned>& neighborIndices)
{
    if (!glWindow || neighborIndices.empty()) return;
    
    // 创建邻域点云用于可视化
    ccPointCloud* neighborhoodCloud = new ccPointCloud();
    neighborhoodCloud->setName("邻域点云");
    neighborhoodCloud->reserve(neighborIndices.size());
    
    // 添加邻域内的点
    for (unsigned index : neighborIndices) {
        if (index < currentCloud->size()) {
            const CCVector3* point = currentCloud->getPoint(index);
            neighborhoodCloud->addPoint(*point);
        }
    }
    
    // 设置邻域点云的颜色和显示属性
    neighborhoodCloud->setColor(ccColor::yellow);
    neighborhoodCloud->showColors(true);
    neighborhoodCloud->setPointSize(neighborhoodCloud->getPointSize() * 2.0f); // 增大点的大小
    
    // 添加到GL窗口
    glWindow->addToOwnDB(neighborhoodCloud);
    
    // 3秒后自动移除邻域可视化
    QTimer::singleShot(3000, [this, neighborhoodCloud]() {
        if (glWindow && neighborhoodCloud) {
            glWindow->removeFromOwnDB(neighborhoodCloud);
            delete neighborhoodCloud;
            glWindow->redraw();
        }
    });
    
    glWindow->redraw();
} 
