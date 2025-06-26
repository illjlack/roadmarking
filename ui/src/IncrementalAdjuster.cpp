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

using namespace roadmarking;

// 之前准备一个个的查询密度，但是其实某一个八叉树的层就表示每个点的密度

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



//// 获取点在八叉树中的单元格位置
//Tuple3i getPointCellPosition(ccOctree* octree, const CCVector3& point)
//{
//    Tuple3i cellPos(0, 0, 0);
//    
//    if (!octree) {
//        return cellPos;
//    }
//    
//    // 调用八叉树的getTheCellPosWhichIncludesThePoint方法
//    octree->getTheCellPosWhichIncludesThePoint(&point, cellPos);
//    
//    return cellPos;
//}
//
//// 根据单元格位置获取莫顿码
//CCCoreLib::DgmOctree::CellCode getCellCodeFromPosition(ccOctree* octree, const Tuple3i& cellPos, unsigned char level)
//{
//    if (!octree) {
//        return 0;
//    }
//    
//    // 莫顿码编码：将3D坐标交错排列成1D编码
//    CCCoreLib::DgmOctree::CellCode cellCode = 0;
//    
//    for (int i = 0; i < level; ++i) {
//        // 从最低位开始，逐位提取x、y、z的对应位
//        int xBit = (cellPos.x >> i) & 1;
//        int yBit = (cellPos.y >> i) & 1;
//        int zBit = (cellPos.z >> i) & 1;
//        
//        // 将位交错排列：z位最高，y位中间，x位最低
//        cellCode |= (static_cast<CCCoreLib::DgmOctree::CellCode>(zBit) << (3 * i + 2));
//        cellCode |= (static_cast<CCCoreLib::DgmOctree::CellCode>(yBit) << (3 * i + 1));
//        cellCode |= (static_cast<CCCoreLib::DgmOctree::CellCode>(xBit) << (3 * i));
//    }
//    
//    return cellCode;
//}
//
//// 组合函数：直接从点获取单元格编码
//CCCoreLib::DgmOctree::CellCode getPointCellCodeFromPosition(ccOctree* octree, const CCVector3& point, unsigned char level)
//{
//    if (!octree) {
//        return 0;
//    }
//    
//    // 先获取点在八叉树中的单元格位置
//    Tuple3i cellPos = getPointCellPosition(octree, point);
//    
//    // 然后根据位置获取莫顿码
//    return getCellCodeFromPosition(octree, cellPos, level);
//}
//
//
//
//// 八叉树的辅助函数，查询点所在的code，使用动态level
//CCCoreLib::DgmOctree::CellCode getPointCellCode(ccOctree* octree, const CCVector3& point, float targetCellSize)
//{
//    if (!octree) {
//        return 0;
//    }
//    
//    // 根据目标大小获取合适的level
//    unsigned char level = 10; // 1024分之一应该差不多够小了
//    
//    // 获取位偏移量
//    unsigned char bitShift = CCCoreLib::DgmOctree::GET_BIT_SHIFT(level);
//    
//    // 使用八叉树获取点所在的单元格代码
//    CCCoreLib::DgmOctree::CellCode cellCode = octree->getCellCode(point, level);
//    
//    return cellCode;
//}
//
//
//
//// 八叉树的辅助函数，查询一块中点的数量，用来代替密度
//unsigned getCellPointCount(ccOctree* octree, const CCCoreLib::DgmOctree::CellCode& code, unsigned char level)
//{
//    if (!octree) {
//        return 0;
//    }
//    
//    // 获取位偏移量
//    unsigned char bitShift = CCCoreLib::DgmOctree::GET_BIT_SHIFT(level);
//    
//    // 右移位偏移量得到当前层级的代码
//    CCCoreLib::DgmOctree::CellCode currentCode = code >> bitShift;
//    
//    // 获取当前代码对应的单元格索引
//    unsigned currentCellIndex = octree->getCellIndex(currentCode, bitShift);
//    
//    // 计算下一个代码（当前代码+1）
//    CCCoreLib::DgmOctree::CellCode nextCode = currentCode + 1;
//    
//    // 获取下一个代码对应的单元格索引
//    unsigned nextCellIndex = octree->getCellIndex(nextCode, bitShift);
//    
//    // 返回两个索引之间的差值，即该单元格中点的数量
//    return nextCellIndex - currentCellIndex;
//}
//



// ==================== 构造函数和析构函数 ====================

IncrementalAdjuster::IncrementalAdjuster(QObject* parent)
    : QObject(parent)
    , currentCloud(nullptr)
    , glWindow(nullptr)
    , currentMode(SelectionMode::NONE)
    , binContainer(nullptr)
    , currentBinId(0)
    , binRange(5)
    , searchRadius(1.0f)
    , densityCalculated(false)
    , binMinValue(0.0f)
    , binMaxValue(1.0f)
    , binSize(0.1f)
{
    // 创建bin容器
    binContainer = new ccHObject("增量调整器");
}

IncrementalAdjuster::~IncrementalAdjuster()
{
    // 清理密度数组
    clearDensityArray();
    
    // 清理所有bin点云
    for (auto& pair : binClouds) {
        if (pair.second) {
            delete pair.second;
        }
    }
    binClouds.clear();
    
    // 清理bin容器
    if (binContainer) {
        delete binContainer;
    }
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
        
        currentMode = mode;
        emit modeChanged(mode);
    }
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
    
    // 获取点击的点坐标
    const CCVector3* point = cloud->getPoint(idx);
    if (!point) {
        return;
    }
    
    // 创建bins并分割点云
    createBins();
    
    // 计算邻域平均值并更新bin选择
	calculateAverageAndBinId(*point);
}

// ==================== 分层管理 ====================

void IncrementalAdjuster::createBins()
{
    if (!currentCloud) {
        return;
    }
    
    // 根据模式确定bin的数量和阈值
    int numBins = 200;
    binMinValue = 0.0f;
    binMaxValue = 1.0f;
    
    switch (currentMode) {
    case SelectionMode::ELEVATION:
    {
        const ccBBox& bbox = currentCloud->getOwnBB();
        binMinValue = static_cast<float>(bbox.minCorner().z);
        binMaxValue = static_cast<float>(bbox.maxCorner().z);
    }
    break;
    
    case SelectionMode::INTENSITY:
    {
        int intensityIdx = currentCloud->getScalarFieldIndexByName("intensity");
        if (intensityIdx >= 0) {
            auto* intensityField = currentCloud->getScalarField(intensityIdx);
            if (intensityField) {
                binMinValue = static_cast<float>(intensityField->getMin());
                binMaxValue = static_cast<float>(intensityField->getMax());
            }
        }
    }
    break;
    
    case SelectionMode::DENSITY:
    {
        // 使用密度数组的最大最小值
        if (!densityArray.empty()) {
            auto minMax = std::minmax_element(densityArray.begin(), densityArray.end());
            binMinValue = static_cast<float>(*minMax.first);
            binMaxValue = static_cast<float>(*minMax.second);
        }
        else {
            // 如果密度数组为空，使用默认值
            binMinValue = 0.0f;
            binMaxValue = 100.0f;
        }
    }
    break;
    
    default:
        return;
    }
    
    // 计算bin大小
    binSize = (binMaxValue - binMinValue) / numBins;
    
    // 创建bins
    for (int i = 0; i < numBins; ++i) {
        ccPointCloud* binCloud = new ccPointCloud();
        binCloud->setName(QString("Bin_%1").arg(i));
        
        // 设置bin的显示属性
        binCloud->setPointSize(currentCloud->getPointSize());
        binCloud->setVisible(false); // 初始时隐藏
        
        // 添加到容器
        binContainer->addChild(binCloud);
        binClouds[i] = binCloud;
    }
    
    // 根据每个点的数据值，将点分配到对应的bins中
    for (unsigned i = 0; i < currentCloud->size(); ++i) {
        float value = getValueAtPoint(i);
        
        // 计算该点应该属于哪个bin
        int binIndex = getBinIdFromValue(value);
        
        // 将点添加到对应的bin中
        auto it = binClouds.find(binIndex);
        if (it != binClouds.end() && it->second) {
            const CCVector3* point = currentCloud->getPoint(i);
            if (point) {
                it->second->addPoint(*point);
            }
        }
    }
    
    // 根据当前点击的点设置初始的currentBinId
    // 这里需要获取点击点的值并计算对应的bin
    if (currentCloud && currentCloud->size() > 0) {
        // 假设我们有一个全局变量存储点击的点索引
        // 如果没有，可以在这里添加逻辑来获取点击点的值
        // 暂时使用第一个点作为示例
        float clickedValue = getValueAtPoint(0);
        currentBinId = getBinIdFromValue(clickedValue);
        
        // 更新bin可见性
        updateBinVisibility();
    }
}

int IncrementalAdjuster::getBinIdFromValue(float value)
{
    if (binSize <= 0.0f) {
        return 0;
    }
    
    // 计算bin索引
    int binIndex = static_cast<int>((value - binMinValue) / binSize);
    
    // 确保索引在有效范围内
    binIndex = std::max(0, std::min(binIndex, static_cast<int>(binClouds.size()) - 1));
    
    return binIndex;
}

void IncrementalAdjuster::updateBinVisibility()
{
    if (binClouds.empty()) {
        return;
    }
    
    // 隐藏所有bins
    for (auto& pair : binClouds) {
        if (pair.second) {
            pair.second->setVisible(false);
        }
    }
    
    // 显示当前bin及其周围的bins
    int startBin = std::max(0, currentBinId - binRange);
    int endBin = std::min(static_cast<int>(binClouds.size()) - 1, currentBinId + binRange);
    
    for (int i = startBin; i <= endBin; ++i) {
        auto it = binClouds.find(i);
        if (it != binClouds.end() && it->second) {
            it->second->setVisible(true);
        }
    }
    
    // 重绘窗口
    if (glWindow) {
        glWindow->redraw(true, false);
    }
}

// ==================== 调整方法 ====================

void IncrementalAdjuster::adjustBinUp()
{
    if (currentBinId < static_cast<int>(binClouds.size()) - 1) {
        currentBinId++;
        updateBinVisibility();
    }
}

void IncrementalAdjuster::adjustBinDown()
{
    if (currentBinId > 0) {
        currentBinId--;
        updateBinVisibility();
    }
}

void IncrementalAdjuster::confirmSelection()
{
    ccPointCloud* mergedCloud = mergeSelectedBins();
    if (mergedCloud) {
        emit selectionConfirmed(mergedCloud);
    }
    
    // 重置模式
    setSelectionMode(SelectionMode::NONE);
}

void IncrementalAdjuster::cancelSelection()
{
    // 重置模式
    setSelectionMode(SelectionMode::NONE);
}

ccPointCloud* IncrementalAdjuster::mergeSelectedBins()
{
    if (binClouds.empty()) {
        return nullptr;
    }
    
    // 创建合并的点云
    ccPointCloud* mergedCloud = new ccPointCloud();
    mergedCloud->setName("增量选择结果");
    
    // 收集所有可见bins中的点
    for (auto& pair : binClouds) {
        if (pair.second && pair.second->isVisible()) {
            // 将当前bin中的点添加到合并点云中
            for (unsigned i = 0; i < pair.second->size(); ++i) {
                const CCVector3* point = pair.second->getPoint(i);
                mergedCloud->addPoint(*point);
            }
        }
    }
    
    if (mergedCloud->size() == 0) {
        delete mergedCloud;
        return nullptr;
    }
    
    return mergedCloud;
}

// ==================== 邻域搜索 ====================

void IncrementalAdjuster::calculateAverageAndBinId(const CCVector3& clickedPoint)
{
    if (!currentCloud || !currentCloud->getOctree()) {
        return;
    }
    
    // 使用八叉树进行邻域搜索
    ccOctree::Shared octree = currentCloud->getOctree();
    
    // 获取合适的层级
    int level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(searchRadius);
    
    // 搜索邻域内的点
    CCCoreLib::DgmOctree::NeighboursSet neighbours;
    bool success = octree->getPointsInSphericalNeighbourhood(clickedPoint, searchRadius, neighbours, level);
    
    if (!success || neighbours.empty()) {
        ccLog::Warning("[IncrementalAdjuster] 邻域搜索失败或未找到邻域点");
        return;
    }
    
    // 计算邻域内点的平均值
    float totalValue = 0.0f;
    unsigned validCount = 0;
    
    for (const auto& neighbour : neighbours) {
        unsigned idx = neighbour.pointIndex;
        if (idx < currentCloud->size()) {
            totalValue += getValueAtPoint(idx);
            validCount++;
        }
    }
    
    if (validCount > 0) {
        float averageValue = totalValue / validCount;
        ccLog::Print(QString("[IncrementalAdjuster] 邻域平均值: %1 (基于 %2 个点)").arg(averageValue).arg(validCount));
        
        // 根据平均值选择对应的bin
        int targetBinId = getBinIdFromValue(averageValue);
        if (targetBinId != currentBinId) {
            currentBinId = targetBinId;
            updateBinVisibility();
        }
    }
}

float IncrementalAdjuster::getValueAtPoint(unsigned pointIndex)
{
    if (!currentCloud || pointIndex >= currentCloud->size()) {
        return 0.0f;
    }
    
    switch (currentMode) {
    case SelectionMode::ELEVATION:
        {
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

// ==================== 键盘处理 ====================

bool IncrementalAdjuster::handleArrowUp()
{
    if (currentMode != SelectionMode::NONE) {
        adjustBinUp();
        return true;
    }
    return false;
}

bool IncrementalAdjuster::handleArrowDown()
{
    if (currentMode != SelectionMode::NONE) {
        adjustBinDown();
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

// ==================== 密度计算 ====================

void IncrementalAdjuster::calculateDensityArray()
{
    if (!currentCloud || !currentCloud->getOctree()) {
        ccLog::Warning("[IncrementalAdjuster] 无法计算密度：点云或八叉树为空");
        return;
    }
    
    ccLog::Print("[IncrementalAdjuster] 开始计算密度数组...");
    
    // 计算密度数组
    densityArray = getPointDensityAtLevel(currentCloud, 10);
    densityCalculated = true;
    
    ccLog::Print(QString("[IncrementalAdjuster] 密度数组计算完成，共 %1 个点").arg(densityArray.size()));
}

void IncrementalAdjuster::clearDensityArray()
{
    densityArray.clear();
    densityCalculated = false;
    ccLog::Print("[IncrementalAdjuster] 密度数组已清理");
}
