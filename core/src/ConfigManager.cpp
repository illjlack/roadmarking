#include "ConfigManager.h"
#include <fstream>
#include <stdexcept>
#include <filesystem>
#include <cassert>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace roadmarking {

	void ConfigManager::initialize(const std::string& configPath) {
		m_filePath = configPath;
		loadConfig();
	}

	void ConfigManager::loadConfig() {
		try {
			// 如果配置文件不存在，则生成默认配置
			if (!std::filesystem::exists(m_filePath)) {
				std::cout << "配置文件不存在，正在生成默认配置文件..." << std::endl;
				createDefaultConfig();
				return;
			}

			// 先创建默认配置，这样所有配置项都有默认值
			createDefaultConfig();

			std::ifstream ifs(m_filePath);
			std::string line;
			std::string currentModule;
			std::string currentDescription;
			
			while (std::getline(ifs, line)) {
				// 跳过空行
				if (line.empty()) {
					continue;
				}

				// 处理注释行
				if (line[0] == '#') {
					currentDescription = line.substr(1);  // 去掉开头的 #
					// 去除前导空格
					currentDescription.erase(0, currentDescription.find_first_not_of(" \t"));
					continue;
				}

				// 解析键值对
				size_t pos = line.find('=');
				if (pos != std::string::npos) {
					std::string key = line.substr(0, pos);
					std::string value = line.substr(pos + 1);
					
					// 解析模块名和配置项名
					pos = key.find('.');
					if (pos != std::string::npos) {
						std::string moduleName = key.substr(0, pos);
						std::string itemName = key.substr(pos + 1);
						
						// 确保模块存在
						if (m_modules.find(moduleName) == m_modules.end()) {
							Module m;
							m.description = moduleName;
							m_modules[moduleName] = std::move(m);
						}
						
						// 更新配置值
						ConfigItem& item = m_modules[moduleName].items[itemName];
						item.value = value;
						// 如果配置项是新添加的，设置默认值
						if (item.default_value.empty()) {
							item.default_value = value;
						}
						// 更新描述
						if (!currentDescription.empty()) {
							item.description = currentDescription;
						}
						currentDescription.clear();
					}
				}
			}
		}
		catch (const std::exception& e) {
			std::cout << "加载配置文件失败：" << e.what() << std::endl;
			std::cout << "正在重新生成默认配置文件..." << std::endl;
			createDefaultConfig();
		}
	}

	void ConfigManager::createDefaultConfig() {
		m_modules.clear();

		{
			Module module;
			module.description = "模板匹配相关设置";
			ConfigItem item;

			item.value = std::to_string(20);
			item.default_value = std::to_string(20);
			item.description = "迭代次数";
			module.items["iter_num"] = item;

			item.value = std::to_string(0.2f);
			item.default_value = std::to_string(0.2f);
			item.description = "ICP适配度阈值";
			module.items["correct_match_fitness_thre"] = item;

			item.value = std::to_string(0.2f);
			item.default_value = std::to_string(0.2f);
			item.description = "允许的点云重叠距离";
			module.items["overlap_dis"] = item;

			item.value = std::to_string(0.60f);
			item.default_value = std::to_string(0.60f);
			item.description = "最小重叠比例";
			module.items["tolerant_min_overlap"] = item;

			item.value = std::to_string(15.0f);
			item.default_value = std::to_string(15.0f);
			item.description = "旋转角度步长";
			module.items["heading_increment"] = item;

			m_modules["template_matching"] = std::move(module);
		}

		{
			Module module;
			module.description = "点云处理相关设置";
			ConfigItem item;

			item.value = std::to_string(0.05f);
			item.default_value = std::to_string(0.05f);
			item.description = "体素大小";
			module.items["voxel_size"] = item;

			item.value = std::to_string(0.1f);
			item.default_value = std::to_string(0.1f);
			item.description = "法向量计算半径";
			module.items["normal_radius"] = item;

			item.value = std::to_string(0.3f);
			item.default_value = std::to_string(0.3f);
			item.description = "欧式聚类半径";
			module.items["euclidean_cluster_radius"] = item;

			item.value = std::to_string(10);
			item.default_value = std::to_string(10);
			item.description = "最小聚类点数";
			module.items["min_cluster_size"] = item;

			item.value = std::to_string(50);
			item.default_value = std::to_string(50);
			item.description = "统计滤波的K近邻点数";
			module.items["statistical_outlier_mean_k"] = item;

			item.value = std::to_string(1.0f);
			item.default_value = std::to_string(1.0f);
			item.description = "统计滤波的标准差倍数";
			module.items["statistical_outlier_std_dev"] = item;

			m_modules["cloud_processing"] = std::move(module);
		}

		{
			Module module;
			module.description = "地面提取相关设置";
			ConfigItem item;

			item.value = std::to_string(0.65f);
			item.default_value = std::to_string(0.65f);
			item.description = "CSF算法时间步长";
			module.items["csf_time_step"] = item;

			item.value = std::to_string(0.5f);
			item.default_value = std::to_string(0.5f);
			item.description = "CSF分类阈值";
			module.items["csf_class_threshold"] = item;

			item.value = std::to_string(1.0f);
			item.default_value = std::to_string(1.0f);
			item.description = "CSF布料分辨率";
			module.items["csf_cloth_resolution"] = item;

			item.value = std::to_string(3);
			item.default_value = std::to_string(3);
			item.description = "CSF刚度";
			module.items["csf_rigidness"] = item;

			item.value = std::to_string(500);
			item.default_value = std::to_string(500);
			item.description = "CSF迭代次数";
			module.items["csf_iterations"] = item;

			m_modules["ground_extraction"] = std::move(module);
		}

		{
			Module module;
			module.description = "道路标线提取相关设置";
			ConfigItem item;

			item.value = std::to_string(0.3f);
			item.default_value = std::to_string(0.3f);
			item.description = "搜索半径（单位：米）";
			module.items["search_radius"] = item;

			item.value = std::to_string(5.0f);
			item.default_value = std::to_string(5.0f);
			item.description = "角度阈值（单位：度）";
			module.items["angle_threshold"] = item;

			item.value = std::to_string(0.01f);
			item.default_value = std::to_string(0.01f);
			item.description = "曲率阈值";
			module.items["curvature_threshold"] = item;

			item.value = std::to_string(1.0f);
			item.default_value = std::to_string(1.0f);
			item.description = "最小条纹长度（单位：米）";
			module.items["min_stripe_length"] = item;

			item.value = std::to_string(10);
			item.default_value = std::to_string(10);
			item.description = "密度阈值";
			module.items["density_threshold"] = item;

			m_modules["road_marking_extraction"] = std::move(module);
		}

		{
			Module module;
			module.description = "点云生长相关设置";
			ConfigItem item;

			item.value = std::to_string(1.0f);
			item.default_value = std::to_string(1.0f);
			item.description = "窗口宽度（单位：米）";
			module.items["window_width"] = item;

			item.value = std::to_string(2.0f);
			item.default_value = std::to_string(2.0f);
			item.description = "生长长度（单位：米）";
			module.items["growth_length"] = item;

			item.value = std::to_string(20);
			item.default_value = std::to_string(20);
			item.description = "最小点数";
			module.items["min_points"] = item;

			item.value = std::to_string(45.0f);
			item.default_value = std::to_string(45.0f);
			item.description = "最大角度（单位：度）";
			module.items["max_angle"] = item;

			item.value = std::to_string(10);
			item.default_value = std::to_string(10);
			item.description = "最大跳跃点数";
			module.items["max_jump"] = item;

			m_modules["point_growth"] = std::move(module);
		}

		{
			Module module;
			module.description = "可视化相关设置";
			ConfigItem item;

			item.value = std::to_string(2);
			item.default_value = std::to_string(2);
			item.description = "点云显示大小";
			module.items["point_size"] = item;

			item.value = std::to_string(2);
			item.default_value = std::to_string(2);
			item.description = "线条宽度";
			module.items["line_width"] = item;

			item.value = "false";
			item.default_value = "false";
			item.description = "是否启用调试模式";
			module.items["debug_mode"] = item;

			m_modules["visualization"] = std::move(module);
		}

		// 保存默认配置到文件
		try {
			const auto path = std::filesystem::path(m_filePath);
			std::filesystem::create_directories(path.parent_path());
			std::ofstream ofs(m_filePath);
			
			// 写入配置
			for (const auto& [moduleName, module] : m_modules) {
				ofs << "################### " << module.description << "\n";
				for (const auto& [key, item] : module.items) {
					ofs << "# " << item.description << "\n";
					ofs << moduleName << "." << key << "=" << item.value << "\n";
				}
				ofs << "\n";
			}
			std::cout << "默认配置文件已生成：" << m_filePath << std::endl;
		}
		catch (const std::exception& e) {
			std::cerr << "生成默认配置文件失败：" << e.what() << std::endl;
			throw;
		}
	}

	void ConfigManager::saveConfig() {
		try {
			const auto path = std::filesystem::path(m_filePath);
			std::filesystem::create_directories(path.parent_path());
			std::ofstream ofs(m_filePath);
			
			// 写入配置
			bool isFirstModule = true;
			for (const auto& [moduleName, module] : m_modules) {
				if (!isFirstModule) {
					ofs << "\n###################\n\n";
				}
				isFirstModule = false;

				ofs << "# " << module.description << "\n";
				for (const auto& [key, item] : module.items) {
					ofs << "# " << item.description << "\n";
					ofs << moduleName << "." << key << "=" << item.value << "\n";
				}
			}
		}
		catch (const std::exception& e) {
			std::cerr << "保存配置文件失败：" << e.what() << std::endl;
			throw;
		}
	}

	std::string ConfigManager::getDescription(const std::string& module, const std::string& key) {
		auto modIt = m_modules.find(module);
		if (modIt != m_modules.end()) {
			auto itemIt = modIt->second.items.find(key);
			if (itemIt != modIt->second.items.end()) {
				return itemIt->second.description;
			}
		}
		return {};
	}

	void ConfigManager::displayAllConfigs() {
		std::cout << "\n当前配置信息：" << std::endl;
		std::cout << "================" << std::endl;
		for (const auto& [moduleName, module] : m_modules) {
			std::cout << "\n模块: " << moduleName << std::endl;
			std::cout << "描述: " << module.description << std::endl;
			std::cout << "配置项:" << std::endl;
			for (const auto& [key, item] : module.items) {
				std::cout << "  " << key << ":" << std::endl;
				std::cout << "    当前值: " << item.value << std::endl;
				std::cout << "    默认值: " << item.default_value << std::endl;
				std::cout << "    描述: " << item.description << std::endl;
			}
		}
		std::cout << "\n================" << std::endl;
	}

	void ConfigManager::reloadConfig() {
		m_modules.clear();
		loadConfig();
	}

} // namespace roadmarking
