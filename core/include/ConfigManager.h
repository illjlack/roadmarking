/*
#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

class ConfigManager {
public:
	// 配置项数据结构：包含描述、当前值和默认值
	struct ConfigItem {
		std::string description;
		YAML::Node value;
		YAML::Node default_value;
	};

	// 模块数据结构：包含描述和一组配置项
	struct Module {
		std::string description;
		std::unordered_map<std::string, ConfigItem> items;
	};

	// 构造函数，接受配置文件路径
	ConfigManager(const std::string& filePath);

	// 加载配置文件
	void loadConfig();

	// 创建默认配置文件
	void createDefaultConfig();

	// 获取配置项的值，返回默认值（如果配置项不存在）
	template <typename T>
	T get(const std::string& module, const std::string& key, T defaultValue);

	// 设置配置项的值
	template <typename T>
	void set(const std::string& module, const std::string& key, const T& value);

	// 保存配置到文件
	void saveConfig();

	// 获取配置项的描述
	std::string getDescription(const std::string& module, const std::string& key);

	// 显示所有配置项（用于调试）
	void displayAllConfigs();

private:
	std::string m_filePath;             // 配置文件路径
	YAML::Node m_config;                // 配置数据
	std::unordered_map<std::string, Module> m_modules;  // 存储模块和配置项
};

*/
