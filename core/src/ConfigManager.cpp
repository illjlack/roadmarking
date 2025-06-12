/*
#include "ConfigManager.h"
#include <iostream>
#include <fstream>
#include <stdexcept>

ConfigManager::ConfigManager(const std::string& filePath)
	: m_filePath(filePath) {
	// 尝试加载配置文件
	loadConfig();
}

void ConfigManager::loadConfig() {
	try {
		m_config = YAML::LoadFile(m_filePath);
		// 解析配置文件并填充模块
		for (auto& module : m_config)
		{
			Module m;
			m.description = module.second["description"].as<std::string>();
			for (auto& item : module.second)
			{
				if (item.first != "description")
				{
					ConfigItem configItem;
					configItem.value = item.second["value"];
					configItem.default_value = item.second["default"];
					configItem.description = item.second["description"].as<std::string>();
					m.items[item.first.as<std::string>()] = configItem;
				}
			}
			m_modules[module.first.as<std::string>()] = m;
		}
	}
	catch (const std::exception& e)
	{
		createDefaultConfig();  // 配置文件不存在时，创建默认配置文件
	}
}

void ConfigManager::createDefaultConfig() {
	// 模块：window
	m_config["window"]["description"] = "Window related settings";
	m_config["window"]["width"]["value"] = 800;
	m_config["window"]["width"]["description"] = "The width of the window";
	m_config["window"]["width"]["default"] = 800;
	m_config["window"]["height"]["value"] = 600;
	m_config["window"]["height"]["description"] = "The height of the window";
	m_config["window"]["height"]["default"] = 600;
	m_config["window"]["theme"]["value"] = "dark";
	m_config["window"]["theme"]["description"] = "The theme of the application";
	m_config["window"]["theme"]["default"] = "light";
	m_config["window"]["is_fullscreen"]["value"] = false;
	m_config["window"]["is_fullscreen"]["description"] = "Whether the window is fullscreen";
	m_config["window"]["is_fullscreen"]["default"] = false;

	// 模块：network
	m_config["network"]["description"] = "Network related settings";
	m_config["network"]["timeout"]["value"] = 30;
	m_config["network"]["timeout"]["description"] = "Timeout for network requests";
	m_config["network"]["timeout"]["default"] = 30;
	m_config["network"]["retries"]["value"] = 3;
	m_config["network"]["retries"]["description"] = "Number of retry attempts";
	m_config["network"]["retries"]["default"] = 3;

	saveConfig();
}

template <typename T>
T ConfigManager::get(const std::string& module, const std::string& key, T defaultValue) {
	if (m_modules.find(module) != m_modules.end()) {
		if (m_modules[module].items.find(key) != m_modules[module].items.end()) {
			return m_modules[module].items[key].value.as<T>();
		}
	}
	return defaultValue;  // 如果配置项不存在，返回默认值
}

template <typename T>
void ConfigManager::set(const std::string& module, const std::string& key, const T& value) {
	if (m_modules.find(module) != m_modules.end()) {
		if (m_modules[module].items.find(key) != m_modules[module].items.end()) {
			m_modules[module].items[key].value = value;
			m_config[module][key]["value"] = value;
		}
	}
}

void ConfigManager::saveConfig() {
	try {
		std::ofstream file(m_filePath);
		file << m_config;
	}
	catch (const std::exception& e) {
		std::cerr << "Error saving configuration file: " << e.what() << std::endl;
	}
}

std::string ConfigManager::getDescription(const std::string& module, const std::string& key) {
	if (m_modules.find(module) != m_modules.end()) {
		if (m_modules[module].items.find(key) != m_modules[module].items.end()) {
			return m_modules[module].items[key].description;
		}
	}
	return "No description available";
}

void ConfigManager::displayAllConfigs() {
	for (auto& module : m_modules) {
		std::cout << module.first << ":" << std::endl;
		for (auto& item : module.second.items) {
			std::cout << "  " << item.first << ":" << std::endl;
			std::cout << "    Value: " << item.second.value.as<std::string>() << std::endl;
			std::cout << "    Description: " << item.second.description << std::endl;
		}
	}
}

*/
