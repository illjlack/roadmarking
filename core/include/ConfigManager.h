#pragma once

#include <string>
#include <map>
#include <memory>
#include <cassert>
#include <stdexcept>

namespace roadmarking {

	// 配置项结构
	struct ConfigItem {
		std::string value;        // 当前值
		std::string default_value;// 默认值
		std::string description; // 描述
	};

	// 模块结构
	struct Module {
		std::string description;                     // 模块描述
		std::map<std::string, ConfigItem> items;    // 配置项映射
	};

	class ConfigManager {
	public:
		// 获取单例实例
		static ConfigManager& getInstance() {
			static ConfigManager instance;
			static bool initialized = false;
			if (!initialized) {
				instance.initialize();
				initialized = true;
			}
			return instance;
		}

		// 删除拷贝构造和赋值操作
		ConfigManager(const ConfigManager&) = delete;
		ConfigManager& operator=(const ConfigManager&) = delete;

		// 初始化配置管理器
		void initialize(const std::string& configPath = "config/config.properties");

		// 获取配置值
		template<typename T>
		T get(const std::string& module, const std::string& key) {
			if (!m_modules.count(module)) {
				throw std::runtime_error("配置模块不存在: " + module);
			}
			if (!m_modules.at(module).items.count(key)) {
				throw std::runtime_error("配置项不存在: " + module + "." + key);
			}
			const std::string& value = m_modules.at(module).items.at(key).value;
			try {
				if constexpr (std::is_same_v<T, int>) {
					return std::stoi(value);
				}
				else if constexpr (std::is_same_v<T, float>) {
					return std::stof(value);
				}
				else if constexpr (std::is_same_v<T, double>) {
					return std::stod(value);
				}
				else if constexpr (std::is_same_v<T, bool>) {
					return value == "true" || value == "1";
				}
				else if constexpr (std::is_same_v<T, std::string>) {
					return value;
				}
				else {
					static_assert(sizeof(T) == 0, "不支持的配置值类型");
				}
			}
			catch (const std::exception& e) {
				throw std::runtime_error("配置值转换失败 [" + module + "." + key + "]: " + e.what() + " (值: " + value + ")");
			}
		}

		// 设置配置值
		template<typename T>
		void set(const std::string& module, const std::string& key, const T& value) {
			auto modIt = m_modules.find(module);
			if (modIt != m_modules.end()) {
				auto& items = modIt->second.items;
				auto itemIt = items.find(key);
				if (itemIt != items.end()) {
					if constexpr (std::is_same_v<T, bool>) {
						itemIt->second.value = value ? "true" : "false";
					}
					else {
						itemIt->second.value = std::to_string(value);
					}
				}
			}
		}

		// 保存配置到文件
		void saveConfig();

		// 获取配置项描述
		std::string getDescription(const std::string& module, const std::string& key);

		// 显示所有配置
		void displayAllConfigs();

		// 重新加载配置
		void reloadConfig();

	private:
		// 私有构造函数
		ConfigManager() = default;

		// 私有析构函数
		~ConfigManager() = default;

		// 加载配置
		void loadConfig();

		// 创建默认配置
		void createDefaultConfig();

		std::string m_filePath;                     // 配置文件路径
		std::map<std::string, Module> m_modules;    // 模块映射
	};

} // namespace roadmarking
