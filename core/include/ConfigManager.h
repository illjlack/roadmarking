/*
#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

class ConfigManager {
public:
	// ���������ݽṹ��������������ǰֵ��Ĭ��ֵ
	struct ConfigItem {
		std::string description;
		YAML::Node value;
		YAML::Node default_value;
	};

	// ģ�����ݽṹ������������һ��������
	struct Module {
		std::string description;
		std::unordered_map<std::string, ConfigItem> items;
	};

	// ���캯�������������ļ�·��
	ConfigManager(const std::string& filePath);

	// ���������ļ�
	void loadConfig();

	// ����Ĭ�������ļ�
	void createDefaultConfig();

	// ��ȡ�������ֵ������Ĭ��ֵ�������������ڣ�
	template <typename T>
	T get(const std::string& module, const std::string& key, T defaultValue);

	// �����������ֵ
	template <typename T>
	void set(const std::string& module, const std::string& key, const T& value);

	// �������õ��ļ�
	void saveConfig();

	// ��ȡ�����������
	std::string getDescription(const std::string& module, const std::string& key);

	// ��ʾ������������ڵ��ԣ�
	void displayAllConfigs();

private:
	std::string m_filePath;             // �����ļ�·��
	YAML::Node m_config;                // ��������
	std::unordered_map<std::string, Module> m_modules;  // �洢ģ���������
};

*/
