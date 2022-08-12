#ifndef NEURALPROJECT_SETTINGSFILE_H
#define NEURALPROJECT_SETTINGSFILE_H
#define RAPIDJSON_HAS_STDSTRING 1

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include "rapidjson/document.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"


namespace SettingsFile{
    enum FileType {Local=true, Default=false};
    static rapidjson::Document defaultJson;
    static rapidjson::Document localJson;

    bool ReadSettingsFile(FileType ftype = Default);
    void SaveConfigFile(FileType ftype = Default);

    void InitSettings();
	std::string StringSetting(std::string settingName);

}
#endif //NEURALPROJECT_SETTINGSFILE_H