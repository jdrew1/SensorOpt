#ifndef NEURALPROJECT_SETTINGSFILE_H
#define NEURALPROJECT_SETTINGSFILE_H
#define RAPIDJSON_HAS_STDSTRING 1

#include <iostream>
#include <cstdio>
#include <string>
#include "rapidjson/document.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"


namespace SettingsFile{
    enum FileType {Local=true, Default=false};
    using namespace rapidjson;
    static Document defaultJson;
    static Document localJson;

    bool ReadSettingsFile(FileType ftype = Default){
        const char* filepath = ftype ? "./localConfig.conf" : "./defaultConfig.conf";
        std::ifstream readFile(filepath);
        if (!readFile){
            std::cout << std::endl << "Could not find settings file at:"<< filepath << std::endl;
            return false;
        }
        IStreamWrapper wrappedFile(readFile);
        if (ftype) localJson.ParseStream(wrappedFile);
        else defaultJson.ParseStream(wrappedFile);

        return true;
    }
    void SaveConfigFile(FileType ftype = Default){
        const char* filepath = ftype ? "./localConfig.conf" : "./defaultConfig.conf";
        if (ftype){
            if (localJson.IsNull()){
                std::cout << "Attempted to save configFile to: " << filepath << ", but no settings exist." << std::endl;
                return;
            }
        }
        else{
            if (defaultJson.IsNull()){
                std::cout << "Attempted to save configFile to: " << filepath << ", but no settings exist." << std::endl;
                return;
            }
        }
        std::ofstream writeFile(filepath);
        if (!writeFile){
            std::cout << "Attempted to save configFile to: " << filepath << ", error opening file" << std::endl;
            return;
        }
        OStreamWrapper wrappedFile(writeFile);
        Writer<OStreamWrapper> writer(wrappedFile);
        if (ftype) localJson.Accept(writer);
        else defaultJson.Accept(writer);
    }

    void InitSettings(){
        ReadSettingsFile(Default);
        if (!ReadSettingsFile(Local)){
            std::cout << "Init local settings" << std::endl;
            localJson.CopyFrom(defaultJson, defaultJson.GetAllocator());
            SaveConfigFile(Local);
        }
    }
    template <typename T>
    bool GetSetting(const char* settingName, T * in){
        bool local = false;
        if (!localJson.IsNull() && localJson.HasMember(settingName))
            local = true;

        *in = local ?
                localJson[settingName].Get<T>()
              : defaultJson[settingName].Get<T>();
        return true;
    }
	std::string StringSetting(const char * settingName){
		bool local = false;
		if (!localJson.IsNull() && localJson.HasMember(settingName))
			local = true;

		return local ?
			  localJson[settingName].Get<std::string>()
					: defaultJson[settingName].Get<std::string>();
	}
}
#endif //NEURALPROJECT_SETTINGSFILE_H