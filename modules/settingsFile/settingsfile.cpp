#include "settingsfile.h"

namespace SettingsFile{
    using namespace rapidjson;

    bool ReadSettingsFile(FileType ftype){
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
    void SaveConfigFile(FileType ftype){
        const char* filepath = ftype ? "localConfig.conf" : "defaultConfig.conf";

        if (ftype ? localJson.IsNull() : defaultJson.IsNull()){
            std::cout << "Attempted to save configFile to: " << filepath << ", but no settings object exist." << std::endl;
            return;
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

    std::string StringSetting(std::string settingName){
        bool local = false;
        if (!localJson.IsNull() && localJson.HasMember(settingName))
            local = true;
        else if (!defaultJson.HasMember(settingName)) {
            std::cout << "No setting named: " << settingName << " in default Settings file" << std::endl;
            return 0;
        }
        return local ?
               localJson[settingName].Get<std::string>()
                     : defaultJson[settingName].Get<std::string>();
    }
}