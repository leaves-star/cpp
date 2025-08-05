#ifndef __SYS_RT_H__
#define __SYS_RT_H__

#include "simple_ini.h"
#include "cjson/cJSON.h"

using SimpleIni = CSimpleIniA;

class SysRt {
public:
    SysRt();
    ~SysRt();

    static SysRt *getInstance();

    int loadIni(std::string &iniFile);
    int saveIni(void);
    void updateFromJson(cJSON *jpara);

    std::string getN5G() {
        return sN5G;
    }

    std::string getN4G() {
        return sN4G;
    }

    std::string getNDET() {
        return sNDET;
    }
    
    std::string getRobotId() {
        return sRobotId;
    }

    std::string getRobotKey() {
        return sRobotKey;
    }

    int getCid() {
        return nCid;
    }

    std::string getProtocol() {
        return sProtocol;
    }

    std::string getLorahubPort() {
        return lorahubPort;
    }

    int getLorahubBaud() {
        return lorahubBaud;
    }

    uint8_t getLoraRfCh() {
        return nLoraRfCh;
    }

    std::string getSIDH() {
        return sSIDH;
    }

    std::list<uint8_t> getRobotIds() {
        return lRids;
    }

    std::string getMqttUrl() {
        return sMqttUrl;
    }

    std::string getMqttSubTopic() {
        return sMqttSubTopic;
    }

    std::string getMqttPubTopic() {
        return sMqttPubTopic;
    }

    std::string getS5gMqttUrl() {
        return s5gMqttUrl;
    }

    std::string getS5gMqttSubTopic() {
        return s5gMqttSubTopic;
    }

    std::string getS5gMqttPubTopic() {
        return s5gMqttPubTopic;
    }

    std::string getS5gMqttUser() {
        return s5gMqttUser;
    }

    std::string getS5gMqttPswd() {
        return s5gMqttPswd;
    }

    int  getDebugLevel() {
        return debugLevel;
    }

    int getLeftDriveCorrection() {
        return nLeftDriveCorrection;
    }

    int getRightDriveCorrection() {
        return nRightDriveCorrection;
    }

    float getMovingSpeedMax() {
        return fMovingSpeedMax;
    }

private:
    std::string sIniFile;

    std::string sN5G;
    std::string sN4G;
    std::string sNDET;

    std::string sRobotId;
    std::string sRobotKey;
    int nCid;
    std::string sProtocol;

    std::string lorahubPort;
    int lorahubBaud;
    uint8_t nLoraRfCh;

    std::string sSIDH;
    std::list<uint8_t> lRids;

    std::string sMqttUrl;
    std::string sMqttSubTopic;
    std::string sMqttPubTopic;

    std::string s5gMqttUrl;
    std::string s5gMqttSubTopic;
    std::string s5gMqttPubTopic;
    std::string s5gMqttUser;
    std::string s5gMqttPswd;

    int debugLevel;

    int nLeftDriveCorrection;
    int nRightDriveCorrection;
    float fMovingSpeedMax;
};

#endif
