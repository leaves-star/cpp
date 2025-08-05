#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include "sys_rt.h"

using namespace std;
static std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

static SysRt *g_sysrt = nullptr;

SysRt::SysRt() {
    g_sysrt = this;
}

SysRt::~SysRt() {
    g_sysrt = nullptr;
}

SysRt *SysRt::getInstance() {
    return g_sysrt;
}

int SysRt::loadIni(std::string &iniFile) {
    SimpleIni ini;

    ini.SetUnicode();
    SI_Error rc = ini.LoadFile(iniFile.c_str());
    if (rc < 0) {
        return rc;
    };
    
    sIniFile = iniFile;

    const char *pv;

    pv = ini.GetValue("SYS", "N5G", "usb0");
    sN5G = std::string(pv);
    pv = ini.GetValue("SYS", "N4G", "usb1");
    sN4G = std::string(pv);
    pv = ini.GetValue("SYS", "NDET", "8.148.8.41");
    sNDET = std::string(pv);

    pv = ini.GetValue("ROBOT", "ID", "A12RSN1234567890");
    sRobotId = std::string(pv);
    pv = ini.GetValue("ROBOT", "KEY", "1234567890ABCDEF");
    sRobotKey = std::string(pv);
    pv = ini.GetValue("ROBOT", "CID", "1001");
    nCid = std::stoi(std::string(pv));
    pv = ini.GetValue("ROBOT", "PROTOCOL", "1.0");
    sProtocol = std::string(pv);

    pv = ini.GetValue("LORAHUB", "PORT", "/dev/ttyS0");
    lorahubPort = std::string(pv);
    pv = ini.GetValue("LORAHUB", "BAUD", "115200");
    lorahubBaud = std::stoi(std::string(pv));
    pv = ini.GetValue("LORAHUB", "RFCH", "32");
    nLoraRfCh = std::stoi(std::string(pv)) & 0xFF;

    pv = ini.GetValue("ROBOT", "SIDH", "C10R12SN240100");
    sSIDH = std::string(pv);
    pv = ini.GetValue("ROBOT", "IDS", "1,2,3,4,5,6,7");
    std::string sRids = std::string(pv);
    std::vector<std::string> slRids = split(sRids, ',');
    for (const std::string &token : slRids) {
        int id = std::stoi(token);
        lRids.push_back(id & 0xFF);
    }

    pv = ini.GetValue("DEBUG", "LEVEL", "0");
    debugLevel = std::stoi(std::string(pv));

    pv = ini.GetValue("MQTT", "URL", "mqtt://127.0.0.1:1883");
    sMqttUrl = std::string(pv);
    pv = ini.GetValue("MQTT", "SUBTOPIC", "C10/CUST/DOWNLINK/");
    sMqttSubTopic = std::string(pv);
    pv = ini.GetValue("MQTT", "PUBTOPIC", "C10/CUST/UPLINK/");
    sMqttPubTopic = std::string(pv);

    pv = ini.GetValue("GN5G", "URL", "mqtt://10.153.92.131:1883");
    s5gMqttUrl = std::string(pv);
    pv = ini.GetValue("GN5G", "SUBTOPIC", "C10/CUST/DOWNLINK/");
    s5gMqttSubTopic = std::string(pv);
    pv = ini.GetValue("GN5G", "PUBTOPIC", "C10/CUST/UPLINK/");
    s5gMqttPubTopic = std::string(pv);
    pv = ini.GetValue("GN5G", "USER", "user");
    s5gMqttUser = std::string(pv);
    pv = ini.GetValue("GN5G", "PSWD", "pswd");
    s5gMqttPswd = std::string(pv);

    pv = ini.GetValue("DRIVE", "LeftDriveCorrection", "0");
    nLeftDriveCorrection = std::stoi(std::string(pv));
    pv = ini.GetValue("DRIVE", "RightDriveCorrection", "0");
    nRightDriveCorrection = std::stoi(std::string(pv));
    pv = ini.GetValue("DRIVE", "MovingSpeedMax", "0.5");
    fMovingSpeedMax = std::stof(std::string(pv));
    
    return 0;
}

int SysRt::saveIni(void) {
    SimpleIni ini;
    char sbuf[32];

    ini.SetValue("SYS", "N5G", sN5G.c_str());
    ini.SetValue("SYS", "N4G", sN4G.c_str());
    ini.SetValue("SYS", "NDET", sNDET.c_str());

    ini.SetValue("ROBOT", "ID", sRobotId.c_str());
    ini.SetValue("ROBOT", "KEY", sRobotKey.c_str());
    sprintf(sbuf, "%d", nCid);
    ini.SetValue("ROBOT", "CID", sbuf);
    ini.SetValue("ROBOT", "PROTOCOL", sProtocol.c_str());

    ini.SetValue("LORAHUB", "PORT", lorahubPort.c_str());
    sprintf(sbuf, "%d", lorahubBaud);
    ini.SetValue("LORAHUB", "BAUD", sbuf);
    sprintf(sbuf, "%d", nLoraRfCh);
    ini.SetValue("LORAHUB", "RFCH", sbuf);

    ini.SetValue("ROBOT", "SIDH", sSIDH.c_str());
    // TODO

    sprintf(sbuf, "%d", debugLevel);
    ini.SetValue("DEBUG", "LEVEL", sbuf);
    
    ini.SetValue("MQTT", "URL", sMqttUrl.c_str());
    ini.SetValue("MQTT", "SUBTOPIC", sMqttSubTopic.c_str());
    ini.SetValue("MQTT", "PUBTOPIC", sMqttPubTopic.c_str());

    ini.SetValue("GN5G", "URL", s5gMqttUrl.c_str());
    ini.SetValue("GN5G", "SUBTOPIC", s5gMqttSubTopic.c_str());
    ini.SetValue("GN5G", "PUBTOPIC", s5gMqttPubTopic.c_str());
    ini.SetValue("GN5G", "USER", s5gMqttUser.c_str());
    ini.SetValue("GN5G", "PSWD", s5gMqttUser.c_str());

    sprintf(sbuf, "%d", nLeftDriveCorrection);
    ini.SetValue("DRIVE", "LeftDriveCorrection", sbuf);
    sprintf(sbuf, "%d", nRightDriveCorrection);
    ini.SetValue("DRIVE", "RightDriveCorrection", sbuf);
    sprintf(sbuf, "%.2f", fMovingSpeedMax);
    ini.SetValue("DRIVE", "MovingSpeedMax", sbuf);
    
    return ini.SaveFile(sIniFile.c_str());
}

void SysRt::updateFromJson(cJSON *jpara) {
    cJSON *jitem;

    cJSON *jDEBUG = cJSON_GetObjectItem(jpara, "DEBUG");
    if (jDEBUG != nullptr) {
        cJSON *jDEBUG_LEVEL = cJSON_GetObjectItem(jDEBUG, "LEVEL");
        if (jDEBUG_LEVEL != nullptr) {
            debugLevel = jDEBUG_LEVEL->valueint;
        }
    }

    cJSON *jDRIVE = cJSON_GetObjectItem(jpara, "DRIVE");
    if (jDRIVE != nullptr) {
        cJSON *jDRIVE_LeftDriveCorrection = cJSON_GetObjectItem(jDRIVE, "LeftDriveCorrection");
        if (jDRIVE_LeftDriveCorrection != nullptr) {
            nLeftDriveCorrection = jDRIVE_LeftDriveCorrection->valueint;
        }
        cJSON *jDRIVE_RightDriveCorrection = cJSON_GetObjectItem(jDRIVE, "RightDriveCorrection");
        if (jDRIVE_RightDriveCorrection != nullptr) {
            nRightDriveCorrection = jDRIVE_RightDriveCorrection->valueint;
        }
        cJSON *jDRIVE_MovingSpeedMax = cJSON_GetObjectItem(jDRIVE, "MovingSpeedMax");
        if (jDRIVE_MovingSpeedMax != nullptr) {
            fMovingSpeedMax = (float)jDRIVE_MovingSpeedMax->valuedouble;
        }
    }
}
