#ifndef __UPLINK_H__
#define __UPLINK_H__

#include "mqtt/async_client.h"
#include "cjson/cJSON.h"
#include "thread_pool.h"
#include "lorahub.h"
#include "cpu_monitor.h"

namespace hnc10 {
    class DownlinkCmd {
    public:
        std::string topic;
        std::string payload;
    };

    class Uplink : public CommonTask {
    public:
        Uplink(std::string mqttUrl, std::string robotId, std::string robotKey);
        ~Uplink();

        void setMqttPara(std::string subTopic, std::string pubTopic) {
            //sSubTopic = subTopic + sRobotId;
            //sPubTopic = pubTopic + sRobotId + "/STATE";
            sSubTopic = subTopic;
            sPubTopic = pubTopic;
        }

        void setThreadPool(ThreadPool *pool) {
            pThreadPool = pool;
        }

        void setLoraHub(LoraHub *lorahub) {
            pLorahub = lorahub;
        }

        int pubMsg(std::string &stopic, const char *msg, int qos);
        int pubLog(const char *log);

        void rcvCmd(DownlinkCmd &dcmd) {
            pthread_mutex_lock(&mutex_);
            dCmds.push_back(dcmd);
            pthread_mutex_unlock(&mutex_);
        }

        int getCmd(DownlinkCmd &dcmd) {
            if (dCmds.size() == 0) {
                return 0;
            }
            pthread_mutex_lock(&mutex_);
            dcmd = dCmds.front();
            dCmds.pop_front();
            pthread_mutex_unlock(&mutex_);
            return 1;
        }
        
        int cmdBpCfgAll(cJSON *jpara);
        int cmdBpCfgSave(cJSON *jpara);

        void run(void);
        
    protected:
        void rptStats(RobotRt *cRt);
        void handleCmd(uint8_t id, int cmd, cJSON *jpara);

        void rptRfOffline(void);

        std::string aesEncrypt(std::string data_str, const char* key);
        std::string aesDecrypt(std::string data_str, const char* key);
        
        std::string base64Encode(const char *bytes_to_encode, unsigned int in_len);
        //std::vector<unsigned char> base64Decode(const std::string &encoded_string);

        std::string buildRobotId(uint8_t id);
        std::string buildPubTopic(std::string &rid);

        static bool startsWith(const std::string& str, const std::string& prefix) {
            return str.substr(0, prefix.size()) == prefix;
        }

    private:
        std::string sMqttUrl;
        std::string sRobotId;
        std::string sRobotKey;
        std::string sClientId;
        std::string sSubTopic;
        std::string sPubTopic;
        mqtt::async_client *pPubClient;
        mqtt::async_client *pSubClient;

        pthread_mutex_t mutex_;
        std::list<DownlinkCmd> dCmds;

        ThreadPool *pThreadPool;

        LoraHub *pLorahub;

        CpuMonitor cm;

        long gotCmdTime;
    };
}

#endif // __UPLINK_H__
