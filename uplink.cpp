#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <openssl/aes.h>
#include "sys_rt.h"
#include "lorahub.h"
#include "uplink.h"

static const int	QOS = 1;
static const int	QOS2 = 2;
static const int	N_RETRY_ATTEMPTS = 20;

static bool mqttPubLost = false;
static bool mqttSubLost = false;
static const char *LWT_PAYLOAD = "{\"lwt\":\"OutOfService\"}";

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

namespace hnc10 {
    class PubCallback : public virtual mqtt::callback {
    public:
        void connection_lost(const std::string &cause) override {
            zout << "pub connection lost ..." << std::endl;
            if (!cause.empty())
                zout << "\tcause: " << cause << std::endl;
            mqttPubLost = true;
        }

        void delivery_complete(mqtt::delivery_token_ptr tok) override {
            zout << "\tdelivery complete for token: "
                << (tok ? tok->get_message_id() : -1) << std::endl;
        }
    };

    class action_listener : public virtual mqtt::iaction_listener {
        std::string name_;

        void on_failure(const mqtt::token& tok) override {
            zout << name_ << " failure";
            if (tok.get_message_id() != 0)
                std::cout << " for token: [" << tok.get_message_id() << "]";
            std::cout << std::endl;
        }

        void on_success(const mqtt::token& tok) override {
            zout << name_ << " success";
            if (tok.get_message_id() != 0)
                std::cout << " for token: [" << tok.get_message_id() << "]";
            auto top = tok.get_topics();
            if (top && !top->empty())
                std::cout << "\ttoken topic: '" << (*top)[0] << "', ...";
            std::cout << std::endl;
        }

    public:
        action_listener(const std::string& name) : name_(name) {}
    };

    class SubCallback : public virtual mqtt::callback, public virtual mqtt::iaction_listener {
        int nretry_;
        std::string sClientId;
        std::string sTopic;
        mqtt::async_client *cli_;
        mqtt::connect_options& connOpts_;
        action_listener subListener_;

        Uplink *parent;

        void reconnect() {
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            try {
                cli_->connect(connOpts_, nullptr, *this);
            } catch (const mqtt::exception& exc) {
                std::cerr << "sub error: " << exc.what() << std::endl;
                //exit(1);
            }
        }

        void on_failure(const mqtt::token& tok) override {
            zout << "sub connection attempt failed" << std::endl;
            if (++nretry_ > N_RETRY_ATTEMPTS) {
                //exit(1);
            }
            reconnect();
        }

        void on_success(const mqtt::token& tok) override {}

        void connected(const std::string& cause) override {
            zout << "\nsub connection success..." << std::endl;
            zout << "\nsubscribing to topic '" << sTopic << "'\n"
                << "\tfor client " << sClientId
                << " using QoS" << QOS2 << "\n" << std::endl;

            cli_->subscribe(sTopic, QOS2, nullptr, subListener_);
        }

        void connection_lost(const std::string& cause) override {
            zout << "\nsub onnection lost..." << std::endl;
            if (!cause.empty())
                zout << "\tcause: " << cause << std::endl;

            zout << "sub econnecting..." << std::endl;
            nretry_ = 0;
            reconnect();
        }

        void message_arrived(mqtt::const_message_ptr msg) override {
            zout << "message arrived: " << std::endl;
            zout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
            zout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;

            DownlinkCmd dcmd;
            dcmd.topic = msg->get_topic();
            dcmd.payload = msg->to_string();
            parent->rcvCmd(dcmd);
        }

        void delivery_complete(mqtt::delivery_token_ptr token) override {}

    public:
        SubCallback(Uplink *p, mqtt::async_client *cli, mqtt::connect_options &connOpts, std::string clientid, std::string &topic)
                    : parent(p), nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"), sClientId(clientid), sTopic(topic) {}
    };

    Uplink::Uplink(std::string mqttUrl, std::string robotId, std::string robotKey) {
        bExit = false;
        sMqttUrl = mqttUrl;
        sRobotId = robotId;
        sRobotKey = robotKey;

        srand(static_cast<unsigned int>(time(nullptr)));
        sClientId = sRobotId + std::to_string((rand()%1000));

        pthread_mutex_init(&mutex_, NULL);

        pThreadPool = nullptr;
        pLorahub = nullptr;

        pPubClient = new mqtt::async_client(sMqttUrl, sClientId+"_P");
        pSubClient = new mqtt::async_client(sMqttUrl, sClientId+"_S");
    }

    Uplink::~Uplink() {
        bExit = true;

        // Disconnect
        zout << "\nMQTT disconnecting...";
        try {
            pSubClient->disconnect()->wait();
            pPubClient->disconnect()->wait();
            zout << "  ...OK" << std::endl;
        } catch (const mqtt::exception& exc) {
		    std::cerr << exc << std::endl;
	    }
    }

    int Uplink::pubMsg(std::string &stopic, const char *msg, int qos) {
        try {
            //zout << "\nMQTT sending message..." << std::endl;
            mqtt::message_ptr pubmsg = mqtt::make_message(stopic, msg);
            pubmsg->set_qos(qos);
            pPubClient->publish(pubmsg)->wait_for(1000);
            //zout << "  ...OK" << std::endl;
            return 0;
        } catch (const mqtt::exception &exc) {
            std::cerr << exc.what() << std::endl;
            return -1;
        }
    }

    std::string Uplink::aesEncrypt(std::string data_str, const char *key) {
        const unsigned char *pt = (const unsigned char *)data_str.data();
        AES_KEY encKey;
        AES_set_encrypt_key((const unsigned char *)key, 128, &encKey); // 一块为128位，16字节。
        int length = 0;
        int len = strlen((char*)pt) + 1;
        if (len % 16 != 0) {
            length = ((len / 16) + 1) * 16;
        } else {
            length = len;
        }

        unsigned char ivec[AES_BLOCK_SIZE];
        memset(ivec, 9, sizeof(ivec));
        char *out = new char[length];
        AES_cbc_encrypt((const unsigned char *)pt, (unsigned char *)out, length, &encKey, ivec, AES_ENCRYPT);
        std::string res = std::string(out, length);
        delete[] out;
        return res;
    }

    std::string Uplink::aesDecrypt(std::string data_str, const char *key) {
        int length = data_str.size();
        unsigned char *data = new unsigned char[length];
        unsigned char ivec[AES_BLOCK_SIZE];
        AES_KEY deckey;
        memset(ivec, 9, sizeof(ivec));
        AES_set_decrypt_key((const unsigned char *)key, 128, &deckey);
        AES_cbc_encrypt((const unsigned char *)data_str.data(), data, length, &deckey, ivec, AES_DECRYPT);
        //zout << "还原数据:" << data << std::endl;
        std::string res = std::string((char *)data, length);
        delete[] data;
        return res;
    }

    std::string Uplink::base64Encode(const char *bytes_to_encode, unsigned int in_len) {
        std::string ret;
        int i = 0;
        int j = 0;
        unsigned char char_array_3[3];
        unsigned char char_array_4[4];
        while (in_len--) {
            char_array_3[i++] = *(bytes_to_encode++);
            if (i == 3) {
                char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
                char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
                char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
                char_array_4[3] = char_array_3[2] & 0x3f;
                for(i = 0; i <4 ; i++) {
                    ret += base64_chars[char_array_4[i]];
                }
                i = 0;
            }
        }
        if (i) {
            for(j = i; j < 3; j++) {
                char_array_3[j] = '\0';
            }
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for (j = 0; j < i + 1; j++) {
                ret += base64_chars[char_array_4[j]];
            }
            while((i++ < 3)) {
                ret += '=';
            }
        }
        return ret;
    }

    void Uplink::rptStats(RobotRt *cRt) {
        char fbuff[48];
        std::string robotid = buildRobotId(cRt->robot_id);

        //sprintf(fbuff, "%015ld", cRt->timestamp);
        //std::string sign_o = robotid + "_" + fbuff;
        //std::string sign_ = aesEncrypt(sign_o, sRobotKey.c_str()); 
        //std::string sign = base64Encode(sign_.c_str(), sign_.size());

        cJSON *jsonMsg = cJSON_CreateObject();
        cJSON_AddStringToObject(jsonMsg, "robotid", robotid.c_str());
        cJSON_AddStringToObject(jsonMsg, "custid", "1001");     // 客户ID
        cJSON_AddStringToObject(jsonMsg, "protocol", "1.0");    // 协议版本
        //cJSON_AddStringToObject(jsonMsg, "sign", sign.c_str()); // 签名
        cJSON_AddNumberToObject(jsonMsg, "timestamp", cRt->timestamp);
        sprintf(fbuff, "%.1f", cRt->battv);
        cJSON_AddStringToObject(jsonMsg, "battv", fbuff);
        cJSON_AddNumberToObject(jsonMsg, "btemp", cRt->btemp);
        cJSON_AddNumberToObject(jsonMsg, "tstate", cRt->tstate);
        cJSON_AddNumberToObject(jsonMsg, "dstate", cRt->dstate);
        cJSON_AddNumberToObject(jsonMsg, "io", cRt->io);
        cJSON_AddNumberToObject(jsonMsg, "moc", cRt->moc);
        cJSON_AddNumberToObject(jsonMsg, "mfault", cRt->mfault);
        sprintf(fbuff, "%.3f", cRt->lw_current);
        cJSON_AddStringToObject(jsonMsg, "lw_current", fbuff);
        sprintf(fbuff, "%.3f", cRt->rw_current);
        cJSON_AddStringToObject(jsonMsg, "rw_current", fbuff);
        sprintf(fbuff, "%.3f", cRt->br1_current);
        cJSON_AddStringToObject(jsonMsg, "br1_current", fbuff);
        sprintf(fbuff, "%.3f", cRt->br2_current);
        cJSON_AddStringToObject(jsonMsg, "br2_current", fbuff);
        sprintf(fbuff, "%.3f", cRt->tg_current);
        cJSON_AddStringToObject(jsonMsg, "tg_current", fbuff);

        cJSON_AddStringToObject(jsonMsg, "brush", (cRt->brush?"ON":"OFF"));
        cJSON_AddStringToObject(jsonMsg, "woter_pump", (cRt->woter_pump?"ON":"OFF"));
        cJSON_AddStringToObject(jsonMsg, "push_rod", (cRt->push_rod?"ON":"OFF"));
        cJSON_AddStringToObject(jsonMsg, "fan", (cRt->fan?"ON":"OFF"));

        sprintf(fbuff, "%.1f", cm.GetTemperature());
        cJSON_AddStringToObject(jsonMsg, "cpu_temp", fbuff);

        cJSON_AddNumberToObject(jsonMsg, "state", cRt->state);
        cJSON_AddStringToObject(jsonMsg, "faults", cRt->sfault.c_str());

        std::string stopic = buildPubTopic(robotid);
        char *spayload = cJSON_Print(jsonMsg);
        pubMsg(stopic, spayload, 0);

        zout << "[RPT4G], T: " << stopic << ", M: " << std::string(spayload) << std::endl;

        cJSON_Delete(jsonMsg);
        free(spayload);
    }

    void Uplink::rptRfOffline(void) {
        char fbuff[48];
        std::string robotid = buildRobotId(0xFF);

        cJSON *jsonMsg = cJSON_CreateObject();
        cJSON_AddStringToObject(jsonMsg, "robotid", robotid.c_str());
        cJSON_AddStringToObject(jsonMsg, "custid", "1001");     // 客户ID
        cJSON_AddStringToObject(jsonMsg, "protocol", "1.0");    // 协议版本
        cJSON_AddNumberToObject(jsonMsg, "timestamp", pLorahub->get_time_ms());

        sprintf(fbuff, "%.1f", cm.GetTemperature());
        cJSON_AddStringToObject(jsonMsg, "cpu_temp", fbuff);

        cJSON_AddNumberToObject(jsonMsg, "state", 3);
        cJSON_AddStringToObject(jsonMsg, "faults", "RF_OFFLINE");

        std::string stopic = buildPubTopic(robotid);
        char *spayload = cJSON_Print(jsonMsg);
        pubMsg(stopic, spayload, 0);

        zout << "[RPT4G], T: " << stopic << ", M: " << std::string(spayload) << std::endl;

        cJSON_Delete(jsonMsg);
        free(spayload);
    }

    std::string Uplink::buildRobotId(uint8_t id) {
        char fbuff[16];
        SysRt *psys = SysRt::getInstance();

        sprintf(fbuff, "%02d", id);
        return psys->getSIDH() + fbuff;
    }

    std::string Uplink::buildPubTopic(std::string &rid) {
        SysRt *psys = SysRt::getInstance();
        return psys->getMqttPubTopic() + rid + "/STATE";
    }

    void Uplink::handleCmd(uint8_t id, int cmd, cJSON *jpara) {
        cJSON *jitem1;
        switch (cmd) {
        case CMD_START:
        case 1002:
            {
                bool brush = true;
                bool woter = false;
                uint8_t round = 1;
                jitem1 = cJSON_GetObjectItem(jpara, "brush");
                if (jitem1 != nullptr) {
                    brush = strncmp(jitem1->valuestring, "on", 2) == 0;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "woter");
                if (jitem1 != nullptr) {
                    woter = strncmp(jitem1->valuestring, "on", 2) == 0;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "round");
                if (jitem1 != nullptr) {
                    round = jitem1->valueint & 0x0F;
                }
                zout << "@@@@@@4G task start, brush: " << (brush?"on":"off") << ", woter: " << (woter?"on":"off") << ", round: " << (int)round << std::endl;
                pLorahub->cmdTaskStart(id, brush, woter, round, false);
            }
            break;
        case CMD_STOP:
            pLorahub->cmdTaskStop(id, false);
            break;
        case CMD_CFG_TASK1:
            {
                uint8_t hour = 25, minute, round;
                jitem1 = cJSON_GetObjectItem(jpara, "hour");
                if (jitem1 != nullptr) {
                    hour = jitem1->valueint & 0x1F;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "minute");
                if (jitem1 != nullptr) {
                    minute = jitem1->valueint & 0x3F;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "round");
                if (jitem1 != nullptr) {
                    round = jitem1->valueint & 0x3F;
                }
                if (hour != 25) {
                    pLorahub->cmdCfgScheduleTask1(id, hour, minute, round, false);
                }
            }
            break;
        case CMD_CFG_TASK2:
            {
                uint8_t hour = 25, minute, round;
                jitem1 = cJSON_GetObjectItem(jpara, "hour");
                if (jitem1 != nullptr) {
                    hour = jitem1->valueint & 0x1F;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "minute");
                if (jitem1 != nullptr) {
                    minute = jitem1->valueint & 0x3F;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "round");
                if (jitem1 != nullptr) {
                    round = jitem1->valueint & 0x3F;
                }
                if (hour != 25) {
                    pLorahub->cmdCfgScheduleTask2(id, hour, minute, round, false);
                }
            }
            break;
        case CMD_CFG_CUR_THRES:
            {
                float tg = -1.0f, drive;
                jitem1 = cJSON_GetObjectItem(jpara, "tg");
                if (jitem1 != nullptr) {
                    tg = jitem1->valuedouble;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "drive");
                if (jitem1 != nullptr) {
                    drive = jitem1->valuedouble;
                }
                if (tg > 0.0f) {
                    pLorahub->cmdCfgCurrentThreshold(id, tg, drive, false);
                }
            }
            break;
        case CMD_CFG_OC_THD:
            {
                float doc = -1.0f, boc;
                jitem1 = cJSON_GetObjectItem(jpara, "drive_oc");
                if (jitem1 != nullptr) {
                    doc = jitem1->valuedouble;
                }
                jitem1 = cJSON_GetObjectItem(jpara, "brush_oc");
                if (jitem1 != nullptr) {
                    boc = jitem1->valuedouble;
                }
                if (doc > 0.0f) {
                    pLorahub->cmdCfgOcThd(id, doc, boc, false);
                }
            }
            break;
        case CMD_CFG_NOTASK_BATV:
            {
                float battv = -1.0f;
                jitem1 = cJSON_GetObjectItem(jpara, "battv");
                if (jitem1 != nullptr) {
                    battv = jitem1->valuedouble;
                }
                if (battv > 0.0f) {
                    pLorahub->cmdCfgNoTaskBattv(id, battv, false);
                }
            }
            break;
        case CMD_CFG_TIME:
            {
                // TODO
            }
            break;
        case CMD_DRV_FORWARD:
            pLorahub->cmdDrive(id, true, false);
            break;
        case CMD_DRV_BACKWARD:
            pLorahub->cmdDrive(id, false, false);
            break;
        case CMD_DRV_STOP:
            pLorahub->cmdDriveStop(id, false);
            break;
        case CMD_BRUSH_FORWARD:
            pLorahub->cmdBrush(id, true, false);
            break;
        case CMD_BRUSH_BACKWARD:
            pLorahub->cmdBrush(id, false, false);
            break;
        case CMD_BRUSH_OFF:
            pLorahub->cmdBrushStop(id, false);
            break;
        case CMD_WOTER_PUMP_ON:
            pLorahub->cmdWoterPump(id, true, false);
            break;
        case CMD_WOTER_PUMP_OFF:
            pLorahub->cmdWoterPump(id, false, false);
            break;
        case CMD_TG_LOCK:
            pLorahub->cmdPushRod(id, true, false);
            break;
        case CMD_TG_UNLOCK:
            pLorahub->cmdPushRod(id, false, false);
            break;
        case CMD_FAN_ON:
            pLorahub->cmdFan(id, true, false);
            break;
        case CMD_FAN_OFF:
            pLorahub->cmdFan(id, false, false);
            break;
        case CMD_REBOOT:
            pLorahub->cmdCfgSysReboot(id, false);
            break;
        case CMD_RESET_FACTORY:
            pLorahub->cmdCfgSysResetFactory(id, false);
            break;
//        case CMD_REBOOT:
//            zout << "!!!!!! SYSTEM REBOOT !!!!!!" << std::endl;
//            sleep(2);
//            system("sudo reboot");
//            break;
//        case CMD_SHUTDOWN:
//            zout << "!!!!!! SYSTEM SHUTDOWN !!!!!!" << std::endl;
//            sleep(2);
//            system("sudo shutdown -h now");
//            break;
        default:
            break;
        }
    }

    int Uplink::cmdBpCfgAll(cJSON *jpara) {
        SysRt *psys = SysRt::getInstance();
        zout << "^^ update config from json..." << std::endl;
        psys->updateFromJson(jpara);
        return 0;
    }

    int Uplink::cmdBpCfgSave(cJSON *jpara) {
        SysRt *psys = SysRt::getInstance();
        zout << "^^ save INI file..." << std::endl;
        psys->saveIni();
        return 0;
    }

    void Uplink::run(void) {
        taskState == TASK_RUNNING;
        zout << "mqtt server url: " << sMqttUrl << std::endl;
        zout << "mqtt pub topic: " << sPubTopic << std::endl;
        zout << "mqtt sub topic: " << sSubTopic << std::endl;

        PubCallback pubCb;
    	pPubClient->set_callback(pubCb);

        auto connPubOpts = mqtt::connect_options_builder()
            .clean_session()
            //.will(mqtt::message(sSubTopic, LWT_PAYLOAD, 1))
            .finalize();

        try {
            zout << "pub connecting..." << std::endl;
            mqtt::token_ptr conntok = pPubClient->connect(connPubOpts);
            zout << "waiting for the connection..." << std::endl;
            conntok->wait();
            zout << "waiting for the connection done." << std::endl;
            mqttPubLost = false;
        } catch (const mqtt::exception& exc) {
            zout << "pub connecting error: " << exc << std::endl;
        }

        ////////////////////////////////////////////////
        mqtt::connect_options connSubOpts;
	    connSubOpts.set_clean_session(true);
        // Install the callback(s) before connecting.
        SubCallback subCb(this, pSubClient, connSubOpts, sClientId, sSubTopic);
        pSubClient->set_callback(subCb);
        try {
            zout << "sub connecting to the MQTT server..." << std::flush;
            mqtt::token_ptr conntok = pSubClient->connect(connSubOpts, nullptr, subCb);
            conntok->wait();
            zout << "  ...OK" << std::endl;
        } catch (const mqtt::exception& exc) {
            zout << "pub connecting error: " << exc << std::endl;
        }
        ///////////////////////////////////////////////

        mqd_t mqUl4 = pLorahub->getMqUl4();
        mqd_t mqRfOff = pLorahub->getMqRfOff();

        struct timeval now;
        gettimeofday(&now, NULL);
        long rptTime = now.tv_sec*1000 + now.tv_usec/1000;
        long startTime = rptTime;
        while (!bExit) {
            usleep(200*1000);

            DownlinkCmd dcmd;
            if (getCmd(dcmd)) {
                gettimeofday(&now, NULL);
                long cmdTime = now.tv_sec*1000 + now.tv_usec/1000;
                gotCmdTime = cmdTime;
                if (cmdTime < (startTime+2000)) {
                    // 刚刚连接上收到的指令，丢弃
                    zout << "[4G] ~~~~drop json payload: " << dcmd.payload << std::endl;
                    continue;
                }

                cJSON *jcmd = cJSON_Parse(dcmd.payload.c_str());
                if (jcmd == nullptr) {
                    zout << "[4G] ~invalid json payload: " << dcmd.payload << std::endl;
                    continue;
                }

                cJSON *jitem = cJSON_GetObjectItem(jcmd, "robotid");
                std::string rcvrid = std::string(jitem->valuestring);
                if (!startsWith(rcvrid, sRobotId)) {
                    zout << "[4G] robot id not matched, received: " << rcvrid << ", expected: " << sRobotId << std::endl;
                    continue;
                }
                // TODO, 增加安全校验机制

                std::string sIdx = rcvrid.substr(sRobotId.length(), rcvrid.length());
                int rId = std::stoi(sIdx);

                jitem = cJSON_GetObjectItem(jcmd, "cmd");
                int cmd = jitem->valueint;
                jitem = cJSON_GetObjectItem(jcmd, "para");
                zout << "[4G] handle command[" << rId << "]: " << cmd << ", para: " << cJSON_Print(jitem) << std::endl;
                handleCmd(rId&0xFF, cmd, jitem);

                cJSON_Delete(jcmd);
            }

            if (mqttPubLost) {
                try {
                    zout << "[4G] pub re-connecting..." << std::endl;
                    mqtt::token_ptr conntok = pPubClient->connect(connPubOpts);
                    zout << "[4G] waiting for the re-connection..." << std::endl;
                    conntok->wait();
                    zout << "[4G] waiting for the re-connection done." << std::endl;
                    mqttPubLost = false;
                } catch (const mqtt::exception& exc) {
                    zout << "[4G] pub re-connecting error: " << exc << std::endl;
                }
            }

            char msgId[16];
            int msgN = 16;
            memset(msgId, 0, sizeof(msgId));
            if (receive_msg(mqUl4, msgId, msgN) == 0) {
                int id = std::stoi(std::string(msgId), nullptr, 16);
                RobotRt *pRt = pLorahub->getRobotRt(id);
                if (pRt == nullptr) {
                    continue;
                }
                zout << "[UL4] got robot report, id: " << pRt->sRobotId << std::endl;

                rptStats(pRt);
            }

            char msgId1[16];
            int msgN1 = 16;
            if (receive_msg(mqRfOff, msgId1, msgN1) == 0) {
                zout << "[UL4] got rf offline report..." << std::endl;
                rptRfOffline();
            }
        }

        taskState = TASK_FINISHED;
    }
}
