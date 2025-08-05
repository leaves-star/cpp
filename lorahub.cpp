#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include "sys_rt.h"
#include "lorahub.h"

namespace hnc10 {
    static LoraHub* g_lorahub = nullptr;

    LoraHub::LoraHub(std::string &port, int baud) {
        bExit = false;
        dbgLevel = 0;
        sPort = port;
        nBaud = baud;
        pSerial = nullptr;
        timeoutMs = 1000;

        bGotRsp = false;
        nRspCode = 0;
        nNoRspCnt = 0;

        g_lorahub = nullptr;
        id6wp = 0;
        bRfOffline = false;

        SysRt *psys = SysRt::getInstance();
        std::list<uint8_t> rids = psys->getRobotIds();
        nTotalRobots = rids.size();
        rRts = new RobotRt[nTotalRobots];
        int i = 0;
        for (const auto& value : rids) {
            rRts[i].id = i;
            rRts[i].robot_id = value;

            char idstr[8];
            sprintf(idstr, "%02d", rRts[i].robot_id);
            rRts[i].sRobotId = psys->getSIDH() + std::string(idstr);

            i++;
        }

        wiringPiSetupGpio();
        pinMode(1, OUTPUT);
        digitalWrite(1, LOW);
        pinMode(4, OUTPUT);
        digitalWrite(4, LOW);
        pinMode(5, OUTPUT);
        digitalWrite(5, LOW);
        system("gpio mode 1 output; gpio mode 4 output; gpio mode 5 output; gpio write 1 0; gpio write 4 0; gpio write 5 0");

        mqUl4 = create_queue("/msg_ul4");
        mqUl5 = create_queue("/msg_ul5");
        mqRfOff = create_queue("/msg_rfoff");

        struct mq_attr old_attr;
        mq_getattr(mqUl4, &old_attr);
        struct mq_attr new_attr = old_attr;
        new_attr.mq_flags |= O_NONBLOCK;
        mq_setattr(mqUl4, &new_attr, &old_attr);

        mq_getattr(mqUl5, &old_attr);
        new_attr = old_attr;
        new_attr.mq_flags |= O_NONBLOCK;
        mq_setattr(mqUl5, &new_attr, &old_attr);

        mq_getattr(mqRfOff, &old_attr);
        new_attr = old_attr;
        new_attr.mq_flags |= O_NONBLOCK;
        mq_setattr(mqRfOff, &new_attr, &old_attr);
    }

    LoraHub::~LoraHub() {
        if (pSerial != nullptr) {
            pSerial->Close();
        }
        mq_close(mqUl4);
        mq_close(mqUl5);
        mq_close(mqRfOff);
        mq_unlink("/msg_ul4");
        mq_unlink("/msg_ul5");
        mq_unlink("/msg_rfoff");
    }

    LoraHub* LoraHub::getInstance() {
        return g_lorahub;
    }

    int LoraHub::open() {
        pSerial = new SerialPort(sPort, (speed_t)nBaud);
        pSerial->SetNumDataBits(NumDataBits::EIGHT);
        pSerial->SetParity(Parity::NONE);
        pSerial->SetNumStopBits(NumStopBits::ONE);
        pSerial->SetTimeout(200);
        pSerial->Open();
        return (pSerial->GetState() == State::OPEN) ? 0 : -1;
    }

    void LoraHub::close() {
        bExit = true;
        if (pSerial != nullptr) {
            pSerial->Close();
            pSerial = nullptr;
        }
    }

    uint64_t LoraHub::get_time_ms() {
        struct timeval now;
        gettimeofday(&now, NULL);
        return (now.tv_sec*1000 + now.tv_usec/1000);
    }

    char LoraHub::bin2hex(uint8_t v) {
        if (v >= 0 && v <= 9) {
            return v + '0';
        } else if (v >= 0x0A && v <= 0x0F) {
            return v - 0x0A + 'A';
        } else {
            return ' ';
        }
    }

    std::string LoraHub::toHex(uint8_t* buff, int len) {
        std::string sret = "";
        for (int i = 0; i < len; i++) {
            char sbuf[8];
            sprintf(sbuf, "%02X", buff[i]);
            sret += std::string(sbuf);
        }
        return sret;
    }

    std::string LoraHub::toHex(std::vector<uint8_t>& qba) {
        std::string sret = "";
        for (int i = 0; i < qba.size(); i++) {
            char sbuf[8];
            sprintf(sbuf, "%02X", qba.at(i));
            sret += std::string(sbuf);
        }
        return sret;
    }

    bool LoraHub::isOpened(void) {
        return (pSerial != nullptr && pSerial->GetState() == State::OPEN);
    }

    void LoraHub::pktRcved(std::vector<uint8_t> &qba) {
        for (int i = 0; i < qba.size(); i++) {
            uint8_t d = (uint8_t)qba.at(i); 

            if (nSm == 0) {
                if (d == 0x7E) {
                    mPkt.head[0] = d;
                    nSm = 1;
                }
                continue;
            } else if (nSm == 1) {
                if (d == 0xE7) {
                    mPkt.head[1] = d;
                    nSm = 2;
                } else {
                    nSm = 0;
                }
            } else if (nSm == 2) {
                mPkt.src = d;
                nSm = 3;
            } else if (nSm == 3) {
                mPkt.dst = d;
                nSm = 4;
            } else if (nSm == 4) {
                mPkt.cmd = d;
                nSm = 5;
                nIdx = 0;
            } else if (nSm == 5) {
                mPkt.payload[nIdx++] = d;
                if (nIdx >= UPKT_PAYLOAD_LEN) {
                    nSm = 6;
                }
            } else if (nSm == 6) {
                mPkt.chksum = d;
                nSm = 7;
            } else if (nSm == 7) {
                mPkt.tail = d;
                nSm = 0;
                if (mPkt.tail == 0xA5) {
                    pktProc(mPkt);
                } else {
                    zout << "error packet......" << std::endl;
                }
            } else {
                nSm = 0;
            }
        }
    }

    void LoraHub::pktProc(Upkt &pkt) {
        RobotRt *pRt = getRobotRt(pkt.src);
        if (pRt == nullptr) {
            zout << "sub id out of range: " << pkt.src << std::endl;
            return;
        }

        pRt->timestamp = get_time_ms() / 1000;

        pRt->tstate = pkt.payload[0];
        pRt->dstate = pkt.payload[1];
        pRt->io = pkt.payload[2];
        pRt->moc = pkt.payload[3];
        pRt->mfault = pkt.payload[4];

        pRt->brush = (pRt->dstate & 0x01) ? 0x01 : 0x00;
        pRt->woter_pump = (pRt->dstate & 0x02) ? 0x01 : 0x00;
        pRt->push_rod = (pRt->dstate & 0x04) ? 0x01 : 0x00;

        uint16_t uint16v;
        uint16v = (uint16_t) (pkt.payload[5] | (pkt.payload[6]<<8));
        pRt->battv = (float)uint16v / 1000.0f;

        uint16v = (uint16_t) (pkt.payload[7] | (pkt.payload[8]<<8));
        pRt->lw_current = (float)uint16v / 1000.0f;

        uint16v = (uint16_t) (pkt.payload[9] | (pkt.payload[10]<<8));
        pRt->rw_current = (float)uint16v / 1000.0f;

        uint16v = (uint16_t) (pkt.payload[11] | (pkt.payload[12]<<8));
        pRt->br1_current = (float)uint16v / 1000.0f;

        uint16v = (uint16_t) (pkt.payload[13] | (pkt.payload[14]<<8));
        pRt->br2_current = (float)uint16v / 1000.0f;

        uint16v = (uint16_t) (pkt.payload[15] | (pkt.payload[16]<<8));
        pRt->tg_current = (float)uint16v / 1000.0f;

        pRt->btemp = (int16_t)(pkt.payload[17] | (pkt.payload[18] << 8));

        if (pRt->tstate == SYS_STATE_UNKNOW || pRt->tstate == SYS_STATE_INITED 
                || pRt->tstate == SYS_STATE_STANDBY) {
            pRt->state = ROBOT_STATE_STANDBY;
            pRt->sfault = "";
        } else if (pRt->tstate == SYS_STATE_WORKING) {
            pRt->state = ROBOT_STATE_CLEANING;
            pRt->sfault = "";
        } else if (pRt->tstate == SYS_STATE_WORKDONE) {
            pRt->state = ROBOT_STATE_DONE;
            pRt->sfault = "";
        }else if (pRt->tstate == SYS_STATE_BATV_LOW) {
            pRt->state = ROBOT_STATE_STANDBY;
            pRt->sfault = "battery low";
        } else {
            pRt->state = ROBOT_STATE_FAULT;
            if (pRt->tstate == SYS_STATE_DRIVE_OC) {
                pRt->sfault = "drive over current";
            } else if (pRt->tstate == SYS_STATE_BRUSH_OC) {
                pRt->sfault = "brush over current";
            } else if (pRt->tstate == SYS_STATE_TG_OC) {
                pRt->sfault = "push rod over current";
            } else if (pRt->tstate == SYS_STATE_FAULT) {
                pRt->sfault = "motor fault";
            } else if (pRt->tstate == SYS_STATE_LOCKED) {
                pRt->sfault = "locked";
            } else if (pRt->tstate == SYS_STATE_OVER_TIME) {
                pRt->sfault = "task over time";
            } else {
                pRt->sfault = "unknow";
            }
        }

        zout << pRt->tostring() << std::endl;

        int n = 0;
        char msgId[12];
        sprintf(msgId, "%08x", pRt->id);
        n = send_msg(mqUl4, msgId);
        zout << "send_msg_ul4: " << n << std::endl;
        n = send_msg(mqUl5, msgId);
        zout << "send_msg_ul5: " << n << std::endl;
    }

    void LoraHub::printPkt(uint8_t *p, int len) {
        char buff[16];
        std::cout << "  ~~pkt: ";
        for (int i=0; i<len; i++) {
            sprintf(buff, " %02x", p[i]);
            std::cout << buff;
        }
        std::cout << " ~~" << std::endl;
    }

    int LoraHub::sendPkt(dpkt_t *pkt, bool waitrsp, int retry) {
        pkt->head[0] = 0x7E;
        pkt->head[1] = 0xE7;
        pkt->src = 0xFF;
        pkt->tail = 0xA5;

        pkt->chksum = pkt->src + pkt->dst + pkt->cmd + pkt->reserved;
        for (int i=0; i<DPKT_PAYLOAD_LEN; i++) {
            pkt->chksum += pkt->payload[i];
        }
        
        std::vector<uint8_t> sndbuf;
        uint8_t *pbuf = (uint8_t *)pkt;
        for (int i=0; i<sizeof(dpkt_t); i++) {
            sndbuf.push_back(pbuf[i]);
        }
        zout << "[S]: " << toHex(sndbuf) << std::endl;

        bGotRsp = false;
        nRspCode = 0x00;
        for (int i=0; i<retry; i++) {
            pSerial->WriteBinary(sndbuf);
            if (!waitrsp) {
                return 0;
            }

            int tms = timeoutMs;
            while (!bGotRsp && tms > 0) {
                usleep(10*1000);
                tms -= 10;
            }
            if (bGotRsp) {
                nNoRspCnt = 0;
                break;
            } else {
                char ch[8];
                sprintf(ch, "%02d", pkt->cmd);
                zout << "wait command response timeout, cmd: [" << ch << "], retry: " << i << std::endl;
                nNoRspCnt++;
            }
        }
        return bGotRsp?0:-1;
    }

    int LoraHub::cmdDrive(uint8_t id, bool dir, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Drive[" << id << "]: " << (dir?"forward":"backward") << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = dir?CMD_DRV_FORWARD:CMD_DRV_BACKWARD;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdDriveStop(uint8_t id, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Drive Stop[" << id << "]..." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_DRV_STOP;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdBrush(uint8_t id, bool dir, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Brush[" << id << "]: " << (dir?"forward":"backward") << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = dir?CMD_BRUSH_FORWARD:CMD_BRUSH_BACKWARD;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdBrushStop(uint8_t id, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Drive Stop[" << id << "]..." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_BRUSH_OFF;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdWoterPump(uint8_t id, bool onoff, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Woter Pump[" << id << "]: " << (onoff?"on":"off") << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = onoff?CMD_WOTER_PUMP_ON:CMD_WOTER_PUMP_OFF;
        return sendPkt(&pkt, waitrsp);
    }
    
    int LoraHub::cmdPushRod(uint8_t id, bool dir, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Push Rod[" << id << "]: " << (dir?"lock":"unlock") << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = dir?CMD_TG_LOCK:CMD_TG_UNLOCK;
        return sendPkt(&pkt, waitrsp);
    }
    
    int LoraHub::cmdFan(uint8_t id, bool onoff, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Fan[" << id << "]: " << (onoff?"on":"off") << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = onoff?CMD_FAN_ON:CMD_FAN_OFF;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdQueryState(uint8_t id) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Query State[" << id << "]..." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_QUERY_STATE;
        return sendPkt(&pkt, false);
    }

    int LoraHub::cmdTaskStart(uint8_t id, bool brush, bool woter, uint8_t round, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Task Start[" << id << "]..." << std::endl;
        }

        if (brush) {
            if (id == 0x06) {
                // 主控箱对应的机器人ID，由主控器直接操作水泵加水，加水时长25秒
                id6wp = get_time_ms();
                zout << "!!!!!!!!!!!!!!!open woter pump" << std::endl;
                pinMode(4, OUTPUT);
                digitalWrite(4, HIGH);
                system("gpio mode 4 output; gpio write 4 1;");
                sleep(1);

                //sleep(24);
                //digitalWrite(4, LOW);
                //digitalWrite(5, LOW);
                dpkt_t pkt1;

                zout << " ~CMD Task Start[" << id << "], open woter pump @09 for 25 secs..." << std::endl;
                pkt1.src = 0xFF;
                pkt1.dst = 0x09;
                pkt1.reserved = 0x00;
                pkt1.payload[0] = 0x20;
                pkt1.payload[1] = 0x00;
                pkt1.payload[2] = 0x00;
                pkt1.payload[3] = 0x00;
                pkt1.cmd = CMD_WOTER_PUMP_ON;
                sendPkt(&pkt1, waitrsp);
                sleep(2);
            } else if (id == 0x07) {
                // 副控制箱对应的机器人ID，由ID=08的控制板开水泵加水
                dpkt_t pkt1;

                zout << " ~CMD Task Start[" << id << "], open woter pump @08 for 25 secs..." << std::endl;
                pkt1.src = 0xFF;
                pkt1.dst = 0x08;
                pkt1.reserved = 0x00;
                pkt1.payload[0] = 0x20;
                pkt1.payload[1] = 0x00;
                pkt1.payload[2] = 0x00;
                pkt1.payload[3] = 0x00;
                pkt1.cmd = CMD_WOTER_PUMP_ON;
                sendPkt(&pkt1, waitrsp);
                sleep(2);
            }
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = brush ? 0x01 : 0x00;
        pkt.payload[1] = woter ? 0x01 : 0x00;
        pkt.payload[2] = round;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_START;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdTaskStop(uint8_t id, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Task Stop[" << id << "]..." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_STOP;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdCfgScheduleTask1(uint8_t id, uint8_t hour, uint8_t minute, uint8_t round, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Config Schedule Task1[" << id << "]: " << hour << ":" << minute << ":0, round: " << round << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = hour;
        pkt.payload[1] = minute;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = round;

        pkt.cmd = CMD_CFG_TASK1;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdCfgScheduleTask2(uint8_t id, uint8_t hour, uint8_t minute, uint8_t round, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Config Schedule Task2[" << id << "]: " << hour << ":" << minute << ":0, round: " << round << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = hour;
        pkt.payload[1] = minute;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = round;

        pkt.cmd = CMD_CFG_TASK2;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdCfgNoTaskBattv(uint8_t id, float batv, bool waitrsp) {
        dpkt_t pkt;
        uint16_t batv16 = (batv * 1000);

        if (dbgLevel >= 1) {
            zout << " ~CMD Config No Task BattV[" << id << "]: " << batv << "V." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = batv16 & 0xFF;
        pkt.payload[1] = (batv16 & 0xFF00) >> 8;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_CFG_NOTASK_BATV;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdCfgCurrentThreshold(uint8_t id, float tg, float drive, bool waitrsp) {
        dpkt_t pkt;
        uint16_t tg16 = (tg * 1000);
        uint16_t drive16 = (drive * 1000);

        if (dbgLevel >= 1) {
            zout << " ~CMD Config Current Threshold[" << id << "], TG:" << tg << "A, Drive: " << drive << "A." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = tg16 & 0xFF;
        pkt.payload[1] = (tg16 & 0xFF00) >> 8;
        pkt.payload[2] = drive16 & 0xFF;
        pkt.payload[3] = (drive16 & 0xFF00) >> 8;

        pkt.cmd = CMD_CFG_CUR_THRES;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdCfgOcThd(uint8_t id, float drive_oc, float brush_oc, bool waitrsp) {
        dpkt_t pkt;
        uint16_t doc16 = (drive_oc * 1000);
        uint16_t boc16 = (drive_oc * 1000);

        if (dbgLevel >= 1) {
            zout << " ~CMD Config OC THD[" << id << "], DRIVE:" << doc16 << "A, Brush: " << boc16 << "A." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = doc16 & 0xFF;
        pkt.payload[1] = (doc16 & 0xFF00) >> 8;
        pkt.payload[2] = boc16 & 0xFF;
        pkt.payload[3] = (boc16 & 0xFF00) >> 8;

        pkt.cmd = CMD_CFG_OC_THD;
        return sendPkt(&pkt, waitrsp);
    }

    int LoraHub::cmdCfgSysReboot(uint8_t id, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Sys Reboot[" << id << "]..." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_REBOOT;
        return sendPkt(&pkt, false);
    }

    int LoraHub::cmdCfgSysResetFactory(uint8_t id, bool waitrsp) {
        dpkt_t pkt;

        if (dbgLevel >= 1) {
            zout << " ~CMD Sys Reset Factory[" << id << "]..." << std::endl;
        }

        pkt.src = 0xFF;
        pkt.dst = id;
        pkt.reserved = 0x00;
        pkt.payload[0] = 0x00;
        pkt.payload[1] = 0x00;
        pkt.payload[2] = 0x00;
        pkt.payload[3] = 0x00;

        pkt.cmd = CMD_RESET_FACTORY;
        return sendPkt(&pkt, false);
    }

    int LoraHub::broadcastTime(std::tm* now_tm) {
        dpkt_t pkt;

        pkt.src = 0xFF;
        pkt.dst = 0xFF;
        pkt.reserved = now_tm->tm_year - 2000;
        pkt.payload[0] = now_tm->tm_mon;
        pkt.payload[1] = now_tm->tm_mday;
        pkt.payload[2] = now_tm->tm_hour;
        pkt.payload[3] = now_tm->tm_min;

        pkt.cmd = CMD_RESET_FACTORY;
        return sendPkt(&pkt, false);
    }

    void LoraHub::configRf(uint8_t chn) {
        uint8_t lora_cfg[] = {0xC0, 0x00, 0x08, 0xFF, 0xFF, 0x63, 0x00, 0x20, 0x00, 0x00, 0x00};

        if (dbgLevel >= 1) {
            zout << " ~CMD RF Channel: " << chn << std::endl;
        }

        lora_cfg[7] = chn;

        digitalWrite(1, HIGH);
        usleep(500*1000);

        std::vector<uint8_t> sndbuf;
        for (int i=0; i<sizeof(lora_cfg); i++) {
            sndbuf.push_back(lora_cfg[i]);
        }
        pSerial->WriteBinary(sndbuf);

        std::vector<uint8_t> rcv_data;
        pSerial->ReadBinary(rcv_data);
        rcv_data.clear();
        pSerial->ReadBinary(rcv_data);
        rcv_data.clear();
        pSerial->ReadBinary(rcv_data);
        usleep(500*1000);
        digitalWrite(1, LOW);
        usleep(100*1000);
    }

    void LoraHub::run(void) {
        int idx;
        uint64_t ctkRcv;
        uint64_t ctk, ctk1;
        std::vector<uint8_t> rcv_data;
        SysRt *psys = SysRt::getInstance();

        rePowerLoraModule(6);

        taskState = TASK_RUNNING;
        zout << "LoraHub thread started..." << std::endl;
        int rtv = open();
        zout << "LoraHub open serial: " << rtv << std::endl;

        configRf(psys->getLoraRfCh());
        zout << "LoraHub config RF done..." << std::endl;
        sleep(1);

        bRfOffline = false;
        idx = 0;
        ctk = get_time_ms();
        ctkRcv = ctk;
        while (!bExit) {
            rcv_data.clear();
            pSerial->ReadBinary(rcv_data);
            if (rcv_data.size() > 0) {
                zout << "[R]: " << toHex(rcv_data) << std::endl;
                pktRcved(rcv_data);
                bRfOffline = false;
                ctkRcv = get_time_ms();
                continue;
            }

            usleep(100*1000);
            ctk1 = get_time_ms();
            if ((ctk1 - ctk) >= 2000) {
                ctk = get_time_ms();
                cmdQueryState(rRts[idx].robot_id);
                //zout << "LoraHub query robot, id: " << rRts[idx].sRobotId << std::endl;
                idx++;
                if (idx >= nTotalRobots) {
                    idx = 0;
                }
            }

            if (id6wp != 0 && (ctk1 - id6wp) >= 32000) {
                // 如果6号机器人开了水泵
                zout << "!!!!!!!!!!!!!!!close woter pump" << std::endl;
                pinMode(4, OUTPUT);
                digitalWrite(4, LOW);
                system("gpio mode 4 output; gpio write 4 0;");
                id6wp = 0;
            }

            if ((ctk1 - ctkRcv) > 60*1000) {
                zout << "NO RF DATA received for 60 seconds, re-power lora module..." << std::endl;
                bRfOffline = true;

                char msgId[12];
                sprintf(msgId, "%08x", ctkRcv/1000);
                send_msg(mqRfOff, msgId);
                zout << "send_msg_rfoff..." << std::endl;

                rePowerLoraModule(6);
                sleep(1);
                ctkRcv = get_time_ms();
            }

            // 整点的时候广播一下时间
            std::time_t now = std::time(nullptr);
            std::tm* now_tm = std::localtime(&now);
            if (now_tm->tm_min == 0 && now_tm->tm_sec >= 29 && now_tm->tm_sec <= 30) {
                sleep(1);
                char buffer[80];
                std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", now_tm);
                zout << " ~~~~BROADCAST TIME: " << std::string(buffer) <<  std::endl;
                broadcastTime(now_tm);
                sleep(1);
            }
        }

        close();
        taskState = TASK_FINISHED;
    }

    void LoraHub::updateRobotOnline(void) {
        uint64_t cts = get_time_ms();
        for (int i = 0; i < nTotalRobots; i++) {
            RobotRt *pRobot = &rRts[i];
            pRobot->online = (cts - pRobot->timestamp) < nTotalRobots*3;
        }
    }

    void LoraHub::rePowerLoraModule(int dlysecs) {
        zout << "~~~ re-power lora module" << std::endl;
        pinMode(5, OUTPUT);
        digitalWrite(5, HIGH);
        system("gpio mode 5 output; gpio write 5 1;");
        sleep(dlysecs);
        pinMode(5, OUTPUT);
        digitalWrite(5, LOW);
        system("gpio mode 5 output; gpio write 5 0;");
    }
}
