#ifndef __LORAHUB_H__
#define __LORAHUB_H__

#include <mqueue.h>
#include <CppLinuxSerial/SerialPort.hpp>
#include <wiringPi.h>
#include "common_task.h"

using namespace mn::CppLinuxSerial;

#define MAX_ROBOTS                      7

#define CMD_NONE                    0x00
#define CMD_QUERY_STATE             0x01
#define CMD_START                   0x02
#define CMD_STOP                    0x03
#define CMD_CFG_TASK1               0x04
#define CMD_CFG_TASK2               0x05
#define CMD_CFG_CUR_THRES           0x06
#define CMD_CFG_NOTASK_BATV         0x07
#define CMD_CFG_TIME                0x08
#define CMD_CFG_OC_THD              0x09

#define CMD_DRV_FORWARD             0x10
#define CMD_DRV_BACKWARD            0x11
#define CMD_DRV_STOP                0x12
#define CMD_BRUSH_FORWARD           0x13
#define CMD_BRUSH_BACKWARD          0x14
#define CMD_BRUSH_OFF               0x15
#define CMD_WOTER_PUMP_ON           0x16
#define CMD_WOTER_PUMP_OFF          0x17
#define CMD_TG_LOCK                 0x18
#define CMD_TG_UNLOCK               0x19
#define CMD_FAN_ON                  0x1A
#define CMD_FAN_OFF                 0x1B

#define CMD_REBOOT                  0x77
#define CMD_RESET_FACTORY           0x78

#define CMD_ACK                     0x80
#define CMD_ERR                     0x81
#define CMD_EVENT                   0x82

#define DPKT_PAYLOAD_LEN            4
#define UPKT_PAYLOAD_LEN            19

// 机器人内部的状态
#define SYS_STATE_UNKNOW                        0
#define SYS_STATE_INITED                        1
#define SYS_STATE_STANDBY                       2
#define SYS_STATE_WORKING                       3
#define SYS_STATE_WORKDONE                      4
#define SYS_STATE_BATV_LOW                      5       // 电池电压低
#define SYS_STATE_DRIVE_OC                      6       // 驱动轮过流
#define SYS_STATE_BRUSH_OC                      7       // 滚刷过流
#define SYS_STATE_TG_OC                         8       // 推杆过流
#define SYS_STATE_OVER_TIME                     9       // 任务超时
#define SYS_STATE_FAULT                         19
#define SYS_STATE_LOCKED                        99

// 上报给上位机系统的状态
#define ROBOT_STATE_STANDBY         0       // 待机
#define ROBOT_STATE_CLEANING        1       // 清扫中
#define ROBOT_STATE_DONE            2       // 清扫完成
#define ROBOT_STATE_FAULT           3       // 故障

namespace hnc10 {
    typedef struct {
        uint8_t head[2];
        uint8_t src;
        uint8_t dst;
        uint8_t cmd;
        uint8_t reserved;
        uint8_t payload[DPKT_PAYLOAD_LEN];
        uint8_t chksum;
        uint8_t tail;
    } dpkt_t;


    class Upkt {
    public:
        uint8_t head[2];
        uint8_t src;
        uint8_t dst;
        uint8_t cmd;
        uint8_t payload[UPKT_PAYLOAD_LEN];
        uint8_t chksum;
        uint8_t tail;
    };

    /*
    typedef struct {
        rt_uint8_t head1;
        rt_uint8_t head2;
        rt_uint8_t src;
        rt_uint8_t dst;
        rt_uint8_t cmd;
        rt_uint8_t tstate;          // task state
        rt_uint8_t dstate;          // drive state
        rt_uint8_t io;              // io state
        rt_uint8_t moc;             // motoe over current
        rt_uint8_t mfault;          // motor fault
        rt_uint16_t batv;
        rt_uint16_t drv1_current;
        rt_uint16_t drv2_current;
        rt_uint16_t drv3_current;
        rt_uint16_t drv4_current;
        rt_uint16_t tg_current;
        rt_uint8_t chksum;
        rt_uint8_t tail;
    } e220_pkt_t;
    */

    class RobotRt {
    public:
        int id = 0;
        std::string sRobotId = "";
        bool online = false;
        uint8_t robot_id = 0;
        uint8_t state = 0;
        uint8_t brush = 0;
        uint8_t woter_pump = 0;
        uint8_t push_rod = 0;
        int8_t btemp = 0;
        uint8_t fan = 0;

        uint8_t tstate = 0;
        uint8_t dstate = 0;
        uint8_t io = 0;
        uint8_t moc = 0;
        uint8_t mfault = 0;

        float lw_current = 0.0f;           // A, 左驱动轮电流
        float rw_current = 0.0f;           // A, 右驱动轮电流
        float br1_current = 0.0f;          // A, 滚刷1电流
        float br2_current = 0.0f;          // A, 滚刷2电流
        float tg_current = 0.0f;           // A, 推杆电流
        float battv = 0.0f;                // V, 电池电压
        
        long timestamp;

        std::string sfault = "";           // 故障描述

        std::string tostring() {
            char sbuf[64];
            std::string sout = "[" + sRobotId + "]: ";
            sprintf(sbuf, "state: %d, faults: %s", state, sfault.c_str());
            sout += std::string(sbuf) + ", ";
            sprintf(sbuf, "battv: %.1f, btemp: %d℃", battv, btemp);
            sout += std::string(sbuf) + ", ";
            sprintf(sbuf, "io: %02x, moc: %02x, mfault: %02x", io, moc, mfault);
            sout += std::string(sbuf) + ", ";
            sprintf(sbuf, "up drive current: %.2f, down drive current: %.2f", lw_current, rw_current);
            sout += std::string(sbuf) + ", ";
            sprintf(sbuf, "up brush current: %.2f, down brush current: %.2f", br1_current, br2_current);
            sout += std::string(sbuf);
            return sout;
        }
    };

    class LoraHub : public CommonTask {
    public:
        LoraHub(std::string &port, int baud);
        ~LoraHub();

        static LoraHub* getInstance();

        int open();
        void close();
        bool isOpened(void);

        int cmdDrive(uint8_t id, bool dir, bool waitrsp = true);
        int cmdDriveStop(uint8_t id, bool waitrsp = true);

        int cmdBrush(uint8_t id, bool dir, bool waitrsp = true);
        int cmdBrushStop(uint8_t id, bool waitrsp = true);

        int cmdWoterPump(uint8_t id, bool onoff, bool waitrsp = true);
        int cmdPushRod(uint8_t id, bool dir, bool waitrsp = true);
        int cmdFan(uint8_t id, bool onoff, bool waitrsp = true);

        int cmdQueryState(uint8_t id);
        int cmdTaskStart(uint8_t id, bool brush, bool woter, uint8_t round = 1, bool waitrsp = true);
        int cmdTaskStop(uint8_t id, bool waitrsp = true);

        int cmdCfgScheduleTask1(uint8_t id, uint8_t hour, uint8_t minute, uint8_t round, bool waitrsp = true);
        int cmdCfgScheduleTask2(uint8_t id, uint8_t hour, uint8_t minute, uint8_t round, bool waitrsp = true);
        int cmdCfgNoTaskBattv(uint8_t id, float batv, bool waitrsp = true);
        int cmdCfgCurrentThreshold(uint8_t id, float tg, float drive, bool waitrsp = true);
        int cmdCfgOcThd(uint8_t id, float tg, float drive, bool waitrsp = true);

        int cmdCfgSysReboot(uint8_t id, bool waitrsp = false);
        int cmdCfgSysResetFactory(uint8_t id, bool waitrsp = false);

        int broadcastTime(std::tm* now_tm);

        void configRf(uint8_t chn);

        int getTotalRobots() {
            return nTotalRobots;
        }

        RobotRt *getRobotRt(int id) {
            if (id >= nTotalRobots) {
                return nullptr;
            }
            return &rRts[id];
        }

        RobotRt *getRobotRt(uint8_t id) {
            for (int i = 0; i < nTotalRobots; i++) {
                if (rRts[i].robot_id == (id & 0xFF)) {
                    return &rRts[i];
                }
            }
            return nullptr;
        }
        
        int getTimeoutMs() {
            return timeoutMs;
        }

        void setTimeoutMs(int timeoutms) {
            timeoutMs = timeoutms;
        }

        void run(void);

        uint64_t get_time_ms();

        static char bin2hex(uint8_t v);
        std::string toHex(uint8_t* buff, int len);
        std::string toHex(std::vector<uint8_t>& qba);

        mqd_t getMqUl4() {
            return mqUl4;
        }
        
        mqd_t getMqUl5() {
            return mqUl5;
        }

        bool isRfOffline() {
            return bRfOffline;
        }

        mqd_t getMqRfOff() {
            return mqRfOff;
        }

    protected:
        void pktRcved(std::vector<uint8_t> &qba);
        void pktProc(Upkt &pkt);

        int sendPkt(dpkt_t *pkt, bool waitrsp = true, int retry=5);
        void printPkt(uint8_t *p, int len);

        void updateRobotOnline(void);
        void rePowerLoraModule(int dlysecs);

    private:
        int dbgLevel;
        std::string sPort;
        speed_t nBaud;
        SerialPort *pSerial;

        int nTotalRobots;
        RobotRt *rRts;

        int nSm;
        int nIdx;
        uint8_t cCrc;
        Upkt mPkt;

        int timeoutMs;
        bool bGotRsp;
        uint8_t nRspCode;
        int nNoRspCnt;

        mqd_t mqUl4;
        mqd_t mqUl5;

        uint64_t id6wp;

        bool bRfOffline;
        mqd_t mqRfOff;
    };
}

#endif // __LORAHUB_H__
