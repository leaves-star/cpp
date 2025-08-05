#include <stdio.h>      // included for `printf`
#include <getopt.h>     // included for `getopt_long`
#include <string.h>     // included for `basename`
#include <stdlib.h>     // included for `EXIT_SUCCESS|EXIT_FAILURE`
#include <unistd.h>
#include <iostream>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <net/if.h>
#include "sys_rt.h"
#include "thread_pool.h"
#include "lorahub.h"
#include "uplink.h"
#include "uplink5g.h"

using namespace hnc10;

#define VERSION "1.0.0"

#define RM500U_AT_DEV           "/dev/ttyUSB2"

// define option table
static const struct option longopts[] = {
    {"help", no_argument, NULL, 'h'},
    {"version", no_argument, NULL, 'V'},
    {"config", required_argument, NULL, 'g'},
    {NULL, 0, NULL, 0}
};

// Print help info.
static void print_help(const char *progname) {
    printf("Usage: %s [OPTION]...\n", progname);
    printf("Options:\n");
    printf("  -h, --help              display this help\n");
    printf("  -V, --version           display version information\n");
    printf("  -c, --config=FILEPATH   use FILEPATH as the config ini file\n");
}

static void print_version() {
    printf("%s\n", VERSION);
}

static ThreadPool *tpool = nullptr;

struct ethtool_value {
    __uint32_t cmd;
    __uint32_t data;
};

static int reset_rm500u(const char *atdev) {
    SerialPort *pSerial = new SerialPort(atdev, 9600);
    pSerial->SetNumDataBits(NumDataBits::EIGHT);
    pSerial->SetParity(Parity::NONE);
    pSerial->SetNumStopBits(NumStopBits::ONE);
    pSerial->SetTimeout(200);
    pSerial->Open();
    if (pSerial->GetState() != State::OPEN) {
        zout << "can't open rm500u tty device..." << std::endl;
        return -1;
    }

    pSerial->Write(std::string("AT+CFUN=1,1\r\n"));
    sleep(2);

    pSerial->Close();
    return 0;
}

/*return 1:has cable;   return 0:no cable*/
int detect_eth_cable(char *ifname) {
    struct ethtool_value edata;
    struct ifreq ifr;
    int fd = -1, err = 0;

    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, ifname);

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        //perror("Cannot get control socket");
        return -1;
     }
    edata.cmd = 0x0000000A;
    ifr.ifr_data = (caddr_t)&edata;
    err = ioctl(fd, 0x8946, &ifr);
    if (err == 0) {
        fprintf(stdout, "Link detected: %s\n", edata.data ? "yes":"no");
    } else if (errno != EOPNOTSUPP) {
        perror("Cannot get link status");
     }
    return (edata.data==1?1:0);
}

bool is_ppp_connected(const char* interfaceName) {
    int sockfd;
    struct ifreq ifr;
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    strncpy(ifr.ifr_name, interfaceName, IFNAMSIZ - 1);
    if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) == -1) {
        perror("Error in ioctl");
        return false;
    }
    
    bool connected = ((ifr.ifr_flags & IFF_UP) && (ifr.ifr_flags & IFF_RUNNING));
    close(sockfd);
    
    return connected;
}

static void ctrl_c_handler(int signo) {
    printf("oops! stop all tasks!!!\n");
    if (tpool != nullptr) {
        tpool->stop();
    }
    printf("system exit...\n");
    _exit(0);
}

int main(int argc, char **argv)  {
    int optc;
    const char *program_name = basename(argv[0]);
    int lose = 0;
    const char *config_file = NULL;
    int dnCnt = 0;
    // ----------

    while ((optc = getopt_long(argc, argv, "hVc", longopts, NULL)) != -1)
        switch (optc) {
            case 'h':
                print_help(program_name);
                exit(EXIT_SUCCESS);
                break;
            case 'V':
                print_version();
                exit(EXIT_SUCCESS);
                break;
            case 'g':
                config_file = optarg;
                break;
            default:
                lose = 1;
                break;
        }

    if (lose || optind < argc) {
        /* Print error message and exit.  */
        if (optind < argc)
            fprintf(stderr, "%s: extra operand: %s\n", program_name,
                    argv[optind]);
        fprintf(stderr, "Try `%s --help' for more information.\n",
                program_name);
        exit(EXIT_FAILURE);
    }

    std::string configIni = std::string(config_file);
    SysRt *sysrt = new SysRt();
    sysrt->loadIni(configIni);

    std::cout << "HNC10, robot id: " << sysrt->getRobotId() << std::endl;
    std::string lorahubPort = sysrt->getLorahubPort();
    std::cout << "LORAHUB port: " << lorahubPort << std::endl;
    std::cout << "LORAHUB baud: " << sysrt->getLorahubBaud() << std::endl;

__wait_net:
    int waitSecs = 0;
    bool bWaited = false;
    while (!is_ppp_connected(sysrt->getN5G().c_str())) {
        sleep(2);
        std::cout << "waiting " << sysrt->getN5G() << "..." << std::endl;
        bWaited = true;
        waitSecs += 2;
    }
    if (bWaited) {
        int result = 0;
        std::string pingCmd = "ping -I " + sysrt->getN5G() + " -c 4 " + sysrt->getNDET();
        do {
            result = system(pingCmd.c_str());
            if (result == 0) {
                std::cout << "Ping " << sysrt->getNDET() << " successful!" << std::endl;
            } else {
                std::cout << "Failed to connect." << std::endl;
            }
        } while (result != 0);
        waitSecs += 4;
        if (waitSecs > 90) {
            // 如果超过1分半钟网络还是没有接入，则重启一下5G模组
            reset_rm500u(RM500U_AT_DEV);
            goto __wait_net;
        }
    }

    ThreadPool threadPool(10);
    LoraHub *lorahub = new LoraHub(lorahubPort, sysrt->getLorahubBaud());
    Uplink *uplinker = new Uplink(sysrt->getMqttUrl(), sysrt->getRobotId(), sysrt->getRobotKey());
    Uplink5G *uplinker5g = new Uplink5G(sysrt->getS5gMqttUrl(), sysrt->getRobotId(), sysrt->getRobotKey());

    uplinker->setMqttPara(sysrt->getMqttSubTopic(), sysrt->getMqttPubTopic());
    uplinker->setThreadPool(&threadPool);
    uplinker->setLoraHub(lorahub);

    uplinker5g->setMqttPara(sysrt->getS5gMqttSubTopic(), sysrt->getS5gMqttPubTopic());
    uplinker5g->setThreadPool(&threadPool);
    uplinker5g->setLoraHub(lorahub);

    threadPool.addTask(lorahub);
    sleep(1);
    threadPool.addTask(uplinker);
    threadPool.addTask(uplinker5g);

    tpool = &threadPool;
    signal(SIGINT, ctrl_c_handler);

    dnCnt = 0;
    sleep(5);
    while (1) {
        //std::cout << "there are still " << threadPool.size() << " tasks need to process" << std::endl;
        if (threadPool.size() == 0) {
            threadPool.stop();
            zout << "Now I will exit from main" << std::endl;
            exit(0);
        }
        sleep(2);

        // 检查网络联通情况
        std::string pingCmd = "ping -I " + sysrt->getN5G() + " -c 4 " + sysrt->getNDET();
        int result = system(pingCmd.c_str());
        if (result != 0) {
            zout << "Failed to connect." << std::endl;
            dnCnt++;
            if (dnCnt >= 30) {
                zout << "network disconnected for 1 min, !!!!!! SYSTEM REBOOT !!!!!!" << std::endl;
                sleep(2);
                system("sudo reboot");
            }
        } else {
            dnCnt = 0;
        }
    }

    return 0;
}
