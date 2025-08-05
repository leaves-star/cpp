#ifndef __COMMON_TASK_H__
#define __COMMON_TASK_H__

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <mqueue.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctime>
#include <sys/stat.h>
#include <sys/time.h>

#define zout            std::cout << CommonTask::getCurTimeStr() << "-> "
#define MSG_MAX_SIZE    16

namespace hnc10 {
    enum TaskState {
        TASK_INITED = 0,
        TASK_RUNNING,
        TASK_PAUSED,
        TASK_FINISHED
    };

    typedef struct {
        long mtype;
        char mtext[MSG_MAX_SIZE];
    } common_msg_t;

    class CommonTask {
    public:
        CommonTask(void* arg = NULL, const std::string taskName = "") : arg_(arg), taskName_(taskName) {
            bExit = false;
            bPaused = false;
            taskState = TASK_INITED;
        }

        virtual ~CommonTask() {
        }
        
        void forceExit() {
            bExit = true;
        }

        TaskState getState() {
            return taskState;
        }

        void pause() {
            bPaused = true;
        }

        void go_on() {
            bPaused = false;
        }

        int wait(int timeoutms=2000) {
            int ms = 0;
            while (taskState != TASK_FINISHED && ms < timeoutms) {
                usleep(20*1000);
                ms += 20;
            }
            return (taskState == TASK_FINISHED);
        }

        void setArg(void *arg) {
            arg_ = arg;
        }

        static std::string getCurTimeStr() {
            std::string ret;

            time_t timep;
            struct tm *p;
            time(&timep);
            p = localtime(&timep);
            std::ostringstream buffer;
            buffer << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec;
            ret = buffer.str();
            return ret;
        }

        mqd_t create_queue(const char *name) {
            mqd_t mq;
            struct mq_attr attr;

            memset(&attr, 0, sizeof(attr));
            attr.mq_maxmsg = 32;
            attr.mq_msgsize = MSG_MAX_SIZE;

            mq = mq_open(name, O_CREAT | O_RDWR | O_EXCL, 0666, &attr);
            if ((int)mq < 0) {
                mq_unlink(name);
                mq = mq_open(name, O_RDWR | O_CREAT, 0666, &attr);
            }
            return mq;
        }

        int send_msg(mqd_t mq, const char *msg) {
            if (mq_send(mq, msg, strlen(msg) + 1, 1) == -1) {
                return -1;
            }
            return 0;
        }

        int receive_msg(mqd_t mq, char *p, int &n) {
            if (mq_receive(mq, p, n, NULL) > 0) {
                return 0;
            } else {
                return -1;
            }
        }

        virtual void run() = 0;

    protected:
        void *arg_;
        std::string taskName_;
        bool bExit;
        bool bPaused;
        TaskState taskState;
    };
}

#endif // __COMMON_TASK_H__
