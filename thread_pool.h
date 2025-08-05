#ifndef __THREAD_POOL_H__
#define __THREAD_POOL_H__

#include <deque>  
#include <pthread.h>
#include <functional>
#include "common_task.h"

namespace hnc10
{
    class ThreadPool {
    public:
        ThreadPool(int threadNum = 10);
        ~ThreadPool();

    public:
        size_t addTask(CommonTask *task);
        void stop();
        int size();
        CommonTask *take();

    private:
        int createThreads();
        static void* threadFunc(void * threadData);

    private:
        ThreadPool& operator=(const ThreadPool&);
        ThreadPool(const ThreadPool&);

    private:
        volatile bool isRunning_;
        int threadsNum_;
        pthread_t *threads_;
        std::deque<CommonTask *> taskQueue_;
        pthread_mutex_t mutex_;
        pthread_cond_t condition_;
    };
}

#endif // __THREAD_POOL_H__
