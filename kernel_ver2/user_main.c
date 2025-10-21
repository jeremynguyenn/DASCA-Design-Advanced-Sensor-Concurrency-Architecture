#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <syslog.h>
#include "sensors.h"
#include "processing.h"
#include "ipc.h"
#include "queue.h"

#define NUM_SENSORS 3
#define NUM_POOL_WORKERS 4 // Thread pool size

static pthread_t sensors_thread_id;
static pthread_t proc_thread, ipc_thread, watchdog_thread;
static pthread_t pool_workers[NUM_POOL_WORKERS];
static pthread_barrier_t startup_barrier;

static void cleanup_handler(void *arg) {
    // Close fds, free
}

static void signal_handler(int sig, siginfo_t *info, void *context) {
    // Signal-safe: no malloc/printf
    int i;
    pthread_cancel(sensors_thread_id);
    for (i = 0; i < NUM_POOL_WORKERS; i++) pthread_cancel(pool_workers[i]);
    pthread_cancel(proc_thread);
    pthread_cancel(ipc_thread);
    pthread_cancel(watchdog_thread);
    _exit(0);
}

static void *watchdog_thread_func(void *arg) {
    while (1) {
        sleep(10);
        // Check queues/atomic counters for stuck, raise(SIGALRM) if deadlock
        // Use wait-for graph stub for detection
        // Simple deadlock detection
        if (pthread_mutex_trylock(&raw_queue_mutex) == EBUSY) {
            syslog(LOG_ERR, "Potential deadlock detected in raw_queue");
            raise(SIGALRM);
        } else {
            pthread_mutex_unlock(&raw_queue_mutex);
        }
        if (pthread_mutex_trylock(&processed_queue_mutex) == EBUSY) {
            syslog(LOG_ERR, "Potential deadlock detected in processed_queue");
            raise(SIGALRM);
        } else {
            pthread_mutex_unlock(&processed_queue_mutex);
        }
    }
    return NULL;
}

int main() {
    struct sigaction sa;
    struct sched_param param;
    cpu_set_t cpuset;
    pid_t pid;
    int i, ret;

    openlog("sensor_app", LOG_PID, LOG_DAEMON);

    sa.sa_sigaction = signal_handler;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGALRM, &sa, NULL); // For watchdog

    // Daemonize
    pid = fork();
    if (pid > 0) exit(0);
    if (pid < 0) exit(1);
    setsid();
    pid = fork();
    if (pid > 0) exit(0);
    umask(0);
    chdir("/");
    close(0); close(1); close(2);
    open("/dev/null", O_RDWR); dup(0); dup(0);

    // Zombie prevention: waitpid in loop if children
    while (waitpid(-1, NULL, WNOHANG) > 0);

    // Cgroup: echo $$ > /sys/fs/cgroup/unified/tasks (comment for resource limit)

    init_queues();

    pthread_barrier_init(&startup_barrier, NULL, NUM_POOL_WORKERS + 3); // Pool + sensors + proc + ipc

    pthread_attr_t attr_detached, attr_joinable;
    pthread_attr_init(&attr_detached);
    pthread_attr_setdetachstate(&attr_detached, PTHREAD_CREATE_DETACHED);

    pthread_attr_init(&attr_joinable);
    pthread_attr_setdetachstate(&attr_joinable, PTHREAD_CREATE_JOINABLE);

    // Affinity for main core 0
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);

    // Sensors thread (detached, core 1)
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    pthread_attr_setaffinity_np(&attr_detached, sizeof(cpuset), &cpuset);
    ret = pthread_create(&sensors_thread_id, &attr_detached, sensors_thread, NULL);
    if (ret) syslog(LOG_ERR, "pthread_create sensors failed: %d", ret);

    // Thread pool workers (detached, core 1)
    for (i = 0; i < NUM_POOL_WORKERS; i++) {
        ret = pthread_create(&pool_workers[i], &attr_detached, sensor_pool_worker, NULL);
        if (ret) syslog(LOG_ERR, "pthread_create pool %d failed: %d", i, ret);
    }

    // Processing (joinable, SCHED_FIFO, priority 50, core 2)
    param.sched_priority = 50;
    pthread_attr_setschedpolicy(&attr_joinable, SCHED_FIFO);
    pthread_attr_setschedparam(&attr_joinable, &param);
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    pthread_attr_setaffinity_np(&attr_joinable, sizeof(cpuset), &cpuset);
    ret = pthread_create(&proc_thread, &attr_joinable, processing_thread, NULL);
    if (ret) syslog(LOG_ERR, "pthread_create processing failed: %d", ret);

    // IPC (detached, SCHED_RR, core 3)
    param.sched_priority = 40;
    pthread_attr_setschedpolicy(&attr_detached, SCHED_RR);
    pthread_attr_setschedparam(&attr_detached, &param);
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_attr_setaffinity_np(&attr_detached, sizeof(cpuset), &cpuset);
    ret = pthread_create(&ipc_thread, &attr_detached, ipc_thread, NULL);
    if (ret) syslog(LOG_ERR, "pthread_create ipc failed: %d", ret);

    // Watchdog (detached, core 0)
    ret = pthread_create(&watchdog_thread, &attr_detached, watchdog_thread_func, NULL);
    if (ret) syslog(LOG_ERR, "pthread_create watchdog failed: %d", ret);

    pthread_join(proc_thread, NULL);

    pthread_barrier_destroy(&startup_barrier);
    destroy_queues();
    closelog();
    return 0;
}