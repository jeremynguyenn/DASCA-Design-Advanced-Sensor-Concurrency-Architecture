#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <semaphore.h>
#include <syslog.h>
#include "processing.h"
#include "queue.h"

#define MPU_DEV "/dev/mpu9250"
#define BME_DEV "/dev/bme280"
#define ENS_DEV "/dev/ens160"
#define MAX_EVENTS 10
#define QUEUE_MAX 100 // Threshold cho circuit breaker

extern pthread_barrier_t startup_barrier;

sem_t bounded_sem; // Bounded waiting FIFO sem

__thread char tls_buffer[64]; // TLS

void data_callback(char *data, size_t len, int type) { // Callback
    publish_to_raw_queue(data, len, type);
}

void *sensor_pool_worker(void *arg) {
    pthread_barrier_wait(&startup_barrier);
    while (1) {
        sem_wait(&bounded_sem);
        // Process task from queue or read if signaled
        // Use atomic ops for lock-free if possible (RCU comment: for lock-free reads)
    }
    return NULL;
}

// Define cleanup_handler (mention trong user_main.c)
static void cleanup_handler(void *arg) {
    // Close fds, free resources
}

void *sensors_thread(void *arg) {
    int epfd = epoll_create1(0);
    struct epoll_event ev, events[MAX_EVENTS];
    int fds[3];
    int ret;

    fds[0] = open(MPU_DEV, O_RDONLY | O_NONBLOCK);
    if (fds[0] < 0) {
        syslog(LOG_ERR, "open MPU failed");
        return NULL;
    }
    fds[1] = open(BME_DEV, O_RDONLY | O_NONBLOCK);
    if (fds[1] < 0) {
        syslog(LOG_ERR, "open BME failed");
        close(fds[0]);
        return NULL;
    }
    fds[2] = open(ENS_DEV, O_RDONLY | O_NONBLOCK);
    if (fds[2] < 0) {
        syslog(LOG_ERR, "open ENS failed");
        close(fds[0]); close(fds[1]);
        return NULL;
    }

    pthread_cleanup_push(cleanup_handler, NULL); // Cancellation cleanup

    for (int i = 0; i < 3; i++) {
        ev.events = EPOLLIN | EPOLLET;
        ev.data.fd = fds[i];
        ret = epoll_ctl(epfd, EPOLL_CTL_ADD, fds[i], &ev);
        if (ret < 0) syslog(LOG_ERR, "epoll_ctl failed for fd %d", fds[i]);
    }

    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    struct itimerspec ts = { {0, 100000000}, {0, 100000000} }; // 10Hz
    timerfd_settime(tfd, 0, &ts, NULL);
    ev.data.fd = tfd;
    ev.events = EPOLLIN;
    ret = epoll_ctl(epfd, EPOLL_CTL_ADD, tfd, &ev);
    if (ret < 0) syslog(LOG_ERR, "epoll_ctl timer failed");

    pthread_barrier_wait(&startup_barrier);

    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL); // Deferred cancellation

    while (1) {
        int nfds = epoll_wait(epfd, events, MAX_EVENTS, -1);
        if (nfds < 0) syslog(LOG_ERR, "epoll_wait failed");
        for (int n = 0; n < nfds; n++) {
            if (events[n].data.fd == tfd) {
                uint64_t exp;
                read(tfd, &exp, sizeof(exp)); // Clear timer
                sem_post(&bounded_sem); // Signal pool
            } else {
                // Circuit breaker cho rate limit (V.20)
                if (atomic_load(&raw_queue_size) > QUEUE_MAX) {
                    syslog(LOG_WARNING, "Queue full, skipping read (circuit breaker)");
                    continue;
                }
                ssize_t len = read(events[n].data.fd, tls_buffer, 64);
                if (len > 0) {
                    int type = (events[n].data.fd == fds[0]) ? MPU_DATA : (events[n].data.fd == fds[1]) ? BME_DATA : ENS_DATA;
                    data_callback(tls_buffer, len, type);
                } else if (len < 0) {
                    syslog(LOG_ERR, "read failed for fd %d", events[n].data.fd);
                }
                pthread_testcancel(); // Cancellation point in I/O
            }
        }
    }

    pthread_cleanup_pop(1);
    close(epfd);
    close(tfd);
    close(fds[0]);
    close(fds[1]);
    close(fds[2]);
    return NULL;
}