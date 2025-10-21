#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <asm/barrier.h>
#include <syslog.h>
#include <stdatomic.h>
#include "queue.h"

#define FUSED_SIZE 64

pthread_rwlock_t queue_rwlock = PTHREAD_RWLOCK_INITIALIZER;
pthread_mutex_t recursive_mutex;
pthread_mutexattr_t recursive_attr;

pthread_mutex_t raw_queue_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t raw_queue_cond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t processed_queue_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t processed_queue_cond = PTHREAD_COND_INITIALIZER;

extern pthread_barrier_t startup_barrier;

atomic_int raw_queue_size = ATOMIC_VAR_INIT(0);
atomic_int processed_queue_size = ATOMIC_VAR_INIT(0);

void init_processing() {
    pthread_mutexattr_init(&recursive_attr);
    pthread_mutexattr_settype(&recursive_attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&recursive_mutex, &recursive_attr);
}

void publish_to_raw_queue(char *data, size_t len, int type) {
    struct data_msg *msg = malloc(sizeof(struct data_msg));
    if (!msg) {
        syslog(LOG_ERR, "malloc failed in raw_queue");
        return;
    }

    msg->type = type;
    msg->len = len;
    memcpy(msg->data, data, len);
    msg->next = NULL;

    pthread_rwlock_wrlock(&queue_rwlock); // Mutual exclusion
    pthread_mutex_lock(&raw_queue_mutex); // Lock ordering for deadlock prevention
    if (!raw_tail) {
        raw_head = raw_tail = msg;
    } else {
        raw_tail->next = msg;
        raw_tail = msg;
    }
    atomic_fetch_add(&raw_queue_size, 1);
    pthread_cond_broadcast(&raw_queue_cond); // Handle spurious
    pthread_mutex_unlock(&raw_queue_mutex);
    pthread_rwlock_unlock(&queue_rwlock);
}

static void publish_to_processed_queue(char *data, size_t len) {
    struct data_msg *msg = malloc(sizeof(struct data_msg));
    if (!msg) {
        syslog(LOG_ERR, "malloc failed in processed_queue");
        return;
    }

    msg->type = FUSED_DATA;
    msg->len = len;
    memcpy(msg->data, data, len);
    msg->next = NULL;

    pthread_rwlock_wrlock(&queue_rwlock);
    pthread_mutex_lock(&processed_queue_mutex);
    if (!processed_tail) {
        processed_head = processed_tail = msg;
    } else {
        processed_tail->next = msg;
        processed_tail = msg;
    }
    atomic_fetch_add(&processed_queue_size, 1);
    pthread_cond_broadcast(&processed_queue_cond);
    pthread_mutex_unlock(&processed_queue_mutex);
    pthread_rwlock_unlock(&queue_rwlock);
}

void *processing_thread(void *arg) {
    char fused_buffer[FUSED_SIZE];
    char mpu_data[20], bme_data[12], ens_data[16];
    int have_mpu = 0, have_bme = 0, have_ens = 0;

    pthread_barrier_wait(&startup_barrier);

    pthread_setcanceltype(PTHREAD_CANCEL_ASYNC, NULL); // Async for quick cancel if needed

    while (1) {
        if (atomic_load(&raw_queue_size) > 100) {
            syslog(LOG_WARNING, "Queue size > 100, skipping processing (circuit breaker)");
            sleep(1); // Throttle
            continue;
        }

        pthread_rwlock_rdlock(&queue_rwlock); // Reader-writer monitor
        pthread_mutex_lock(&raw_queue_mutex);
        while (!raw_head) {
            pthread_cond_wait(&raw_queue_cond, &raw_queue_mutex);
        }

        struct data_msg *msg = raw_head;
        raw_head = msg->next;
        if (!raw_head) raw_tail = NULL;

        switch (msg->type) {
            case MPU_DATA:
                if (msg->len > sizeof(mpu_data)) {
                    free(msg);
                    pthread_mutex_unlock(&raw_queue_mutex);
                    pthread_rwlock_unlock(&queue_rwlock);
                    continue;
                }
                memcpy(mpu_data, msg->data, msg->len);
                have_mpu = 1;
                break;
            case BME_DATA:
                if (msg->len > sizeof(bme_data)) {
                    free(msg);
                    pthread_mutex_unlock(&raw_queue_mutex);
                    pthread_rwlock_unlock(&queue_rwlock);
                    continue;
                }
                memcpy(bme_data, msg->data, msg->len);
                have_bme = 1;
                break;
            case ENS_DATA:
                if (msg->len > sizeof(ens_data)) {
                    free(msg);
                    pthread_mutex_unlock(&raw_queue_mutex);
                    pthread_rwlock_unlock(&queue_rwlock);
                    continue;
                }
                memcpy(ens_data, msg->data, msg->len);
                have_ens = 1;
                break;
        }

        atomic_fetch_sub(&raw_queue_size, 1);
        free(msg);
        pthread_mutex_unlock(&raw_queue_mutex);
        pthread_rwlock_unlock(&queue_rwlock);

        if (have_mpu && have_bme && have_ens) {
            // Fusion: e.g., temp average from BME and ENS AHT
            float bme_temp = *((s32 *)bme_data);
            float ens_temp = *((float *)(ens_data + 5)); // Adjust offset
            float fused_temp = (bme_temp + ens_temp) / 2.0;
            // Similar for others, concat or filter (comment: Có thể thêm Kalman filter cho data fusion V.18)
            // Simple Kalman filter cho temp fusion (từ web search example)
            static float kalman_state = 0.0; // State estimate
            static float kalman_uncertainty = 1.0; // Estimate uncertainty
            float process_noise = 0.01;
            float sensor_noise = 0.1;
            float kalman_gain = kalman_uncertainty / (kalman_uncertainty + sensor_noise);
            kalman_state = kalman_state + kalman_gain * (fused_temp - kalman_state);
            kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty + process_noise;
            fused_temp = kalman_state; // Use filtered temp
            // ... fill fused_buffer with fused_temp and others
            memcpy(fused_buffer, &fused_temp, sizeof(float));
            // ... fill fused_buffer

            __sync_synchronize(); // Reordering prevention

            publish_to_processed_queue(fused_buffer, FUSED_SIZE);
            have_mpu = have_bme = have_ens = 0;
        }
        pthread_testcancel();
    }
    return NULL;
}