#include <sys/time.h>
#include <limits.h>
#include <inttypes.h>

#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mining.h"
#include "miner_job.h"
#include "string.h"
#include "esp_timer.h"

#include "asic.h"
#include "system.h"
#include "esp_heap_caps.h"
#include "utils.h"

static const char *TAG = "create_jobs_task";

void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    uint32_t current_version_mask = 0;
    miner_job_t *current_work = NULL;
    bool current_work_sent = false;
    uint32_t current_version = 0;
    int timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);

    ESP_LOGI(TAG, "ASIC Job Interval: %d ms", timeout_ms);
    ESP_LOGI(TAG, "ASIC Ready!");

    while (1) {
        uint64_t start_time = esp_timer_get_time();
        uint32_t slot_notify = 0;
        TickType_t wait_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : 0;
        BaseType_t notified = xTaskNotifyWait(0, ULONG_MAX, &slot_notify, wait_ticks);
        timeout_ms -= (esp_timer_get_time() - start_time) / 1000;

        if (notified == pdTRUE) {
            miner_job_t *new_work = miner_job_get_slot((size_t)slot_notify);
            ESP_LOGI(TAG, "New Work Activated (slot %lu) %s (type %d)", (unsigned long)slot_notify, new_work->job_id, new_work->type);
            current_work = new_work;
            GLOBAL_STATE->active_job_slot_idx = (uint8_t)(slot_notify % MINER_JOB_POOL_SIZE);
            current_work_sent = false;
            
            // Initialiseer de versie vanuit de nieuwe job
            current_version = new_work->version;

            if (new_work->version_mask != current_version_mask && GLOBAL_STATE->ASIC_initalized) {
                ESP_LOGI(TAG, "Set chip version rolls %i", (int)(new_work->version_mask >> 13));
                ASIC_set_version_mask(GLOBAL_STATE, new_work->version_mask);
                current_version_mask = new_work->version_mask;
            }

            if (!current_work->clean_jobs) {
                continue;
            }
        } else {
            if (current_work == NULL) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
        }

        bm_job *next_job = malloc(sizeof(bm_job));
        if (next_job == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for new job");
            continue;
        }

        uint32_t version_mask = current_work->version_mask;
        double job_diff = current_work->pool_diff;
        uint8_t merkle_root[32];

        memcpy(merkle_root, current_work->merkle_root, 32);

        construct_bm_job_from_miner_job(current_work, current_version, merkle_root, version_mask, job_diff, GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates, next_job);
        next_job->jobid = strdup(current_work->job_id);
        next_job->extranonce2 = strdup(""); 

        if (next_job->jobid == NULL || next_job->extranonce2 == NULL) {
            ESP_LOGE(TAG, "Failed to allocate job metadata");
            free(next_job->jobid);
            free(next_job->extranonce2);
            free(next_job);
            continue;
        }

        if (!GLOBAL_STATE->ASIC_initalized) {
            ESP_LOGW(TAG, "ASIC not initialized, skipping job send");
            free(next_job->jobid);
            free(next_job->extranonce2);
            free(next_job);
            continue;
        }

        ASIC_send_work(GLOBAL_STATE, next_job);

        if (!current_work_sent) {
            SYSTEM_decode_and_apply_coinbase(GLOBAL_STATE, current_work);
        }
        current_work_sent = true;

        // Version Rolling logica (geen extranonce2 ophoging meer)
        uint32_t mask = (current_work->version_mask != 0) ? current_work->version_mask : BIP320_VERSION_ROLLING_MASK;
        uint8_t midstates = GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates;
        
        if (midstates > 0) {
            for (int i = 0; i < midstates; i++) {
                current_version = increment_bitmask(current_version, mask);
            }
        } else {
            current_version = increment_bitmask(current_version, mask);
        }

        timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    }
}
