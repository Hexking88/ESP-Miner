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

// Vaste extranonce2: 4 bytes, allemaal nul (8 hex tekens)
#define FIXED_EXTRANONCE2 "00000000"

// Version rolling UIT: laat construct_bm_job de midstate volledig
// opnieuw berekenen op basis van current_version.
#define DISABLE_VERSION_ROLLING 1

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

        // Voorkom negatieve timeout (busy-loop)
        if (timeout_ms < 0) {
            timeout_ms = 0;
        }

        TickType_t wait_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : 0;
        BaseType_t notified = xTaskNotifyWait(0, ULONG_MAX, &slot_notify, wait_ticks);

        int elapsed_ms = (int)((esp_timer_get_time() - start_time) / 1000);
        timeout_ms -= elapsed_ms;
        if (timeout_ms < 0) {
            timeout_ms = 0;
        }

        if (notified == pdTRUE) {
            miner_job_t *new_work = miner_job_get_slot((size_t)slot_notify);
            if (new_work == NULL) {
                ESP_LOGE(TAG, "miner_job_get_slot(%lu) gaf NULL, sla over",
                         (unsigned long)slot_notify);
                continue;
            }

            ESP_LOGI(TAG, "New Work Activated (slot %lu) %s (type %d) clean_jobs=%d",
                     (unsigned long)slot_notify,
                     new_work->job_id,
                     new_work->type,
                     new_work->clean_jobs);

            current_work = new_work;
            GLOBAL_STATE->active_job_slot_idx = (uint8_t)(slot_notify % MINER_JOB_POOL_SIZE);
            current_work_sent = false;

            // Initialiseer versie vanuit de nieuwe job
            current_version = new_work->version;

#if DISABLE_VERSION_ROLLING
            // Rolling uit: zet mask op 0, zodat de ASIC geen version bits rolt.
            if (current_version_mask != 0 && GLOBAL_STATE->ASIC_initalized) {
                ESP_LOGI(TAG, "Version rolling UIT (mask=0)");
                ASIC_set_version_mask(GLOBAL_STATE, 0);
                current_version_mask = 0;
            }
#else
            if (new_work->version_mask != current_version_mask && GLOBAL_STATE->ASIC_initalized) {
                ESP_LOGI(TAG, "Set chip version rolls %i", (int)(new_work->version_mask >> 13));
                ASIC_set_version_mask(GLOBAL_STATE, new_work->version_mask);
                current_version_mask = new_work->version_mask;
            }
#endif

            if (!current_work->clean_jobs) {
                ESP_LOGW(TAG, "clean_jobs=false, job %s wordt alsnog verzonden",
                         current_work->job_id);
                // GEEN continue: job wordt alsnog naar de ASIC gestuurd
            }
        } else {
            if (current_work == NULL) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        }

        bm_job *next_job = calloc(1, sizeof(bm_job));
        if (next_job == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for new job");
            continue;
        }

#if DISABLE_VERSION_ROLLING
        uint32_t version_mask = 0;
#else
        uint32_t version_mask = current_work->version_mask;
#endif
        double job_diff = current_work->pool_diff;
        uint8_t merkle_root[32];

        memcpy(merkle_root, current_work->merkle_root, 32);

        // Coinbase EERST decoderen, zodat merkle_root klopt voordat de ASIC hasht.
        if (!current_work_sent) {
            SYSTEM_decode_and_apply_coinbase(GLOBAL_STATE, current_work);
            current_work_sent = true;
            // Opnieuw kopiëren: decode kan merkle_root hebben bijgewerkt.
            memcpy(merkle_root, current_work->merkle_root, 32);
        }

        construct_bm_job_from_miner_job(current_work,
                                        current_version,
                                        merkle_root,
                                        version_mask,
                                        job_diff,
                                        GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates,
                                        next_job);

        // LET OP: geen NULL-check op current_work->job_id.
        // Als job_id een array is (char[]), geeft dat -Werror=address op GCC 12+.
        next_job->jobid = strdup(current_work->job_id);
        // Vaste 4-byte extranonce2 (8 hex tekens)
        next_job->extranonce2 = strdup(FIXED_EXTRANONCE2);

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

        // Log merkle root + ntime zodat je kunt vergelijken met de pool
        ESP_LOGI(TAG, "merkle_root[0..3]=%02x%02x%02x%02x",
                 merkle_root[0], merkle_root[1], merkle_root[2], merkle_root[3]);

        // Timing rond ASIC_send_work, zodat je ziet waar de vertraging zit
        uint64_t t0 = esp_timer_get_time();
        ASIC_send_work(GLOBAL_STATE, next_job);
        uint64_t t1 = esp_timer_get_time();

        ESP_LOGI(TAG,
                 "Job verzonden: %s version=%08" PRIx32 " en2=%s (ASIC_send_work duurde %llu us)",
                 next_job->jobid,
                 current_version,
                 next_job->extranonce2,
                 (unsigned long long)(t1 - t0));

#if DISABLE_VERSION_ROLLING
        // Geen rolling: alleen een kleine vaste stap om unieke versies te krijgen.
        // De ASIC berekent de midstate opnieuw per job.
        current_version = increment_bitmask(current_version, 0x00002000);
#else
        // Version rolling (geen extranonce2 ophoging)
        uint32_t mask = (current_work->version_mask != 0)
                            ? current_work->version_mask
                            : BIP320_VERSION_ROLLING_MASK;
        uint8_t midstates = GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates;

        if (midstates > 0) {
            for (int i = 0; i < midstates; i++) {
                current_version = increment_bitmask(current_version, mask);
            }
        } else {
            current_version = increment_bitmask(current_version, mask);
        }
#endif

        timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    }
}
