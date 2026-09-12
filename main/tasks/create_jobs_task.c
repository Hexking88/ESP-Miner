```c
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

#define MAX_EXTRANONCE2_LEN 32
#define MAX_EXTRANONCE2_STR (MAX_EXTRANONCE2_LEN * 2 + 1)

/*
 * Golden-ratio-related 64-bit constant.
 *
 * We use modulo 2^64 arithmetic through uint64_t overflow.
 * This changes the order in which extranonce2 values are visited.
 */
#define GOLDEN_RATIO_STEP 0x9E3779B97F4A7C15ULL


/*
 * Generate the next extranonce2 value.
 *
 * Sequence:
 *
 *   0
 *   11400714819323198485
 *   4354685564936845354
 *   15755400384260043839
 *   ...
 *
 * uint64_t overflow gives us modulo 2^64 behavior.
 */
static inline uint64_t next_extranonce2(uint64_t current)
{
    return current * GOLDEN_RATIO_STEP + 1ULL;
}


static void generate_work_from_miner_job(
    GlobalState *GLOBAL_STATE,
    const miner_job_t *job,
    uint64_t extranonce_2,
    uint32_t current_version)
{
    if (!job) {
        return;
    }

    bm_job *next_job = malloc(sizeof(bm_job));

    if (next_job == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for new job");
        return;
    }

    uint32_t version_mask = job->version_mask;
    double job_diff = job->pool_diff;

    uint8_t merkle_root[32];

    char extranonce_2_str[MAX_EXTRANONCE2_STR] = "";

    uint32_t effective_version = job->version;

    if (!GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling &&
        !miner_job_is_rollable(job)) {

        effective_version = current_version;
    }


    /*
     * SV2 standard job already contains the merkle root.
     */
    if (job->type == JOB_TYPE_SV2_STANDARD) {

        memcpy(
            merkle_root,
            job->merkle_root,
            32
        );

    } else {

        /*
         * Make sure extranonce2 fits in our local buffer.
         */
        size_t e2_len = job->extranonce2_len;

        if (e2_len > MAX_EXTRANONCE2_LEN) {

            ESP_LOGE(
                TAG,
                "extranonce_2_len %u exceeds maximum %d, skipping job",
                (unsigned)e2_len,
                MAX_EXTRANONCE2_LEN
            );

            free(next_job);
            return;
        }


        /*
         * Convert uint64_t extranonce2 into binary representation.
         */
        uint8_t extranonce_2_bin[MAX_EXTRANONCE2_LEN] = {0};

        size_t copy_len =
            (e2_len < sizeof(uint64_t))
                ? e2_len
                : sizeof(uint64_t);


        if (e2_len > 0) {

            memcpy(
                extranonce_2_bin,
                &extranonce_2,
                copy_len
            );

            bin2hex(
                extranonce_2_bin,
                e2_len,
                extranonce_2_str,
                sizeof(extranonce_2_str)
            );
        }


        /*
         * Calculate coinbase hash.
         */
        uint8_t coinbase_tx_hash[32];

        calculate_coinbase_tx_hash_bin(
            job->coinbase_prefix,
            job->coinbase_prefix_len,

            job->extranonce1,
            job->extranonce1_len,

            extranonce_2_bin,
            e2_len,

            job->coinbase_suffix,
            job->coinbase_suffix_len,

            coinbase_tx_hash
        );


        /*
         * Calculate merkle root.
         */
        calculate_merkle_root_hash(
            coinbase_tx_hash,

            (const uint8_t (*)[32])job->merkle_path,

            job->merkle_path_count,

            merkle_root
        );
    }


    /*
     * Build ASIC job.
     */
    construct_bm_job_from_miner_job(
        job,
        effective_version,
        merkle_root,
        version_mask,
        job_diff,
        GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates,
        next_job
    );


    /*
     * Store job metadata.
     */
    next_job->jobid = strdup(job->job_id);
    next_job->extranonce2 = strdup(extranonce_2_str);


    if (next_job->jobid == NULL ||
        next_job->extranonce2 == NULL) {

        ESP_LOGE(TAG, "Failed to allocate job metadata");

        free(next_job->jobid);
        free(next_job->extranonce2);
        free(next_job);

        return;
    }


    /*
     * ASIC must be initialized before sending work.
     */
    if (!GLOBAL_STATE->ASIC_initalized) {

        ESP_LOGW(
            TAG,
            "ASIC not initialized, skipping job send"
        );

        free(next_job->jobid);
        free(next_job->extranonce2);
        free(next_job);

        return;
    }


    /*
     * Send work to ASIC.
     */
    ASIC_send_work(
        GLOBAL_STATE,
        next_job
    );
}


void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE =
        (GlobalState *)pvParameters;


    /*
     * active_jobs / valid_jobs are allocated and zeroed by
     * SYSTEM_init_system(), before any task that touches them can run.
     */

    uint32_t current_version_mask = 0;

    miner_job_t *current_work = NULL;

    bool current_work_sent = false;

    /*
     * Current extranonce2.
     *
     * Starts at zero for every new clean job.
     */
    uint64_t extranonce_2 = 0;

    uint32_t current_version = 0;

    int timeout_ms =
        ASIC_get_asic_job_frequency_ms(
            GLOBAL_STATE
        );


    ESP_LOGI(
        TAG,
        "ASIC Job Interval: %d ms",
        timeout_ms
    );

    ESP_LOGI(
        TAG,
        "ASIC Ready!"
    );


    while (1) {

        uint64_t start_time =
            esp_timer_get_time();

        uint32_t slot_notify = 0;

        TickType_t wait_ticks =
            (timeout_ms > 0)
                ? pdMS_TO_TICKS(timeout_ms)
                : 0;


        BaseType_t notified =
            xTaskNotifyWait(
                0,
                ULONG_MAX,
                &slot_notify,
                wait_ticks
            );


        timeout_ms -=
            (esp_timer_get_time() - start_time) / 1000;


        /*
         * New miner job received.
         */
        if (notified == pdTRUE) {

            miner_job_t *new_work =
                miner_job_get_slot(
                    (size_t)slot_notify
                );


            ESP_LOGI(
                TAG,
                "New Work Activated (slot %lu) %s (type %d)",
                (unsigned long)slot_notify,
                new_work->job_id,
                new_work->type
            );


            current_work = new_work;


            GLOBAL_STATE->active_job_slot_idx =
                (uint8_t)(
                    slot_notify % MINER_JOB_POOL_SIZE
                );


            current_work_sent = false;

            current_version =
                new_work->version;


            /*
             * Update ASIC version rolling mask.
             */
            if (new_work->version_mask != current_version_mask &&
                GLOBAL_STATE->ASIC_initalized) {

                ESP_LOGI(
                    TAG,
                    "Set chip version rolls %i",
                    (int)(
                        new_work->version_mask >> 13
                    )
                );


                ASIC_set_version_mask(
                    GLOBAL_STATE,
                    new_work->version_mask
                );


                current_version_mask =
                    new_work->version_mask;
            }


            /*
             * IMPORTANT:
             *
             * Every new clean job starts a fresh
             * golden-ratio extranonce2 sequence.
             */
            extranonce_2 = 0;


            if (!new_work->clean_jobs) {

                /*
                 * Staged job for next cycle.
                 * Let current ASIC cycle finish.
                 */
                continue;
            }

        } else {

            /*
             * No current work yet.
             */
            if (current_work == NULL) {

                vTaskDelay(
                    100 / portTICK_PERIOD_MS
                );

                continue;
            }


            /*
             * For non-rollable jobs with hardware version
             * rolling, wait for the next job cycle.
             */
            if (!miner_job_is_rollable(current_work) &&
                current_work_sent &&
                GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling) {

                timeout_ms =
                    ASIC_get_asic_job_frequency_ms(
                        GLOBAL_STATE
                    );

                continue;
            }
        }


        /*
         * Generate and send ASIC work.
         */
        generate_work_from_miner_job(
            GLOBAL_STATE,
            current_work,
            extranonce_2,
            current_version
        );


        /*
         * Decode/apply coinbase only once per job.
         */
        if (!current_work_sent) {

            SYSTEM_decode_and_apply_coinbase(
                GLOBAL_STATE,
                current_work
            );
        }


        current_work_sent = true;


        /*
         * ============================================================
         * GOLDEN-RATIO EXTRANONCE2 STEPPING
         * ============================================================
         *
         * Instead of:
         *
         *     extranonce_2++;
         *
         * we jump through the uint64_t space using the
         * golden-ratio-related constant.
         */
        if (miner_job_is_rollable(current_work)) {

            extranonce_2 =
                next_extranonce2(
                    extranonce_2
                );


        } else if (
            !GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling
        ) {

            /*
             * Software version rolling for ASICs without
             * hardware version rolling (e.g. BM1397)
             * on SV2 Standard Channel.
             */

            uint32_t mask =
                (current_work->version_mask != 0)
                    ? current_work->version_mask
                    : BIP320_VERSION_ROLLING_MASK;


            uint8_t midstates =
                GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates;


            for (int i = 0; i < midstates; i++) {

                current_version =
                    increment_bitmask(
                        current_version,
                        mask
                    );
            }
        }


        /*
         * Reset ASIC job interval.
         */
        timeout_ms =
            ASIC_get_asic_job_frequency_ms(
                GLOBAL_STATE
            );
    }
}
```

**Let wel op één belangrijk punt:** deze code gebruikt de golden-ratio-gerelateerde multiplier als **64-bit modulo-2⁶⁴ sequence**. Dat is geschikt om de volgorde sterk te verspreiden, maar het betekent niet automatisch dat jouw specifieke `extranonce2_len` (bijvoorbeeld 4 bytes) het volledige bereik op dezelfde manier doorloopt.

Voor een echte test zou ik eerst de gegenereerde waarden loggen en controleren:

```c
ESP_LOGI(TAG, "extranonce2 = %" PRIu64, extranonce_2);
```

Dan kun je op de ESP zien welke sequence daadwerkelijk naar de ASIC wordt gestuurd.
