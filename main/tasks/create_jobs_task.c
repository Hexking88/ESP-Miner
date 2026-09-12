#include <sys/time.h>
#include <limits.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "work_queue.h"
#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mining.h"
#include "esp_timer.h"

#include "asic.h"
#include "system.h"
#include "esp_heap_caps.h"
#include "sv2_protocol.h"
#include "stratum_api.h"
#include "stratum_v2_task.h"
#include "utils.h"

static const char *TAG = "create_jobs_task";

#define MAX_EXTRANONCE2_LEN 32
#define MAX_EXTRANONCE2_STR (MAX_EXTRANONCE2_LEN * 2 + 1)

static bool generate_work(GlobalState *GLOBAL_STATE,
                          mining_notify *notification,
                          double difficulty);

static bool generate_work_sv2(GlobalState *GLOBAL_STATE,
                              sv2_job_t *job,
                              double difficulty);

static bool generate_work_sv2_ext(GlobalState *GLOBAL_STATE,
                                  sv2_ext_job_t *job,
                                  double difficulty);


/*
 * Free a work item using the correct free function for the
 * protocol it was created under.
 */
static void free_work_item(GlobalState *GLOBAL_STATE,
                           void *work,
                           stratum_protocol_t protocol)
{
    if (!work) {
        return;
    }

    if (protocol == STRATUM_PROTOCOL_V2) {

        if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {
            sv2_ext_job_free((sv2_ext_job_t *)work);
        } else {
            /*
             * sv2_job_t is a flat allocation.
             */
            free(work);
        }

    } else {
        STRATUM_V1_free_mining_notify(work);
    }
}


void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    double difficulty = GLOBAL_STATE->pool_difficulty;

    void *current_work = NULL;

    stratum_protocol_t current_work_protocol =
        GLOBAL_STATE->stratum_protocol;

    int timeout_ms =
        ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);

    /*
     * Last job actually dispatched to the ASIC.
     *
     * IMPORTANT:
     * These are updated only after generate_work*() successfully
     * created and sent the job to the ASIC.
     */
    static char last_dispatched_job_v1[64] = {0};
    static uint32_t last_dispatched_job_sv2 = UINT32_MAX;

    ESP_LOGI(TAG,
             "ASIC Job Interval: %d ms",
             timeout_ms);

    ESP_LOGI(TAG,
             "ASIC Ready! (Zero-Extranonce2 + BIP320 + Auto-Job-Update)");

    while (1) {

        /*
         * Clear extranonce2 reset request.
         */
        if (GLOBAL_STATE->reset_extranonce2) {
            GLOBAL_STATE->reset_extranonce2 = false;
        }


        /*
         * Detect protocol changes before dequeuing work.
         */
        stratum_protocol_t active_protocol =
            GLOBAL_STATE->stratum_protocol;

        if (active_protocol != current_work_protocol) {

            if (current_work != NULL) {

                ESP_LOGI(
                    TAG,
                    "Protocol switched from %s to %s, discarding current work",
                    current_work_protocol == STRATUM_PROTOCOL_V2
                        ? STRATUM_V2
                        : STRATUM_V1,
                    active_protocol == STRATUM_PROTOCOL_V2
                        ? STRATUM_V2
                        : STRATUM_V1
                );

                free_work_item(
                    GLOBAL_STATE,
                    current_work,
                    current_work_protocol
                );

                current_work = NULL;
            }

            current_work_protocol = active_protocol;

            /*
             * A protocol change invalidates the previous dispatch state.
             */
            last_dispatched_job_v1[0] = '\0';
            last_dispatched_job_sv2 = UINT32_MAX;

            /*
             * Re-read the ASIC interval after protocol changes.
             */
            timeout_ms =
                ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
        }


        /*
         * Wait for new stratum work.
         */
        uint64_t start_time = esp_timer_get_time();

        void *new_work =
            queue_dequeue_timeout(
                &GLOBAL_STATE->stratum_queue,
                timeout_ms
            );

        int elapsed_ms =
            (int)((esp_timer_get_time() - start_time) / 1000);

        timeout_ms -= elapsed_ms;

        if (timeout_ms < 0) {
            timeout_ms = 0;
        }


        /*
         * A new work item arrived.
         */
        if (new_work != NULL) {

            active_protocol =
                GLOBAL_STATE->stratum_protocol;


            /*
             * The currently held work is no longer needed.
             */
            if (current_work != NULL) {

                free_work_item(
                    GLOBAL_STATE,
                    current_work,
                    current_work_protocol
                );

                current_work = NULL;
            }


            /*
             * Protocol may have changed while queue_dequeue_timeout()
             * was waiting.
             *
             * Unfortunately the queue item itself does not carry protocol
             * metadata, so only discard it here if the protocol changed.
             *
             * For normal operation this is safe because the item was
             * allocated by the producer and a raw free is appropriate for
             * the stale dequeue path only when protocol ownership cannot
             * be resolved.
             *
             * Prefer preventing this race at the producer/queue layer.
             */
            if (active_protocol != current_work_protocol) {

                ESP_LOGW(
                    TAG,
                    "Protocol switch detected during dequeue, discarding stale item"
                );

                /*
                 * Do not attempt protocol-specific cleanup here because
                 * the queue item does not expose its originating protocol.
                 *
                 * Both V1 mining_notify and SV2 flat jobs use heap ownership
                 * for the top-level allocation. Extended SV2 should normally
                 * not be left in the queue during a protocol switch.
                 */
                free(new_work);

                current_work_protocol = active_protocol;

                last_dispatched_job_v1[0] = '\0';
                last_dispatched_job_sv2 = UINT32_MAX;

                timeout_ms =
                    ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);

                continue;
            }


            current_work = new_work;


            /*
             * Job state.
             */
            bool is_new_job_id = false;
            bool clean = false;
            bool difficulty_changed = false;


            /*
             * Determine job ID and clean_jobs state.
             */
            if (current_work_protocol == STRATUM_PROTOCOL_V2) {

                if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {

                    sv2_ext_job_t *j =
                        (sv2_ext_job_t *)current_work;

                    ESP_LOGI(
                        TAG,
                        "New Work Dequeued SV2 ext job %" PRIu32
                        " (clean: %s)",
                        j->job_id,
                        j->clean_jobs ? "true" : "false"
                    );

                    clean = j->clean_jobs;

                    if (last_dispatched_job_sv2 != j->job_id) {
                        is_new_job_id = true;
                    }

                } else {

                    sv2_job_t *j =
                        (sv2_job_t *)current_work;

                    ESP_LOGI(
                        TAG,
                        "New Work Dequeued SV2 job %" PRIu32
                        " (clean: %s)",
                        j->job_id,
                        j->clean_jobs ? "true" : "false"
                    );

                    clean = j->clean_jobs;

                    if (last_dispatched_job_sv2 != j->job_id) {
                        is_new_job_id = true;
                    }
                }

            } else {

                mining_notify *j =
                    (mining_notify *)current_work;

                ESP_LOGI(
                    TAG,
                    "New Work Dequeued %s (clean: %s)",
                    j->job_id,
                    j->clean_jobs ? "true" : "false"
                );

                clean = j->clean_jobs;

                if (strcmp(
                        last_dispatched_job_v1,
                        j->job_id
                    ) != 0) {

                    is_new_job_id = true;
                }
            }


            /*
             * Difficulty update.
             *
             * IMPORTANT:
             *
             * Even if the job ID has not changed, a new difficulty means
             * the ASIC must receive a freshly constructed job so that the
             * new target/pool difficulty is applied.
             */
            if (GLOBAL_STATE->new_set_mining_difficulty_msg) {

                ESP_LOGI(
                    TAG,
                    "New pool difficulty %.2f",
                    GLOBAL_STATE->pool_difficulty
                );

                difficulty =
                    GLOBAL_STATE->pool_difficulty;

                GLOBAL_STATE->new_set_mining_difficulty_msg =
                    false;

                difficulty_changed = true;
            }


            /*
             * Version rolling mask update.
             */
            if (GLOBAL_STATE->new_stratum_version_rolling_msg &&
                GLOBAL_STATE->ASIC_initalized) {

                ESP_LOGI(
                    TAG,
                    "Set chip version rolls %i",
                    (int)(GLOBAL_STATE->version_mask >> 13)
                );

                ASIC_set_version_mask(
                    GLOBAL_STATE,
                    GLOBAL_STATE->version_mask
                );

                GLOBAL_STATE->new_stratum_version_rolling_msg =
                    false;
            }


            /*
             * Send the work when:
             *
             * 1. It is a new job ID
             * 2. clean_jobs is true
             * 3. Difficulty changed
             *
             * Same job + clean=false + no difficulty change:
             * do not resend.
             */
            if (!is_new_job_id &&
                !clean &&
                !difficulty_changed) {

                continue;
            }


        } else {

            /*
             * Queue timeout.
             *
             * Do not repeatedly resend the same job.
             * ASIC continues its own nonce/version rolling search.
             */
            if (current_work == NULL) {

                vTaskDelay(
                    100 / portTICK_PERIOD_MS
                );

                timeout_ms =
                    ASIC_get_asic_job_frequency_ms(
                        GLOBAL_STATE
                    );

                continue;
            }


            timeout_ms =
                ASIC_get_asic_job_frequency_ms(
                    GLOBAL_STATE
                );

            continue;
        }


        /*
         * Final protocol check immediately before job generation.
         */
        active_protocol =
            GLOBAL_STATE->stratum_protocol;

        if (active_protocol != current_work_protocol) {

            ESP_LOGW(
                TAG,
                "Protocol changed before ASIC dispatch, discarding work"
            );

            free_work_item(
                GLOBAL_STATE,
                current_work,
                current_work_protocol
            );

            current_work = NULL;

            current_work_protocol =
                active_protocol;

            last_dispatched_job_v1[0] = '\0';
            last_dispatched_job_sv2 = UINT32_MAX;

            timeout_ms =
                ASIC_get_asic_job_frequency_ms(
                    GLOBAL_STATE
                );

            continue;
        }


        /*
         * ASIC must be initialized before generating/sending work.
         *
         * Do NOT update last_dispatched_job_* here.
         * Otherwise the job could be considered dispatched even though
         * the ASIC never received it.
         */
        if (!GLOBAL_STATE->ASIC_initalized) {

            ESP_LOGW(
                TAG,
                "ASIC not initialized, keeping work pending"
            );

            timeout_ms =
                ASIC_get_asic_job_frequency_ms(
                    GLOBAL_STATE
                );

            vTaskDelay(
                100 / portTICK_PERIOD_MS
            );

            continue;
        }


        /*
         * Generate and dispatch the fresh ASIC job.
         */
        bool dispatched = false;


        if (active_protocol == STRATUM_PROTOCOL_V2) {

            if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {

                dispatched =
                    generate_work_sv2_ext(
                        GLOBAL_STATE,
                        (sv2_ext_job_t *)current_work,
                        difficulty
                    );

            } else {

                dispatched =
                    generate_work_sv2(
                        GLOBAL_STATE,
                        (sv2_job_t *)current_work,
                        difficulty
                    );
            }

        } else {

            dispatched =
                generate_work(
                    GLOBAL_STATE,
                    (mining_notify *)current_work,
                    difficulty
                );
        }


        /*
         * IMPORTANT:
         *
         * Only mark the job as dispatched after the ASIC job object was
         * successfully created and ASIC_send_work() was called.
         */
        if (dispatched) {

            if (active_protocol == STRATUM_PROTOCOL_V2) {

                if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {

                    sv2_ext_job_t *j =
                        (sv2_ext_job_t *)current_work;

                    last_dispatched_job_sv2 =
                        j->job_id;

                } else {

                    sv2_job_t *j =
                        (sv2_job_t *)current_work;

                    last_dispatched_job_sv2 =
                        j->job_id;
                }

            } else {

                mining_notify *j =
                    (mining_notify *)current_work;

                strncpy(
                    last_dispatched_job_v1,
                    j->job_id,
                    sizeof(last_dispatched_job_v1) - 1
                );

                last_dispatched_job_v1[
                    sizeof(last_dispatched_job_v1) - 1
                ] = '\0';
            }
        }


        /*
         * Restore normal ASIC polling interval.
         */
        timeout_ms =
            ASIC_get_asic_job_frequency_ms(
                GLOBAL_STATE
            );
    }
}


/*
 * STRATUM V1
 */
static bool generate_work(GlobalState *GLOBAL_STATE,
                          mining_notify *notification,
                          double difficulty)
{
    if (!notification) {
        return false;
    }


    if (GLOBAL_STATE->extranonce_2_len >
        MAX_EXTRANONCE2_LEN) {

        ESP_LOGE(
            TAG,
            "extranonce_2_len %d exceeds maximum %d, skipping job",
            GLOBAL_STATE->extranonce_2_len,
            MAX_EXTRANONCE2_LEN
        );

        return false;
    }


    /*
     * Industry-standard zero extranonce2.
     *
     * Example:
     *
     * extranonce_2_len = 4
     *
     * => "00000000"
     */
    char extranonce_2_str[MAX_EXTRANONCE2_STR];

    memset(
        extranonce_2_str,
        '0',
        GLOBAL_STATE->extranonce_2_len * 2
    );

    extranonce_2_str[
        GLOBAL_STATE->extranonce_2_len * 2
    ] = '\0';


    /*
     * Coinbase hash.
     */
    uint8_t coinbase_tx_hash[32];

    calculate_coinbase_tx_hash(
        notification->coinbase_1,
        notification->coinbase_2,
        GLOBAL_STATE->extranonce_str,
        extranonce_2_str,
        coinbase_tx_hash
    );


    /*
     * Merkle root.
     */
    uint8_t merkle_root[32];

    calculate_merkle_root_hash(
        coinbase_tx_hash,
        (uint8_t (*)[32])notification->merkle_branches,
        notification->n_merkle_branches,
        merkle_root
    );


    /*
     * Allocate ASIC job.
     */
    bm_job *next_job =
        malloc(sizeof(bm_job));

    if (next_job == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to allocate memory for new job"
        );

        return false;
    }


    /*
     * Construct job.
     */
    construct_bm_job(
        notification,
        merkle_root,
        GLOBAL_STATE->version_mask,
        difficulty,
        next_job
    );


    /*
     * Job metadata.
     */
    next_job->extranonce2 =
        strdup(extranonce_2_str);

    next_job->jobid =
        strdup(notification->job_id);

    next_job->version_mask =
        GLOBAL_STATE->version_mask;


    if (next_job->extranonce2 == NULL ||
        next_job->jobid == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to allocate V1 job metadata"
        );

        free(next_job->jobid);
        free(next_job->extranonce2);
        free(next_job);

        return false;
    }


    /*
     * Send to ASIC.
     */
    ASIC_send_work(
        GLOBAL_STATE,
        next_job
    );

    return true;
}


/*
 * STRATUM V2 STANDARD CHANNEL
 */
static bool generate_work_sv2(GlobalState *GLOBAL_STATE,
                              sv2_job_t *sv2_job,
                              double difficulty)
{
    if (!sv2_job) {
        return false;
    }


    bm_job *next_job =
        malloc(sizeof(bm_job));

    if (next_job == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to allocate memory for new SV2 job"
        );

        return false;
    }


    uint32_t version_mask =
        GLOBAL_STATE->version_mask;


    /*
     * Basic job parameters.
     */
    next_job->version =
        sv2_job->version;

    next_job->target =
        sv2_job->nbits;

    next_job->ntime =
        sv2_job->ntime;

    next_job->starting_nonce =
        0;

    next_job->pool_diff =
        difficulty;


    /*
     * Byte order conversion.
     */
    reverse_32bit_words(
        sv2_job->merkle_root,
        next_job->merkle_root
    );

    reverse_32bit_words(
        sv2_job->prev_hash,
        next_job->prev_block_hash
    );


    /*
     * Midstate base data.
     *
     * First 64 bytes of the block header:
     *
     * version      4
     * prevhash    32
     * merkle[0:28] 28
     */
    uint8_t midstate_data[64];

    uint32_t base_version =
        sv2_job->version;

    memcpy(
        midstate_data,
        &base_version,
        4
    );

    memcpy(
        midstate_data + 4,
        sv2_job->prev_hash,
        32
    );

    memcpy(
        midstate_data + 36,
        sv2_job->merkle_root,
        28
    );


    /*
     * Base midstate.
     */
    uint8_t midstate[32];

    midstate_sha256_bin(
        midstate_data,
        64,
        midstate
    );

    reverse_32bit_words(
        midstate,
        next_job->midstate
    );


    /*
     * Version rolling.
     */
    if (version_mask != 0) {

        uint32_t rolled_version;


        /*
         * Version #1
         */
        rolled_version =
            increment_bitmask(
                base_version,
                version_mask
            );

        memcpy(
            midstate_data,
            &rolled_version,
            4
        );

        midstate_sha256_bin(
            midstate_data,
            64,
            midstate
        );

        reverse_32bit_words(
            midstate,
            next_job->midstate1
        );


        /*
         * Version #2
         */
        rolled_version =
            increment_bitmask(
                rolled_version,
                version_mask
            );

        memcpy(
            midstate_data,
            &rolled_version,
            4
        );

        midstate_sha256_bin(
            midstate_data,
            64,
            midstate
        );

        reverse_32bit_words(
            midstate,
            next_job->midstate2
        );


        /*
         * Version #3
         */
        rolled_version =
            increment_bitmask(
                rolled_version,
                version_mask
            );

        memcpy(
            midstate_data,
            &rolled_version,
            4
        );

        midstate_sha256_bin(
            midstate_data,
            64,
            midstate
        );

        reverse_32bit_words(
            midstate,
            next_job->midstate3
        );

        next_job->num_midstates = 4;

    } else {

        next_job->num_midstates = 1;
    }


    /*
     * Job ID.
     */
    char jobid_str[16];

    snprintf(
        jobid_str,
        sizeof(jobid_str),
        "%" PRIu32,
        sv2_job->job_id
    );


    next_job->jobid =
        strdup(jobid_str);

    /*
     * Standard SV2 channel does not use V1-style extranonce2.
     */
    next_job->extranonce2 =
        strdup("");

    next_job->version_mask =
        version_mask;


    if (next_job->jobid == NULL ||
        next_job->extranonce2 == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to allocate SV2 job metadata"
        );

        free(next_job->jobid);
        free(next_job->extranonce2);
        free(next_job);

        return false;
    }


    /*
     * Send to ASIC.
     */
    ASIC_send_work(
        GLOBAL_STATE,
        next_job
    );

    return true;
}


/*
 * STRATUM V2 EXTENDED CHANNEL
 */
static bool generate_work_sv2_ext(GlobalState *GLOBAL_STATE,
                                  sv2_ext_job_t *ext_job,
                                  double difficulty)
{
    if (!ext_job) {
        return false;
    }


    sv2_conn_t *conn =
        GLOBAL_STATE->sv2_conn;

    if (!conn) {

        ESP_LOGE(
            TAG,
            "SV2 connection is NULL"
        );

        return false;
    }


    bm_job *next_job =
        malloc(sizeof(bm_job));

    if (!next_job) {

        ESP_LOGE(
            TAG,
            "Failed to allocate memory for SV2 ext job"
        );

        return false;
    }


    uint32_t version_mask =
        GLOBAL_STATE->version_mask;


    /*
     * Extended channel extranonce.
     */
    uint8_t extranonce_2_len =
        conn->extranonce_size;

    if (extranonce_2_len >
        sizeof(((uint8_t[32]){0}))) {

        ESP_LOGE(
            TAG,
            "SV2 extranonce size %u exceeds 32 bytes",
            extranonce_2_len
        );

        free(next_job);

        return false;
    }


    uint8_t extranonce_2[32];

    memset(
        extranonce_2,
        0,
        sizeof(extranonce_2)
    );


    /*
     * Coinbase transaction hash.
     */
    uint8_t coinbase_tx_hash[32];

    calculate_coinbase_tx_hash_bin(
        ext_job->coinbase_prefix,
        ext_job->coinbase_prefix_len,

        conn->extranonce_prefix,
        conn->extranonce_prefix_len,

        extranonce_2,
        extranonce_2_len,

        ext_job->coinbase_suffix,
        ext_job->coinbase_suffix_len,

        coinbase_tx_hash
    );


    /*
     * Merkle root.
     */
    uint8_t merkle_root[32];

    calculate_merkle_root_hash(
        coinbase_tx_hash,
        (const uint8_t (*)[32])ext_job->merkle_path,
        ext_job->merkle_path_count,
        merkle_root
    );


    /*
     * Basic job fields.
     */
    next_job->version =
        ext_job->version;

    next_job->target =
        ext_job->nbits;

    next_job->ntime =
        ext_job->ntime;

    next_job->starting_nonce =
        0;

    next_job->pool_diff =
        difficulty;


    /*
     * Byte order conversion.
     */
    reverse_32bit_words(
        merkle_root,
        next_job->merkle_root
    );

    reverse_32bit_words(
        ext_job->prev_hash,
        next_job->prev_block_hash
    );


    /*
     * Build midstate data.
     */
    uint8_t midstate_data[64];

    uint32_t base_version =
        ext_job->version;

    memcpy(
        midstate_data,
        &base_version,
        4
    );

    memcpy(
        midstate_data + 4,
        ext_job->prev_hash,
        32
    );

    memcpy(
        midstate_data + 36,
        merkle_root,
        28
    );


    /*
     * Base midstate.
     */
    uint8_t midstate[32];

    midstate_sha256_bin(
        midstate_data,
        64,
        midstate
    );

    reverse_32bit_words(
        midstate,
        next_job->midstate
    );


    /*
     * Version rolling.
     */
    if (version_mask != 0) {

        uint32_t rolled_version;


        /*
         * Version #1
         */
        rolled_version =
            increment_bitmask(
                base_version,
                version_mask
            );

        memcpy(
            midstate_data,
            &rolled_version,
            4
        );

        midstate_sha256_bin(
            midstate_data,
            64,
            midstate
        );

        reverse_32bit_words(
            midstate,
            next_job->midstate1
        );


        /*
         * Version #2
         */
        rolled_version =
            increment_bitmask(
                rolled_version,
                version_mask
            );

        memcpy(
            midstate_data,
            &rolled_version,
            4
        );

        midstate_sha256_bin(
            midstate_data,
            64,
            midstate
        );

        reverse_32bit_words(
            midstate,
            next_job->midstate2
        );


        /*
         * Version #3
         */
        rolled_version =
            increment_bitmask(
                rolled_version,
                version_mask
            );

        memcpy(
            midstate_data,
            &rolled_version,
            4
        );

        midstate_sha256_bin(
            midstate_data,
            64,
            midstate
        );

        reverse_32bit_words(
            midstate,
            next_job->midstate3
        );

        next_job->num_midstates = 4;

    } else {

        next_job->num_midstates = 1;
    }


    /*
     * Job ID.
     */
    char jobid_str[16];

    snprintf(
        jobid_str,
        sizeof(jobid_str),
        "%" PRIu32,
        ext_job->job_id
    );

    next_job->jobid =
        strdup(jobid_str);


    /*
     * Extended-channel extranonce2.
     */
    char en2_hex[65];

    bin2hex(
        extranonce_2,
        extranonce_2_len,
        en2_hex,
        sizeof(en2_hex)
    );

    next_job->extranonce2 =
        strdup(en2_hex);

    next_job->version_mask =
        version_mask;


    if (next_job->jobid == NULL ||
        next_job->extranonce2 == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to allocate SV2 ext job metadata"
        );

        free(next_job->jobid);
        free(next_job->extranonce2);
        free(next_job);

        return false;
    }


    /*
     * Send to ASIC.
     */
    ASIC_send_work(
        GLOBAL_STATE,
        next_job
    );

    return true;
}
