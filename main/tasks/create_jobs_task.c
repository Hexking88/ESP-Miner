#include <sys/time.h>
#include <limits.h>
#include <string.h>
#include <inttypes.h>

#include "work_queue.h"
#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "mining.h"
#include "asic.h"
#include "system.h"
#include "stratum_api.h"
#include "utils.h"

static const char *TAG = "create_jobs_task";

#define MAX_EXTRANONCE2_LEN 32
#define MAX_EXTRANONCE2_STR (MAX_EXTRANONCE2_LEN * 2 + 1)

// Oneven stapgrootte (Golden Ratio) → coprime met elke macht van 2,
// dus volledige cyclus zonder duplicaten.
#define EXTRANONCE2_STEP 0x9E3779B97F4A7C15ULL

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Masker de teller zodat hij past binnen extranonce_2_len bytes.
static inline uint64_t mask_extranonce2(uint64_t val, uint8_t len)
{
    if (len == 0) {
        return 0;
    }
    if (len >= 8) {
        return val;
    }
    uint64_t mask = (1ULL << (len * 8)) - 1ULL;
    return val & mask;
}

// Genereer een random startwaarde binnen de toegestane ruimte.
static inline uint64_t random_extranonce2(uint8_t len)
{
    if (len == 0) {
        return 0;
    }
    uint64_t r = ((uint64_t)esp_random() << 32) | (uint64_t)esp_random();
    return mask_extranonce2(r, len);
}

static void generate_work(GlobalState *GLOBAL_STATE,
                          mining_notify *notification,
                          uint64_t extranonce_2,
                          double difficulty);

// ---------------------------------------------------------------------------
// Hoofdtaak
// ---------------------------------------------------------------------------

void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    double difficulty = GLOBAL_STATE->pool_difficulty;

    mining_notify *current_work = NULL;

    uint8_t  last_extranonce_2_len = GLOBAL_STATE->extranonce_2_len;
    uint64_t extranonce_2      = random_extranonce2(last_extranonce_2_len);

    ESP_LOGI(TAG, "ASIC Job Interval: %d ms",
             ASIC_get_asic_job_frequency_ms(GLOBAL_STATE));
    ESP_LOGI(TAG, "Extranonce2 step: 0x%016" PRIx64 ", len: %u bytes (modulus 2^%u)",
             (uint64_t)EXTRANONCE2_STEP,
             (unsigned)last_extranonce_2_len,
             (unsigned)(last_extranonce_2_len * 8));
    ESP_LOGI(TAG, "ASIC Ready!");

    while (1) {
        // -------------------------------------------------------------------
        // 1. Reset aanvragen verwerken (bv. bij reconnect of pool-wijziging)
        // -------------------------------------------------------------------
        if (GLOBAL_STATE->reset_extranonce2 ||
            GLOBAL_STATE->extranonce_2_len != last_extranonce_2_len) {

            last_extranonce_2_len = GLOBAL_STATE->extranonce_2_len;
            extranonce_2 = random_extranonce2(last_extranonce_2_len);

            ESP_LOGI(TAG, "Extranonce2 gereset naar 0x%016" PRIx64 " (len=%u)",
                     extranonce_2, (unsigned)last_extranonce_2_len);

            GLOBAL_STATE->reset_extranonce2 = false;
        }

        // -------------------------------------------------------------------
        // 2. Difficulty / version rolling direct verwerken (niet wachten op werk)
        // -------------------------------------------------------------------
        if (GLOBAL_STATE->new_set_mining_difficulty_msg) {
            difficulty = GLOBAL_STATE->pool_difficulty;
            GLOBAL_STATE->new_set_mining_difficulty_msg = false;
            ESP_LOGI(TAG, "Nieuwe pool difficulty: %.2f", difficulty);
        }

        if (GLOBAL_STATE->new_stratum_version_rolling_msg && GLOBAL_STATE->ASIC_initalized) {
            ESP_LOGI(TAG, "Set chip version rolls %i",
                     (int)(GLOBAL_STATE->version_mask >> 13));
            ASIC_set_version_mask(GLOBAL_STATE, GLOBAL_STATE->version_mask);
            GLOBAL_STATE->new_stratum_version_rolling_msg = false;
        }

        // -------------------------------------------------------------------
        // 3. Wacht op nieuwe work (met resterende timeout)
        // -------------------------------------------------------------------
        int timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
        uint64_t start_us = esp_timer_get_time();

        mining_notify *new_work =
            queue_dequeue_timeout(&GLOBAL_STATE->stratum_queue, timeout_ms);

        if (new_work != NULL) {
            ESP_LOGI(TAG, "New Work Dequeued %s", new_work->job_id);

            // Vervang oude work altijd (V1: nieuwe job is altijd leidend)
            if (current_work != NULL) {
                STRATUM_V1_free_mining_notify(current_work);
            }
            current_work = new_work;

            // clean_jobs: bij V1 betekent false meestal "oude shares nog geldig".
            // Wij sturen altijd direct de nieuwe job; alleen loggen als het afwijkt.
            if (!current_work->clean_jobs) {
                ESP_LOGD(TAG, "clean_jobs=false (job %s)", current_work->job_id);
            }
        } else {
            // Geen nieuwe work: als we nog niets hebben, kort wachten en opnieuw
            if (current_work == NULL) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
            // Anders: blijf de huidige job heruitzenden met nieuwe extranonce2
        }

        // -------------------------------------------------------------------
        // 4. ASIC nog niet klaar? Niet alloceren, kort wachten
        // -------------------------------------------------------------------
        if (!GLOBAL_STATE->ASIC_initalized) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // -------------------------------------------------------------------
        // 5. Werk genereren en naar ASIC sturen
        // -------------------------------------------------------------------
        ESP_LOGD(TAG, "Genereer werk: extranonce_2=0x%016" PRIx64 " (dec: %" PRIu64 ")",
                 extranonce_2, extranonce_2);

        generate_work(GLOBAL_STATE, current_work, extranonce_2, difficulty);

        // Volgende extranonce2 (oneven stap → coprime met modulus)
        extranonce_2 = mask_extranonce2(extranonce_2 + EXTRANONCE2_STEP,
                                        GLOBAL_STATE->extranonce_2_len);

        // Reset timeout voor de volgende iteratie (niet cumulatief)
        int elapsed_ms = (int)((esp_timer_get_time() - start_us) / 1000);
        (void)elapsed_ms; // huidige iteratie is klaar, timeout wordt bovenaan opnieuw bepaald
    }
}

// ---------------------------------------------------------------------------
// Werk genereren voor Stratum V1
// ---------------------------------------------------------------------------

static void generate_work(GlobalState *GLOBAL_STATE,
                          mining_notify *notification,
                          uint64_t extranonce_2,
                          double difficulty)
{
    if (GLOBAL_STATE->extranonce_2_len > MAX_EXTRANONCE2_LEN) {
        ESP_LOGE(TAG, "extranonce_2_len %d exceeds maximum %d, skipping job",
                 GLOBAL_STATE->extranonce_2_len, MAX_EXTRANONCE2_LEN);
        return;
    }

    char extranonce_2_str[MAX_EXTRANONCE2_STR];
    extranonce_2_generate(extranonce_2, GLOBAL_STATE->extranonce_2_len, extranonce_2_str);

    uint8_t coinbase_tx_hash[32];
    calculate_coinbase_tx_hash(notification->coinbase_1,
                               notification->coinbase_2,
                               GLOBAL_STATE->extranonce_str,
                               extranonce_2_str,
                               coinbase_tx_hash);

    uint8_t merkle_root[32];
    calculate_merkle_root_hash(coinbase_tx_hash,
                               (uint8_t(*)[32])notification->merkle_branches,
                               notification->n_merkle_branches,
                               merkle_root);

    bm_job *next_job = calloc(1, sizeof(bm_job));
    if (next_job == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for new job");
        return;
    }

    construct_bm_job(notification, merkle_root, GLOBAL_STATE->version_mask,
                     difficulty, next_job);

    next_job->extranonce2 = strdup(extranonce_2_str);
    next_job->jobid      = strdup(notification->job_id);
    next_job->version_mask = GLOBAL_STATE->version_mask;

    if (next_job->extranonce2 == NULL || next_job->jobid == NULL) {
        ESP_LOGE(TAG, "strdup failed for job fields");
        free(next_job->extranonce2);
        free(next_job->jobid);
        free(next_job);
        return;
    }

    ASIC_send_work(GLOBAL_STATE, next_job);
}
