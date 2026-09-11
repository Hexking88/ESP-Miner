#include <sys/time.h>
#include <limits.h>

#include "work_queue.h"
#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "mining.h"
#include "string.h"
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

static void generate_work(GlobalState *GLOBAL_STATE, mining_notify *notification, uint64_t extranonce_2, double difficulty);
static void generate_work_sv2(GlobalState *GLOBAL_STATE, sv2_job_t *job, double difficulty);
static void generate_work_sv2_ext(GlobalState *GLOBAL_STATE, sv2_ext_job_t *job, double difficulty, uint64_t extranonce_2_counter);

// Oneven stapgroottes per extranonce2-lengte (copriem met 2^(8*len))
static inline uint64_t get_extranonce2_step(uint8_t len)
{
    switch (len) {
        case 1:  return 0x9DULL;
        case 2:  return 0x9E37ULL;
        case 3:  return 0x9E3779ULL;
        case 4:  return 0x9E3779B9ULL;
        case 5:  return 0x9E3779B97FULL;
        case 6:  return 0x9E3779B97F4AULL;
        case 7:  return 0x9E3779B97F4A7CULL;
        case 8:
        default: return 0x9E3779B97F4A7C15ULL;
    }
}

// Pas maskering toe zodat de counter niet groter wordt dan toegestaan door de pool
static inline uint64_t mask_extranonce2(uint64_t val, uint8_t len)
{
    if (len >= 8) {
        return val;
    }
    if (len == 0) {
        return 0;
    }
    uint64_t mask = (1ULL << (len * 8)) - 1ULL;
    return val & mask;
}

// Willekeurige startwaarde om hergebruik na reboot te voorkomen
static inline uint64_t random_extranonce2(uint8_t len)
{
    uint64_t r = ((uint64_t)esp_random() << 32) | esp_random();
    return mask_extranonce2(r, len);
}

// Free a work item using the correct free function for the protocol it was created under
static void free_work_item(GlobalState *GLOBAL_STATE, void *work, stratum_protocol_t protocol)
{
    if (!work) return;
    if (protocol == STRATUM_PROTOCOL_V2) {
        if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {
            sv2_ext_job_free((sv2_ext_job_t *)work);
        } else {
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
    stratum_protocol_t current_work_protocol = GLOBAL_STATE->stratum_protocol;

    // Validatie: extranonce_2_len moet > 0 zijn, anders kunnen we geen unieke werk genereren
    if (GLOBAL_STATE->extranonce_2_len == 0) {
        ESP_LOGW(TAG, "extranonce_2_len is 0; pools zonder extranonce2 leveren duplicate work op. "
                      "Mijnwerker zal doorgaan maar shares kunnen geweigerd worden.");
    }
    if (GLOBAL_STATE->extranonce_2_len > MAX_EXTRANONCE2_LEN) {
        ESP_LOGE(TAG, "extranonce_2_len %d > MAX %d", GLOBAL_STATE->extranonce_2_len, MAX_EXTRANONCE2_LEN);
    }

    // Willekeurige startwaarde zodat we na reboot niet dezelfde reeks hergebruiken
    uint64_t extranonce_2 = random_extranonce2(GLOBAL_STATE->extranonce_2_len);
    uint64_t extranonce_2_step = get_extranonce2_step(GLOBAL_STATE->extranonce_2_len);

    ESP_LOGI(TAG, "Stapgrootte voor extranonce2 (len=%u): 0x%llx, start=0x%llx",
             GLOBAL_STATE->extranonce_2_len,
             (unsigned long long)extranonce_2_step,
             (unsigned long long)extranonce_2);

    int timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    ESP_LOGI(TAG, "ASIC Job Interval: %d ms", timeout_ms);
    ESP_LOGI(TAG, "ASIC Ready!");

    while (1) {
        // Reset aanvraag vanuit stratum-taak
        if (GLOBAL_STATE->reset_extranonce2) {
            extranonce_2 = random_extranonce2(GLOBAL_STATE->extranonce_2_len);
            extranonce_2_step = get_extranonce2_step(GLOBAL_STATE->extranonce_2_len);
            ESP_LOGI(TAG, "Reset extranonce2. Nieuwe start=0x%llx, stap=0x%llx",
                     (unsigned long long)extranonce_2,
                     (unsigned long long)extranonce_2_step);
            GLOBAL_STATE->reset_extranonce2 = false;
        }

        stratum_protocol_t active_protocol = GLOBAL_STATE->stratum_protocol;

        // Protocol-switch: gooi huidige werk weg (hoort bij oud protocol)
        if (active_protocol != current_work_protocol) {
            if (current_work != NULL) {
                ESP_LOGI(TAG, "Protocol switch %s -> %s, huidige werk weggegooid",
                         current_work_protocol == STRATUM_PROTOCOL_V2 ? STRATUM_V2 : STRATUM_V1,
                         active_protocol == STRATUM_PROTOCOL_V2 ? STRATUM_V2 : STRATUM_V1);
                free_work_item(GLOBAL_STATE, current_work, current_work_protocol);
                current_work = NULL;
            }
            current_work_protocol = active_protocol;
            timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
        }

        uint64_t start_time = esp_timer_get_time();
        void *new_work = queue_dequeue_timeout(&GLOBAL_STATE->stratum_queue, timeout_ms);

        int elapsed_ms = (int)((esp_timer_get_time() - start_time) / 1000);
        if (elapsed_ms >= timeout_ms) {
            timeout_ms = 0;
        } else {
            timeout_ms -= elapsed_ms;
        }

        if (new_work != NULL) {
            active_protocol = GLOBAL_STATE->stratum_protocol;

            // Vrijgeven van vorige werk (met het protocol waaronder het gemaakt is)
            free_work_item(GLOBAL_STATE, current_work, current_work_protocol);
            current_work = NULL;

            if (active_protocol != current_work_protocol) {
                // Item was geënqueued onder oud protocol (of nieuw — onzeker).
                // Gebruik het protocol dat actief was tijdens dequeue (current_work_protocol = oud)
                ESP_LOGW(TAG, "Protocol-switch tijdens dequeue; item weggegooid");
                free_work_item(GLOBAL_STATE, new_work, current_work_protocol);
                current_work_protocol = active_protocol;
                timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
                continue;
            }

            if (current_work_protocol == STRATUM_PROTOCOL_V2) {
                if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {
                    ESP_LOGI(TAG, "New Work Dequeued SV2 ext job %lu", ((sv2_ext_job_t *)new_work)->job_id);
                } else {
                    ESP_LOGI(TAG, "New Work Dequeued SV2 job %lu", ((sv2_job_t *)new_work)->job_id);
                }
            } else {
                ESP_LOGI(TAG, "New Work Dequeued %s", ((mining_notify *)new_work)->job_id);
            }

            current_work = new_work;

            if (GLOBAL_STATE->new_set_mining_difficulty_msg) {
                ESP_LOGI(TAG, "New pool difficulty %.2f", GLOBAL_STATE->pool_difficulty);
                difficulty = GLOBAL_STATE->pool_difficulty;
                GLOBAL_STATE->new_set_mining_difficulty_msg = false;
            }

            if (GLOBAL_STATE->new_stratum_version_rolling_msg && GLOBAL_STATE->ASIC_initalized) {
                ESP_LOGI(TAG, "Set chip version rolls %i", (int)(GLOBAL_STATE->version_mask >> 13));
                ASIC_set_version_mask(GLOBAL_STATE, GLOBAL_STATE->version_mask);
                GLOBAL_STATE->new_stratum_version_rolling_msg = false;
            }

            // clean_jobs: alleen gebruiken om te bepalen of oude shares nog geldig zijn.
            // We genereren ALTIJD werk, ook bij clean=false.
            bool clean;
            if (current_work_protocol == STRATUM_PROTOCOL_V2) {
                if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {
                    clean = ((sv2_ext_job_t *)current_work)->clean_jobs;
                } else {
                    clean = ((sv2_job_t *)current_work)->clean_jobs;
                }
            } else {
                clean = ((mining_notify *)current_work)->clean_jobs;
            }
            if (!clean) {
                ESP_LOGD(TAG, "Non-clean job; werk wordt alsnog gegenereerd");
                // GEEN continue — gewoon doorgaan
            }
        } else {
            if (current_work == NULL) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
                continue;
            }
            if (active_protocol == STRATUM_PROTOCOL_V2 && !stratum_v2_is_extended_channel(GLOBAL_STATE)) {
                timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
                continue;
            }
        }

        // Protocol opnieuw lezen; switch kan gebeurd zijn
        active_protocol = GLOBAL_STATE->stratum_protocol;
        if (active_protocol != current_work_protocol) {
            free_work_item(GLOBAL_STATE, current_work, current_work_protocol);
            current_work = NULL;
            current_work_protocol = active_protocol;
            timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
            continue;
        }

        ESP_LOGI(TAG, "Genereren werk: extranonce_2 = 0x%llx (dec: %llu)",
                 (unsigned long long)extranonce_2, (unsigned long long)extranonce_2);

        if (active_protocol == STRATUM_PROTOCOL_V2) {
            if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {
                generate_work_sv2_ext(GLOBAL_STATE, (sv2_ext_job_t *)current_work, difficulty, extranonce_2);
                extranonce_2 = mask_extranonce2(extranonce_2 + extranonce_2_step, GLOBAL_STATE->extranonce_2_len);
            } else {
                generate_work_sv2(GLOBAL_STATE, (sv2_job_t *)current_work, difficulty);
            }
        } else {
            generate_work(GLOBAL_STATE, (mining_notify *)current_work, extranonce_2, difficulty);
            extranonce_2 = mask_extranonce2(extranonce_2 + extranonce_2_step, GLOBAL_STATE->extranonce_2_len);
        }

        timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    }
}

static void generate_work(GlobalState *GLOBAL_STATE, mining_notify *notification, uint64_t extranonce_2, double difficulty)
{
    if (GLOBAL_STATE->extranonce_2_len > MAX_EXTRANONCE2_LEN) {
        ESP_LOGE(TAG, "extranonce_2_len %d exceeds maximum %d, skipping job", GLOBAL_STATE->extranonce_2_len, MAX_EXTRANONCE2_LEN);
        return;
    }

    char extranonce_2_str[MAX_EXTRANONCE2_STR];
    extranonce_2_generate(extranonce_2, GLOBAL_STATE->extranonce_2_len, extranonce_2_str);

    uint8_t coinbase_tx_hash[32];
    calculate_coinbase_tx_hash(notification->coinbase_1, notification->coinbase_2, GLOBAL_STATE->extranonce_str, extranonce_2_str, coinbase_tx_hash);

    uint8_t merkle_root[32];
    calculate_merkle_root_hash(coinbase_tx_hash, (uint8_t(*)[32])notification->merkle_branches, notification->n_merkle_branches, merkle_root);

    bm_job *next_job = malloc(sizeof(bm_job));
    if (next_job == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for new job");
        return;
    }
    // Belangrijk: voorkom dat we op onvoldoende geïnitialiseerde velden vrijgeven
    memset(next_job, 0, sizeof(bm_job));

    construct_bm_job(notification, merkle_root, GLOBAL_STATE->version_mask, difficulty, next_job);

    next_job->extranonce2 = strdup(extranonce_2_str);
    next_job->jobid = strdup(notification->job_id);
    next_job->version_mask = GLOBAL_STATE->version_mask;

    if (next_job->extranonce2 == NULL || next_job->jobid == NULL) {
        ESP_LOGE(TAG, "strdup gefaald bij aanmaken job");
        free(next_job->extranonce2);
        free(next_job->jobid);
        free(next_job);
        return;
    }

    if (!GLOBAL_STATE->ASIC_initalized) {
        ESP_LOGW(TAG, "ASIC not initialized, skipping job send");
        free(next_job->jobid);
        free(next_job->extranonce2);
        free(next_job);
        return;
    }

    ASIC_send_work(GLOBAL_STATE, next_job);
}
