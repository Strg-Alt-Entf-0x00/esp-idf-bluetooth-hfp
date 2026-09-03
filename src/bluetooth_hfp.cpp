// ============================================================================
// Bluetooth HFP Audio Gateway - Full Production Implementation
// ESP-IDF Bluetooth HFP Audio Gateway component.
// ============================================================================
// Role:   Hands-Free (HF) - ESP32 presents itself as a phone headset
// Profile: HFP 1.7, WBS (mSBC 16kHz) + CVSD (8kHz) fallback
// Audio:  HCI path - SCO audio routed through application (no I2S hardware needed)
// ============================================================================

#include "bluetooth_hfp.h"

#include <cstring>
#include <atomic>
#include <algorithm>
#include <cstdio>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

extern "C" {
#include "oi_codec_sbc.h"
#include "sbc_encoder.h"
}

namespace bluetooth_hfp {

// ============================================================================
// Constants
// ============================================================================

static const char* TAG = "BT-HFP";

static constexpr const char* NVS_NAMESPACE    = "esp_hfp";
static constexpr const char* NVS_KEY_LAST_MAC = "last_mac";
static void escape_json_string(const char* input, char* output, size_t output_size) {
    if (!output || output_size == 0) return;
    size_t out = 0;
    if (input) {
        for (size_t in = 0; input[in] != '\0' && out + 1 < output_size; ++in) {
            const char c = input[in];
            if ((c == '"' || c == '\\') && out + 2 < output_size) {
                output[out++] = '\\';
            } else if ((c == '\n' || c == '\r' || c == '\t') && out + 2 < output_size) {
                output[out++] = '\\';
                output[out++] = c == '\n' ? 'n' : (c == '\r' ? 'r' : 't');
                continue;
            }
            output[out++] = c;
        }
    }
    output[out] = '\0';
}

// SCO ringbuffer: 500ms at 8kHz = 8192 bytes (Industrial-Grade Jitter Buffer)
static constexpr size_t SCO_TX_RINGBUF_SIZE = 8192; // PC audio -> SCO to phone
static constexpr size_t SCO_RX_RINGBUF_SIZE = 8192; // SCO from phone -> PC

static constexpr int kMaxBonds = 20;

// ============================================================================
// Internal State
// ============================================================================

static bool              s_bt_initialized      = false;
static bool              s_sco_active          = false;
static bool              s_auto_connect        = false;

// Auto-reconnect state
static bool              s_manual_disconnect   = false;
static TaskHandle_t      s_reconnect_task_hdl  = nullptr;
static std::atomic<bool> s_reconnect_task_active{false};
static std::atomic<bool> s_connection_attempt_in_progress{false};
static constexpr int     kMaxReconnectAttempts = 10;
static constexpr int     kReconnectBaseMs      = 1000;  // Initial delay 1s
static constexpr int     kReconnectMaxMs       = 30000; // Cap at 30s
static ConnectionStatus  s_conn_status         = ConnectionStatus::DISCONNECTED;
static CallState         s_call_state          = CallState::IDLE;
static HFPConnectionInfo s_conn_info           = {};
static BluetoothStats    s_stats               = {};

// SCO connection handle (obtained from ESP_HF_CLIENT_AUDIO_STATE_EVT)
static esp_hf_sync_conn_hdl_t s_sync_conn_hdl = 0;
static uint16_t          s_preferred_frame_size = 120; // Default CVSD size
static int               s_sco_sample_rate      = 0;   // 0=inactive, 8000=CVSD, 16000=mSBC
static std::atomic<int>  s_speaker_volume{10};
static StatusCallback    s_status_cb            = nullptr;
static AudioRxCallback   s_audio_rx_cb          = nullptr;
static StatusMessageCallback s_status_message_cb = nullptr;

static void emit_status_message(const char* message) {
    if (s_status_message_cb && message) s_status_message_cb(message);
}

// Synchronization
static SemaphoreHandle_t s_mutex              = nullptr;
static SemaphoreHandle_t s_scan_semaphore     = nullptr;
static SemaphoreHandle_t s_connection_semaphore = nullptr;

// Ringbuffers for SCO audio bridge
static RingbufHandle_t   s_sco_tx_ringbuf    = nullptr; // PC audio -> SCO to phone
static RingbufHandle_t   s_sco_rx_ringbuf    = nullptr; // SCO from phone -> PC

// Tasks
static TaskHandle_t      s_sco_bridge_task   = nullptr;

// mSBC Codec Contexts (for Wideband Speech when EXTERNAL_CODEC=y)
static OI_CODEC_SBC_DECODER_CONTEXT s_sbc_decoder_ctx;
static OI_UINT32 s_sbc_decoder_data[CODEC_DATA_WORDS(1, SBC_CODEC_FAST_FILTER_BUFFERS)];
static SBC_ENC_PARAMS s_sbc_encoder_ctx;
static bool s_msbc_decoder_initialized = false;
static bool s_msbc_encoder_initialized = false;

// Scan results
static std::vector<BluetoothDevice> s_scan_results;
static std::vector<BluetoothDevice> s_paired_devices;

// ============================================================================
// Forward Declarations
// ============================================================================

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param);
static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t* param);
static void hf_client_audio_rx_cb(esp_hf_sync_conn_hdl_t hdl, esp_hf_audio_buff_t* buf, bool is_bad_frame);
static void sco_tx_bridge_task(void* arg);
static void reconnect_task_fn(void* arg);
static void cancel_reconnect_task();
static void boot_connect_task(void* arg);
static esp_err_t scan_and_connect_once();
static void save_last_mac(const uint8_t* bda);
static bool load_last_mac(uint8_t* bda_out);
static void refresh_paired_devices_locked();
static void trigger_auto_reconnect();

// ============================================================================
// NVS: Persist last connected BT device MAC
// ============================================================================

static void save_last_mac(const uint8_t* bda) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY_LAST_MAC, bda, 6);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Saved last BT MAC %02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static bool load_last_mac(uint8_t* bda_out) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = 6;
    bool ok = (nvs_get_blob(h, NVS_KEY_LAST_MAC, bda_out, &len) == ESP_OK && len == 6);
    nvs_close(h);
    return ok;
}

// ============================================================================
// Auto-Reconnect Task (exponential backoff, separate FreeRTOS task)
// Commercial headsets (Jabra, Plantronics, AirPods) all implement this.
// ============================================================================

static void reconnect_task_fn(void* arg) {
    (void)arg;
    ESP_LOGI(TAG, "Auto-reconnect task started");

    uint8_t last_bda[6] = {};
    if (!load_last_mac(last_bda)) {
        ESP_LOGW(TAG, "Auto-reconnect: no last device in NVS");
        s_reconnect_task_hdl = nullptr;
        s_reconnect_task_active.store(false, std::memory_order_release);
        vTaskDelete(nullptr);
        return;
    }

    int delay_ms = kReconnectBaseMs;
    bool scan_fallback_attempted = false;

    for (int attempt = 1; attempt <= kMaxReconnectAttempts; ++attempt) {
        // Check if we connected in the meantime or user triggered manual disconnect
        if (s_conn_status != ConnectionStatus::DISCONNECTED || s_manual_disconnect) {
            ESP_LOGI(TAG, "Auto-reconnect cancelled (connected or manual disconnect)");
            break;
        }

        ESP_LOGI(TAG, "Auto-reconnect attempt %d/%d (delay=%dms)",
                 attempt, kMaxReconnectAttempts, delay_ms);

        // Notify PC of reconnecting state
        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            s_conn_info.status = ConnectionStatus::RECONNECTING;
            s_conn_status = ConnectionStatus::RECONNECTING;
            xSemaphoreGive(s_mutex);
        }
        if (s_status_cb) {
            s_status_cb(ConnectionStatus::RECONNECTING, s_call_state);
        }

        // Attempt connection
        esp_bd_addr_t remote;
        std::memcpy(remote, last_bda, 6);
        s_connection_attempt_in_progress.store(true, std::memory_order_relaxed);
        esp_err_t ret = esp_hf_client_connect(remote);

        if (ret != ESP_OK) {
            s_connection_attempt_in_progress.store(false, std::memory_order_relaxed);
            ESP_LOGW(TAG, "Auto-reconnect connect call failed: %s", esp_err_to_name(ret));
        }

        // Wait for the backoff period. Check periodically if we connected.
        int waited = 0;
        while (waited < delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(500));
            waited += 500;
            if (s_conn_status == ConnectionStatus::CONNECTED ||
                s_conn_status == ConnectionStatus::SCO_ACTIVE ||
                s_manual_disconnect) {
                break;
            }
        }

        // If connected, we are done
        if (s_conn_status == ConnectionStatus::CONNECTED ||
            s_conn_status == ConnectionStatus::SCO_ACTIVE) {
            ESP_LOGI(TAG, "Auto-reconnect succeeded on attempt %d", attempt);
            ++s_stats.reconnects;
            break;
        }

        if (!scan_fallback_attempted && attempt >= 3 &&
            s_conn_status == ConnectionStatus::DISCONNECTED && !s_manual_disconnect) {
            scan_fallback_attempted = true;
            ESP_LOGW(TAG, "Auto-reconnect: stored address unreachable; trying discovery fallback");
            if (scan_and_connect_once() == ESP_OK) {
                ESP_LOGI(TAG, "Auto-reconnect discovery fallback succeeded");
                break;
            }
        }

        // Exponential backoff: 1s -> 2s -> 4s -> 8s -> 16s -> 30s (capped)
        delay_ms = std::min(delay_ms * 2, kReconnectMaxMs);
    }

    if (s_conn_status == ConnectionStatus::RECONNECTING) {
        // All attempts exhausted, go back to DISCONNECTED
        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            s_conn_info.status = ConnectionStatus::DISCONNECTED;
            s_conn_status = ConnectionStatus::DISCONNECTED;
            xSemaphoreGive(s_mutex);
        }
        if (s_status_cb) {
            s_status_cb(ConnectionStatus::DISCONNECTED, s_call_state);
        }
        s_connection_attempt_in_progress.store(false, std::memory_order_relaxed);
        ESP_LOGW(TAG, "Auto-reconnect exhausted all %d attempts", kMaxReconnectAttempts);
    }

    s_reconnect_task_hdl = nullptr;
    s_reconnect_task_active.store(false, std::memory_order_release);
    vTaskDelete(nullptr);
}

static void trigger_auto_reconnect() {
    bool expected = false;
    if (!s_reconnect_task_active.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) return;

    BaseType_t ret = xTaskCreatePinnedToCore(
        reconnect_task_fn, "bt-reconn",
        3072, nullptr,
        5,  // Low priority: don't interfere with audio
        &s_reconnect_task_hdl,
        0   // Core 0 (protocol core)
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Auto-reconnect task create failed");
        s_reconnect_task_active.store(false, std::memory_order_release);
        s_reconnect_task_hdl = nullptr;
    }
}

static void cancel_reconnect_task() {
    s_reconnect_task_active.store(false, std::memory_order_release);
    s_connection_attempt_in_progress.store(false, std::memory_order_release);

    if (s_reconnect_task_hdl != nullptr) {
        // The reconnect task is implemented as a standalone task; deleting the
        // handle here is the actual cancellation point. Calling this function
        // recursively would recurse until stack exhaustion and leave the BT stack
        // in a permanently stale reconnect state.
        vTaskDelete(s_reconnect_task_hdl);
        s_reconnect_task_hdl = nullptr;
    }
}

static void boot_connect_task(void* arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    if (s_bt_initialized && s_conn_status == ConnectionStatus::DISCONNECTED &&
        !s_manual_disconnect &&
        !s_connection_attempt_in_progress.load(std::memory_order_acquire)) {
        ESP_LOGI(TAG, "Boot auto-connect: starting controlled retry sequence");
        trigger_auto_reconnect();
    }
    vTaskDelete(nullptr);
}

static esp_err_t scan_and_connect_once() {
    s_connection_attempt_in_progress.store(true, std::memory_order_release);
    const int device_count = scan_devices(5);
    if (device_count <= 0) {
        ESP_LOGW(TAG, "Discovery fallback found no Bluetooth devices");
        s_connection_attempt_in_progress.store(false, std::memory_order_release);
        return ESP_ERR_NOT_FOUND;
    }

    const BluetoothDevice* device = get_scan_result(0);
    if (!device) {
        s_connection_attempt_in_progress.store(false, std::memory_order_release);
        return ESP_ERR_NOT_FOUND;
    }

    esp_bd_addr_t remote;
    std::memcpy(remote, device->bda, sizeof(remote));
    while (xSemaphoreTake(s_connection_semaphore, 0) == pdTRUE) {}
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        s_conn_info.status = ConnectionStatus::CONNECTING;
        s_conn_status = ConnectionStatus::CONNECTING;
        xSemaphoreGive(s_mutex);
    }
    if (s_status_cb) s_status_cb(ConnectionStatus::CONNECTING, s_call_state);

    const esp_err_t request = esp_hf_client_connect(remote);
    if (request != ESP_OK) {
        s_connection_attempt_in_progress.store(false, std::memory_order_release);
        return request;
    }
    if (xSemaphoreTake(s_connection_semaphore, pdMS_TO_TICKS(15000)) != pdTRUE) {
        ESP_LOGW(TAG, "Discovery fallback timed out waiting for HFP result");
        s_connection_attempt_in_progress.store(false, std::memory_order_release);
        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
            s_conn_info.status = ConnectionStatus::DISCONNECTED;
            s_conn_status = ConnectionStatus::DISCONNECTED;
            xSemaphoreGive(s_mutex);
        }
        return ESP_ERR_TIMEOUT;
    }
    const bool connected = s_conn_status == ConnectionStatus::CONNECTED ||
                           s_conn_status == ConnectionStatus::SCO_ACTIVE;
    s_connection_attempt_in_progress.store(false, std::memory_order_release);
    return connected ? ESP_OK : ESP_FAIL;
}

// ============================================================================
// Paired Device List (call with s_mutex held)
// ============================================================================

static void refresh_paired_devices_locked() {
    s_paired_devices.clear();
    esp_bd_addr_t dev_list[kMaxBonds];
    int dev_num = kMaxBonds;
    if (esp_bt_gap_get_bond_device_list(&dev_num, dev_list) == ESP_OK) {
        for (int i = 0; i < dev_num; ++i) {
            BluetoothDevice d = {};
            std::memcpy(d.bda, dev_list[i], 6);
            s_paired_devices.push_back(d);
        }
    }
}

// ============================================================================
// SCO Audio Data: Phone -> ESP32 (receive callback)
// Registered via esp_hf_client_register_audio_data_callback()
// Called from BT task context - must be fast, no blocking
// ============================================================================

static void hf_client_audio_rx_cb(esp_hf_sync_conn_hdl_t hdl, esp_hf_audio_buff_t* buf, bool is_bad_frame) {
    (void)hdl;

    // We still notify the TX task even if it's a bad frame, because a bad frame STILL means the 
    // controller processed one SCO time slot and requires one TX frame to keep the balance!
    if (s_sco_bridge_task) {
        xTaskNotifyGive(s_sco_bridge_task);
    }

    if (is_bad_frame || !buf || !buf->data || buf->data_len == 0) {
        if (buf) esp_hf_client_audio_buff_free(buf);
        return;
    }

    const int16_t* pcm_data_ptr = reinterpret_cast<const int16_t*>(buf->data);
    uint16_t pcm_bytes = buf->data_len;

    // With the external codec path, ESP-IDF removes H2 but leaves the mSBC frame.
    OI_INT16 msbc_pcm_buf[120];
    if (s_sco_sample_rate == 16000 && s_msbc_decoder_initialized) {
        if (buf->data_len < ESP_HF_MSBC_ENCODED_FRAME_SIZE) {
            esp_hf_client_audio_buff_free(buf);
            return;
        }
        const OI_BYTE* frame_data = buf->data;
        OI_UINT32 frame_bytes = ESP_HF_MSBC_ENCODED_FRAME_SIZE;
        OI_UINT32 pcm_bytes_out = sizeof(msbc_pcm_buf);
        OI_STATUS decode_status = OI_CODEC_SBC_DecodeFrame(
            &s_sbc_decoder_ctx, &frame_data, &frame_bytes,
            msbc_pcm_buf, &pcm_bytes_out);
        if (decode_status != OI_OK) {
            ESP_LOGD(TAG, "mSBC decode error: %d", decode_status);
            esp_hf_client_audio_buff_free(buf);
            return;
        }
        pcm_data_ptr = msbc_pcm_buf;
        pcm_bytes = static_cast<uint16_t>(pcm_bytes_out);
    }

    if (!s_sco_rx_ringbuf) {
        esp_hf_client_audio_buff_free(buf);
        return;
    }

    const uint16_t num_samples = pcm_bytes / 2;
    if (num_samples > 0) {
        xRingbufferSend(s_sco_rx_ringbuf, pcm_data_ptr, pcm_bytes, 0);

        // Send raw PCM directly to PC via UART transport
        if (s_audio_rx_cb) s_audio_rx_cb(pcm_data_ptr, num_samples);

        ++s_stats.sco_packets_received;
    }

    // MUST free the buffer! The Bluetooth stack allocates it for us and expects us to free it.
    esp_hf_client_audio_buff_free(buf);
}

// ============================================================================
// SCO TX Bridge Task (PC audio -> SCO to phone)
// Runs on Core 1, high priority
// ============================================================================

static void sco_tx_bridge_task(void* arg) {
    ESP_LOGI(TAG, "SCO TX bridge task started (Core %d)", xPortGetCoreID());

    // We allocate a silence frame to cover both CVSD (120) and mSBC cases.
    static uint8_t silence_frame[240];
    memset(silence_frame, 0, sizeof(silence_frame));
    uint8_t pcm_frame[240] = {};

    while (true) {
        // Wait for the RX callback to signal us. This perfectly synchronizes
        // TX and RX clocks and prevents any queue overflow!
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        if (!s_sco_active || s_sync_conn_hdl == 0) {
            continue;
        }

        // Attempt to get audio from PC (via UART, fed into s_sco_tx_ringbuf)
        size_t rx_bytes = 0;
        uint16_t current_frame_size = s_sco_sample_rate == 16000 ? 240 : 120;
        std::memset(pcm_frame, 0, current_frame_size);
        size_t pcm_bytes = 0;
        while (pcm_bytes < current_frame_size) {
            void* item = xRingbufferReceiveUpTo(
                s_sco_tx_ringbuf, &rx_bytes, 0, current_frame_size - pcm_bytes);
            if (!item || rx_bytes == 0) break;
            std::memcpy(pcm_frame + pcm_bytes, item, rx_bytes);
            pcm_bytes += rx_bytes;
            vRingbufferReturnItem(s_sco_tx_ringbuf, item);
        }

        const uint8_t* audio_data = pcm_frame;
        uint16_t audio_len = current_frame_size;

        // Buffer for mSBC encoded frame
        uint8_t msbc_encoded_buf[60];
        
        if (s_sco_sample_rate == 16000 && s_msbc_encoder_initialized) {
            // Encode the complete PCM block to one mSBC frame.
            std::memcpy(s_sbc_encoder_ctx.as16PcmBuffer, audio_data, audio_len);
            s_sbc_encoder_ctx.pu8Packet = msbc_encoded_buf;
            
            SBC_Encoder(&s_sbc_encoder_ctx);
            
            audio_data = msbc_encoded_buf;
            audio_len = ESP_HF_MSBC_ENCODED_FRAME_SIZE;
        }

        // Allocate HFP audio buffer and send
        esp_hf_audio_buff_t* hfp_buf = esp_hf_client_audio_buff_alloc(audio_len);
        if (hfp_buf) {
            hfp_buf->data_len = audio_len;
            std::memcpy(hfp_buf->data, audio_data, audio_len);

            esp_err_t r = esp_hf_client_audio_data_send(s_sync_conn_hdl, hfp_buf);
            
            if (r == ESP_OK) {
                ++s_stats.sco_packets_sent;
                // Buffer is consumed and freed by the Bluetooth stack internally
            } else {
                esp_hf_client_audio_buff_free(hfp_buf); // Only free on error
                if (r != ESP_ERR_INVALID_STATE) {
                    ESP_LOGD(TAG, "SCO TX err: %s", esp_err_to_name(r));
                }
            }
        }
    }
}

// ============================================================================
// GAP Callback
// ============================================================================

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param) {
    switch (event) {

    case ESP_BT_GAP_DISC_RES_EVT: {
        BluetoothDevice dev = {};
        std::memcpy(dev.bda, param->disc_res.bda, 6);
        for (int i = 0; i < param->disc_res.num_prop; ++i) {
            esp_bt_gap_dev_prop_t* p = &param->disc_res.prop[i];
            switch (p->type) {
            case ESP_BT_GAP_DEV_PROP_BDNAME: {
                int len = std::min(static_cast<int>(p->len),
                                   static_cast<int>(sizeof(dev.name)) - 1);
                std::memcpy(dev.name, p->val, len);
                dev.name[len] = '\0';
                break;
            }
            case ESP_BT_GAP_DEV_PROP_RSSI:
                dev.rssi = *reinterpret_cast<int8_t*>(p->val);
                break;
            case ESP_BT_GAP_DEV_PROP_COD:
                dev.cod = *reinterpret_cast<uint32_t*>(p->val);
                break;
            default:
                break;
            }
        }
        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            bool found = false;
            for (auto& d : s_scan_results) {
                if (std::memcmp(d.bda, dev.bda, 6) == 0) { d = dev; found = true; break; }
            }
            if (!found) s_scan_results.push_back(dev);
            xSemaphoreGive(s_mutex);
        }
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (s_scan_semaphore) xSemaphoreGive(s_scan_semaphore);
        }
        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "GAP: ACL connect status=%d handle=%u [%02x:%02x:%02x:%02x:%02x:%02x]",
                 param->acl_conn_cmpl_stat.stat,
                 param->acl_conn_cmpl_stat.handle,
                 param->acl_conn_cmpl_stat.bda[0], param->acl_conn_cmpl_stat.bda[1],
                 param->acl_conn_cmpl_stat.bda[2], param->acl_conn_cmpl_stat.bda[3],
                 param->acl_conn_cmpl_stat.bda[4], param->acl_conn_cmpl_stat.bda[5]);
        {
            char status[96];
            std::snprintf(status, sizeof(status),
                          "{\"event\":\"bt_acl_connect\",\"status\":%d,\"handle\":%u}",
                          param->acl_conn_cmpl_stat.stat,
                          param->acl_conn_cmpl_stat.handle);
            emit_status_message(status);
        }
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGW(TAG, "GAP: ACL disconnected reason=%d handle=%u [%02x:%02x:%02x:%02x:%02x:%02x]",
                 param->acl_disconn_cmpl_stat.reason,
                 param->acl_disconn_cmpl_stat.handle,
                 param->acl_disconn_cmpl_stat.bda[0], param->acl_disconn_cmpl_stat.bda[1],
                 param->acl_disconn_cmpl_stat.bda[2], param->acl_disconn_cmpl_stat.bda[3],
                 param->acl_disconn_cmpl_stat.bda[4], param->acl_disconn_cmpl_stat.bda[5]);
        {
            char status[96];
            std::snprintf(status, sizeof(status),
                          "{\"event\":\"bt_acl_disconnect\",\"reason\":%d,\"handle\":%u}",
                          param->acl_disconn_cmpl_stat.reason,
                          param->acl_disconn_cmpl_stat.handle);
            emit_status_message(status);
        }
        break;

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "GAP: Auth OK [%02x:%02x:%02x:%02x:%02x:%02x]",
                     param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                     param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                     param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
            save_last_mac(param->auth_cmpl.bda);
            if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                refresh_paired_devices_locked();
                xSemaphoreGive(s_mutex);
            }
            char status[96];
            std::snprintf(status, sizeof(status),
                          "{\"event\":\"bt_auth_result\",\"ok\":true,\"status\":%d}",
                          param->auth_cmpl.stat);
            emit_status_message(status);
        } else {
            ESP_LOGW(TAG, "GAP: Auth FAILED (%d)", param->auth_cmpl.stat);
            char status[96];
            std::snprintf(status, sizeof(status),
                          "{\"event\":\"bt_auth_result\",\"ok\":false,\"status\":%d}",
                          param->auth_cmpl.stat);
            emit_status_message(status);
        }
        break;

    case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
        ESP_LOGI(TAG, "GAP: Bond removed");
        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            refresh_paired_devices_locked();
            xSemaphoreGive(s_mutex);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT:
        // Just Works: auto-reply with empty PIN (never reached with IO_CAP_NONE,
        // kept as fallback for legacy BT 2.0 devices that don't support SSP).
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4,
                             (esp_bt_pin_code_t){'0','0','0','0'});
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        // Auto-confirm SSP (Secure Simple Pairing) - no physical display needed
        ESP_LOGI(TAG, "GAP: SSP confirm (numval=%lu) -> accepted",
                 static_cast<unsigned long>(param->cfm_req.num_val));
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        // Just Works: passkey notification never appears with IO_CAP_NONE.
        // Silently ignored.
        break;

    default:
        break;
    }
}

// ============================================================================
// HFP HF Callback
// ============================================================================

static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t* param) {
    switch (event) {

    case ESP_HF_CLIENT_CONNECTION_STATE_EVT: {
        const bool connected = param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED ||
                       param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED;
        const bool connecting = param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING;
        ESP_LOGI(TAG, "HFP: %s state=%d peer_feat=0x%08lx chld_feat=0x%08lx [%02x:%02x:%02x:%02x:%02x:%02x]",
                 connected ? "CONNECTED" : (connecting ? "CONNECTING" : "DISCONNECTED"),
             static_cast<int>(param->conn_stat.state),
             static_cast<unsigned long>(param->conn_stat.peer_feat),
             static_cast<unsigned long>(param->conn_stat.chld_feat),
                 param->conn_stat.remote_bda[0], param->conn_stat.remote_bda[1],
                 param->conn_stat.remote_bda[2], param->conn_stat.remote_bda[3],
                 param->conn_stat.remote_bda[4], param->conn_stat.remote_bda[5]);

        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            std::memcpy(s_conn_info.device.bda, param->conn_stat.remote_bda, 6);
                 s_conn_info.status = connected ? ConnectionStatus::CONNECTED
                                 : (connecting
                                      ? (s_reconnect_task_active.load(std::memory_order_acquire)
                                          ? ConnectionStatus::RECONNECTING
                                          : ConnectionStatus::CONNECTING)
                                      : ConnectionStatus::DISCONNECTED);
            s_conn_status = s_conn_info.status;
            xSemaphoreGive(s_mutex);
        }

        if (connected) {
            // Cancel any pending reconnect task
            if (s_reconnect_task_hdl) {
                vTaskDelete(s_reconnect_task_hdl);
                s_reconnect_task_hdl = nullptr;
            }
            s_manual_disconnect = false;
            s_connection_attempt_in_progress.store(false, std::memory_order_relaxed);
            s_reconnect_task_active.store(false, std::memory_order_release);
        } else if (!connecting) {
            s_connection_attempt_in_progress.store(false, std::memory_order_relaxed);
            s_sco_active = false;
            s_sync_conn_hdl = 0;
            s_sco_sample_rate = 0;
            // Re-enable discoverability
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            ++s_stats.disconnects;

            // Auto-reconnect on unexpected disconnect
            if (!s_manual_disconnect) {
                trigger_auto_reconnect();
            }
        }

        // Notify PC of state change
        if (s_status_cb) {
            s_status_cb(s_conn_info.status, s_call_state);
        }
        if (s_connection_semaphore && (connected || !connecting)) {
            xSemaphoreGive(s_connection_semaphore);
        }
        break;
    }

    case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
        const bool sco_on = (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                             param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC);
        const bool wbs    = (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC);

        ESP_LOGI(TAG, "HFP: SCO %s [%s] - pref_frame_size: %d",
                 sco_on ? "ON" : "OFF",
                 wbs ? "mSBC/16kHz" : "CVSD/8kHz",
                 param->audio_stat.preferred_frame_size);

        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
            s_sco_active = sco_on;
            s_sync_conn_hdl = sco_on ? param->audio_stat.sync_conn_handle : 0;
            s_preferred_frame_size = sco_on ? param->audio_stat.preferred_frame_size : 120;
            s_sco_sample_rate = sco_on ? (wbs ? 16000 : 8000) : 0;
            s_conn_info.status = sco_on ? ConnectionStatus::SCO_ACTIVE
                                        : ConnectionStatus::CONNECTED;
            s_conn_status = s_conn_info.status;
            xSemaphoreGive(s_mutex);
        }

        if (sco_on) {
            if (wbs) {
                // Initialize mSBC Decoder
                OI_STATUS dec_st = OI_CODEC_SBC_DecoderReset(
                    &s_sbc_decoder_ctx,
                    s_sbc_decoder_data,
                    CODEC_DATA_WORDS(1, SBC_CODEC_FAST_FILTER_BUFFERS) * sizeof(OI_UINT32),
                    1, 1, FALSE, TRUE
                );
                s_msbc_decoder_initialized = (dec_st == OI_OK);
                if (!s_msbc_decoder_initialized) {
                    ESP_LOGE(TAG, "mSBC Decoder init failed: %d", dec_st);
                }
                
                // Initialize mSBC Encoder
                std::memset(&s_sbc_encoder_ctx, 0, sizeof(s_sbc_encoder_ctx));
                s_sbc_encoder_ctx.s16SamplingFreq = SBC_sf16000;
                s_sbc_encoder_ctx.s16ChannelMode = SBC_MONO;
                s_sbc_encoder_ctx.s16NumOfSubBands = 8;
                s_sbc_encoder_ctx.s16NumOfBlocks = 15;
                s_sbc_encoder_ctx.s16AllocationMethod = SBC_LOUDNESS;
                s_sbc_encoder_ctx.s16BitPool = 26;
                s_sbc_encoder_ctx.sbc_mode = SBC_MODE_MSBC;
                s_sbc_encoder_ctx.s16NumOfChannels = 1;
                SBC_Encoder_Init(&s_sbc_encoder_ctx);
                s_msbc_encoder_initialized = true;
                
                ESP_LOGI(TAG, "mSBC Codec initialized");
            } else {
                s_msbc_decoder_initialized = false;
                s_msbc_encoder_initialized = false;
            }

            // Re-register the callback every time SCO opens (required by ESP-IDF v5)
            esp_hf_client_register_audio_data_callback(hf_client_audio_rx_cb);
        }

        // Notify status callback so PC gets the sample_rate update
        if (s_status_cb) {
            s_status_cb(s_conn_info.status, s_call_state);
        }
        break;
    }

    case ESP_HF_CLIENT_CIND_CALL_EVT:
        s_call_state = (param->call.status == ESP_HF_CALL_STATUS_CALL_IN_PROGRESS)
                     ? CallState::ACTIVE : CallState::IDLE;
        if (s_status_cb) s_status_cb(s_conn_status, s_call_state);
        break;

    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        switch (param->call_setup.status) {
            case ESP_HF_CALL_SETUP_STATUS_INCOMING:
                s_call_state = CallState::INCOMING;
                break;
            case ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING:
            case ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING:
                s_call_state = CallState::OUTGOING;
                break;
            default:
                if (s_call_state != CallState::ACTIVE) s_call_state = CallState::IDLE;
                break;
        }
        if (s_status_cb) s_status_cb(s_conn_status, s_call_state);
        break;

    case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        if (param->call_held.status != ESP_HF_CALL_HELD_STATUS_NONE)
            s_call_state = CallState::HELD;
        if (s_status_cb) s_status_cb(s_conn_status, s_call_state);
        break;

    case ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT: {
        char status[80];
        std::snprintf(status, sizeof(status),
                      "{\"event\":\"signal\",\"value\":%d}",
                      param->signal_strength.value);
        emit_status_message(status);
        break;
    }

    case ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT: {
        char status[80];
        std::snprintf(status, sizeof(status),
                      "{\"event\":\"battery\",\"value\":%d}",
                      param->battery_level.value);
        emit_status_message(status);
        break;
    }

    case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT: {
        char status[256];
        const char* name = param->cops.name ? param->cops.name : "";
        char escaped_name[220] = {};
        escape_json_string(name, escaped_name, sizeof(escaped_name));
        std::snprintf(status, sizeof(status),
                  "{\"event\":\"operator\",\"name\":\"%s\"}", escaped_name);
        emit_status_message(status);
        break;
    }

    case ESP_HF_CLIENT_CLIP_EVT: {
        char status[128];
        const char* number = param->clip.number ? param->clip.number : "";
        char escaped_number[100] = {};
        escape_json_string(number, escaped_number, sizeof(escaped_number));
        std::snprintf(status, sizeof(status),
                  "{\"event\":\"caller_id\",\"number\":\"%s\"}", escaped_number);
        emit_status_message(status);
        break;
    }

    case ESP_HF_CLIENT_CNUM_EVT: {
        char status[128];
        const char* number = param->cnum.number ? param->cnum.number : "";
        char escaped_number[100] = {};
        escape_json_string(number, escaped_number, sizeof(escaped_number));
        std::snprintf(status, sizeof(status),
                  "{\"event\":\"subscriber\",\"number\":\"%s\"}", escaped_number);
            emit_status_message(status);
        break;
    }

    case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "HFP: Volume type=%d vol=%d",
                 param->volume_control.type, param->volume_control.volume);
        if (param->volume_control.type == ESP_HF_VOLUME_CONTROL_TARGET_SPK) {
            s_speaker_volume.store(std::clamp(param->volume_control.volume, 0, 15),
                                   std::memory_order_relaxed);
        }
        break;

    case ESP_HF_CLIENT_PKT_STAT_NUMS_GET_EVT:
        if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            s_stats.sco_packets_received = param->pkt_nums.rx_total;
            s_stats.sco_packets_sent     = param->pkt_nums.tx_total;
            s_stats.sco_packet_loss      = param->pkt_nums.rx_lost;
            xSemaphoreGive(s_mutex);
        }
        break;

    default:
        ESP_LOGD(TAG, "HFP: event %d (unhandled)", event);
        break;
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t init(const char* device_name) {
    if (s_bt_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BT HFP Audio Gateway...");

    // Synchronization primitives
    if (!s_mutex)          s_mutex          = xSemaphoreCreateMutex();
    if (!s_scan_semaphore) s_scan_semaphore = xSemaphoreCreateBinary();
    if (!s_connection_semaphore) s_connection_semaphore = xSemaphoreCreateBinary();
    if (!s_mutex || !s_scan_semaphore || !s_connection_semaphore) {
        ESP_LOGE(TAG, "Semaphore alloc failed");
        return ESP_ERR_NO_MEM;
    }

    // SCO audio ringbuffers
    s_sco_tx_ringbuf = xRingbufferCreate(SCO_TX_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    s_sco_rx_ringbuf = xRingbufferCreate(SCO_RX_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!s_sco_tx_ringbuf || !s_sco_rx_ringbuf) {
        ESP_LOGE(TAG, "SCO ringbuf alloc failed");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = ESP_OK;

#ifdef CONFIG_BT_ENABLED
    // Release BLE memory (Classic BT only)
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "BLE mem release: %s", esp_err_to_name(ret));
    }

    // BT Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT ctrl init: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT ctrl enable: %s", esp_err_to_name(ret));
        return ret;
    }

    // Bluedroid stack
    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bd_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable: %s", esp_err_to_name(ret));
        return ret;
    }

    // Device name
    const char* name = (device_name && device_name[0]) ? device_name : "ESP32-HFP";
    esp_bt_gap_set_device_name(name);

    // GAP callback + SSP security
    ret = esp_bt_gap_register_callback(bt_gap_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP cb register: %s", esp_err_to_name(ret));
        return ret;
    }

    // IO Capability: NONE = "Just Works" SSP mode.
    // Commercial headsets (Jabra, Plantronics, AirPods) all use this.
    // The phone connects without displaying any code or asking for a PIN.
    esp_bt_sp_param_t sp_param = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t   io_cap   = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(sp_param, &io_cap, sizeof(io_cap));

    // HFP HF profile
    ret = esp_hf_client_register_callback(hf_client_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HFP HF cb register: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register audio data RX callback (phone -> ESP32 audio)
    ret = esp_hf_client_register_audio_data_callback(hf_client_audio_rx_cb);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HFP audio RX cb register: %s", esp_err_to_name(ret));
        // Non-fatal: we can still send audio to phone
    }

    ret = esp_hf_client_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HFP HF init: %s", esp_err_to_name(ret));
        return ret;
    }

    // Make discoverable
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Load paired devices
    refresh_paired_devices_locked();

    s_bt_initialized = true;
    s_conn_status    = ConnectionStatus::DISCONNECTED;
    s_call_state     = CallState::IDLE;

    ESP_LOGI(TAG, "BT stack ready | name=%s | HFP HF | WBS=%s",
             name,
             "enabled");

#else
    ESP_LOGW(TAG, "CONFIG_BT_ENABLED not set");
    s_bt_initialized = true;
#endif

    return ESP_OK;
}

esp_err_t start_hfp_audio_gateway() {
    if (!s_bt_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_sco_bridge_task) {
        ESP_LOGW(TAG, "SCO bridge already running");
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        sco_tx_bridge_task, "sco-tx-bridge",
        4096, nullptr,
        configMAX_PRIORITIES - 2,
        &s_sco_bridge_task,
        1 // Core 1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "SCO bridge task create failed");
        return ESP_FAIL;
    }

    if (s_auto_connect && xTaskCreatePinnedToCore(boot_connect_task, "bt-boot-connect",
                                                   3072, nullptr, 5, nullptr, 0) != pdPASS) {
        ESP_LOGW(TAG, "Boot auto-connect task could not be created");
    }

    ESP_LOGI(TAG, "[OK] HFP Audio Gateway started (Core 1)");
    return ESP_OK;
}

esp_err_t stop_hfp_audio_gateway() {
    if (s_sco_bridge_task) {
        vTaskDelete(s_sco_bridge_task);
        s_sco_bridge_task = nullptr;
    }
    s_sco_active    = false;
    s_sync_conn_hdl = 0;
    return ESP_OK;
}

ConnectionStatus get_connection_status() { return s_conn_status; }
CallState        get_call_state()        { return s_call_state; }

esp_err_t start_sco() {
    if (!s_bt_initialized || s_conn_status == ConnectionStatus::DISCONNECTED)
        return ESP_ERR_INVALID_STATE;
    esp_bd_addr_t remote;
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        std::memcpy(remote, s_conn_info.device.bda, 6);
        xSemaphoreGive(s_mutex);
    } else return ESP_ERR_TIMEOUT;
    return esp_hf_client_connect_audio(remote);
}

esp_err_t stop_sco() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    esp_bd_addr_t remote;
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        std::memcpy(remote, s_conn_info.device.bda, 6);
        xSemaphoreGive(s_mutex);
    } else return ESP_ERR_TIMEOUT;
    return esp_hf_client_disconnect_audio(remote);
}

size_t send_sco_audio(const int16_t* samples, size_t sample_count) {
    if (!samples || sample_count == 0 || !s_sco_tx_ringbuf || !s_sco_active) return 0;
    size_t bytes = sample_count * sizeof(int16_t);
    return (xRingbufferSend(s_sco_tx_ringbuf, samples, bytes, 0) == pdTRUE)
           ? sample_count : 0;
}

size_t receive_sco_audio(int16_t* out_samples, size_t max_samples) {
    if (!out_samples || max_samples == 0 || !s_sco_rx_ringbuf || !s_sco_active) return 0;

    size_t rx_bytes = 0;
    // BLOCKING read: perfectly syncs with the Bluetooth hardware clock.
    // Timeout of 100ms ensures we wait for data instead of polling.
    void* item = xRingbufferReceiveUpTo(s_sco_rx_ringbuf, &rx_bytes, pdMS_TO_TICKS(100), max_samples * sizeof(int16_t));
    if (item && rx_bytes > 0) {
        std::memcpy(out_samples, item, rx_bytes);
        vRingbufferReturnItem(s_sco_rx_ringbuf, item);
        return rx_bytes / sizeof(int16_t);
    }
    return 0;
}

bool is_sco_active() { return s_sco_active; }

int get_sco_sample_rate() { return s_sco_sample_rate; }

const HFPConnectionInfo& get_connection_info() { return s_conn_info; }
const BluetoothStats&    get_stats()            { return s_stats; }

int scan_devices(uint32_t timeout_s) {
    if (!s_bt_initialized || !s_scan_semaphore || !s_mutex) return 0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        s_scan_results.clear();
        xSemaphoreGive(s_mutex);
    }
    uint8_t inq_len = static_cast<uint8_t>(
        std::max(1, std::min(0x30, static_cast<int>((timeout_s + 0.64) / 1.28))));
    if (esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, inq_len, 0) != ESP_OK)
        return 0;
    xSemaphoreTake(s_scan_semaphore, pdMS_TO_TICKS((timeout_s + 2) * 1000));
    int count = 0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        count = static_cast<int>(s_scan_results.size());
        xSemaphoreGive(s_mutex);
    }
    return count;
}

const BluetoothDevice* get_scan_result(size_t index) {
    static BluetoothDevice temp = {};
    if (!s_mutex || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return nullptr;
    if (index >= s_scan_results.size()) { xSemaphoreGive(s_mutex); return nullptr; }
    temp = s_scan_results[index];
    xSemaphoreGive(s_mutex);
    return &temp;
}

esp_err_t pair_device(const uint8_t* bda, const char* pin) {
    if (!bda || !s_bt_initialized) return ESP_ERR_INVALID_ARG;
    if (pin && pin[0]) {
        esp_bt_pin_code_t pin_code = {};
        size_t L = std::min(strlen(pin), sizeof(pin_code));
        std::memcpy(pin_code, pin, L);
        esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, static_cast<uint8_t>(L), pin_code);
    } else {
        esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, nullptr);
    }
    esp_bd_addr_t remote;
    std::memcpy(remote, bda, 6);
    return esp_hf_client_connect(remote);
}

esp_err_t unpair_device(const uint8_t* bda) {
    if (!bda) return ESP_ERR_INVALID_ARG;
    esp_bd_addr_t remote;
    std::memcpy(remote, bda, 6);
    return esp_bt_gap_remove_bond_device(remote);
}

int get_paired_devices_count() {
    if (!s_mutex || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return 0;
    int count = static_cast<int>(s_paired_devices.size());
    xSemaphoreGive(s_mutex);
    return count;
}

const BluetoothDevice* get_paired_device(size_t index) {
    static BluetoothDevice tmp = {};
    if (!s_mutex || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return nullptr;
    if (index >= s_paired_devices.size()) { xSemaphoreGive(s_mutex); return nullptr; }
    tmp = s_paired_devices[index];
    xSemaphoreGive(s_mutex);
    return &tmp;
}

esp_err_t set_auto_connect(bool enabled) {
    s_auto_connect = enabled;
    return ESP_OK;
}

esp_err_t disconnect() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    s_manual_disconnect = true;
    // Cancel any pending reconnect
    cancel_reconnect_task();
    esp_bd_addr_t remote;
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        std::memcpy(remote, s_conn_info.device.bda, 6);
        xSemaphoreGive(s_mutex);
    } else return ESP_ERR_TIMEOUT;
    return esp_hf_client_disconnect(remote);
}

bool is_initialized() { return s_bt_initialized; }

esp_err_t set_device_name(const char* name) {
    if (!name || name[0] == '\0') return ESP_ERR_INVALID_ARG;
    return esp_bt_gap_set_device_name(name);
}

esp_err_t deinit() {
    ESP_LOGI(TAG, "Deinitializing BT HFP...");
    stop_hfp_audio_gateway();
    if (s_reconnect_task_hdl) {
        vTaskDelete(s_reconnect_task_hdl);
        s_reconnect_task_hdl = nullptr;
    }

    if (!s_bt_initialized) return ESP_OK;

#ifdef CONFIG_BT_ENABLED
    esp_hf_client_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
#endif

    if (s_sco_tx_ringbuf) { vRingbufferDelete(s_sco_tx_ringbuf); s_sco_tx_ringbuf = nullptr; }
    if (s_sco_rx_ringbuf) { vRingbufferDelete(s_sco_rx_ringbuf); s_sco_rx_ringbuf = nullptr; }
    if (s_mutex)          { vSemaphoreDelete(s_mutex);           s_mutex = nullptr; }
    if (s_scan_semaphore) { vSemaphoreDelete(s_scan_semaphore);  s_scan_semaphore = nullptr; }
    if (s_connection_semaphore) { vSemaphoreDelete(s_connection_semaphore); s_connection_semaphore = nullptr; }

    s_bt_initialized = false;
    ESP_LOGI(TAG, "BT deinitialized");
    return ESP_OK;
}

// ============================================================================
// Application integration callbacks
// ============================================================================

void register_status_callback(StatusCallback cb) {
    s_status_cb = cb;
}

void register_audio_rx_callback(AudioRxCallback cb) {
    s_audio_rx_cb = cb;
}

void register_status_message_callback(StatusMessageCallback cb) {
    s_status_message_cb = cb;
}

void feed_audio(const int16_t* samples, size_t num_samples) {
    if (!samples || num_samples == 0 || !s_sco_tx_ringbuf) return;
    const size_t bytes = num_samples * sizeof(int16_t);
    // Non-blocking: if ringbuf full, drop oldest by design (BYTEBUF is FIFO)
    xRingbufferSend(s_sco_tx_ringbuf, samples, bytes, 0);
}

int get_volume() {
    return s_conn_info.status != ConnectionStatus::DISCONNECTED
               ? s_speaker_volume.load(std::memory_order_relaxed) : 0;
}

esp_err_t set_volume(int vol) {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    vol = std::max(0, std::min(15, vol));
    const esp_err_t result = esp_hf_client_volume_update(ESP_HF_VOLUME_CONTROL_TARGET_SPK, vol);
    if (result == ESP_OK) s_speaker_volume.store(vol, std::memory_order_relaxed);
    return result;
}

esp_err_t answer_call() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    const esp_err_t result = esp_hf_client_answer_call();
    if (result == ESP_OK) ESP_LOGI(TAG, "Answer call");
    return result;
}

esp_err_t dial_number(const char* number) {
    if (!s_bt_initialized || !number || number[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (s_conn_status == ConnectionStatus::DISCONNECTED)
        return ESP_ERR_INVALID_STATE;

    size_t length = std::strlen(number);
    if (length > 32) return ESP_ERR_INVALID_ARG;
    for (size_t i = 0; i < length; ++i) {
        const char c = number[i];
        if (!((c >= '0' && c <= '9') || c == '+' || c == '*' || c == '#'))
            return ESP_ERR_INVALID_ARG;
    }
    return esp_hf_client_dial(number);
}

esp_err_t send_dtmf(char code) {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    const bool valid = (code >= '0' && code <= '9') || code == '*' || code == '#' ||
                       (code >= 'A' && code <= 'D');
    return valid ? esp_hf_client_send_dtmf(code) : ESP_ERR_INVALID_ARG;
}

esp_err_t query_current_calls() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    return esp_hf_client_query_current_calls();
}

esp_err_t query_operator() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    return esp_hf_client_query_current_operator_name();
}

esp_err_t query_subscriber_number() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    return esp_hf_client_retrieve_subscriber_info();
}

esp_err_t hangup_call() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    const esp_err_t result = esp_hf_client_reject_call();
    if (result == ESP_OK) ESP_LOGI(TAG, "Hangup/reject call");
    return result;
}

esp_err_t reconnect_last() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    if (s_conn_status != ConnectionStatus::DISCONNECTED ||
        s_connection_attempt_in_progress.load(std::memory_order_relaxed) ||
        s_reconnect_task_active.load(std::memory_order_acquire)) return ESP_ERR_INVALID_STATE;
    cancel_reconnect_task();
    s_manual_disconnect = false;
    uint8_t bda[6] = {};
    if (load_last_mac(bda)) {
        esp_bd_addr_t remote;
        std::memcpy(remote, bda, 6);
        ESP_LOGI(TAG, "Reconnecting to last device...");
        s_connection_attempt_in_progress.store(true, std::memory_order_relaxed);
        const esp_err_t result = esp_hf_client_connect(remote);
        if (result != ESP_OK) {
            s_connection_attempt_in_progress.store(false, std::memory_order_relaxed);
        }
        return result;
    } else {
        ESP_LOGW(TAG, "No last device in NVS");
        return ESP_ERR_NOT_FOUND;
    }
}

esp_err_t start_scan_and_connect() {
    if (!s_bt_initialized) return ESP_ERR_INVALID_STATE;
    if (s_conn_status != ConnectionStatus::DISCONNECTED ||
        s_connection_attempt_in_progress.load(std::memory_order_relaxed)) {
        ESP_LOGW(TAG, "HFP connection attempt already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_reconnect_task_active.load(std::memory_order_acquire)) {
        ESP_LOGW(TAG, "HFP reconnect task already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    cancel_reconnect_task();
    s_manual_disconnect = false;
    ESP_LOGI(TAG, "Starting scan before HFP connection...");
    const int device_count = scan_devices(10);
    if (device_count <= 0) {
        ESP_LOGW(TAG, "Bluetooth scan found no devices");
        return ESP_ERR_NOT_FOUND;
    }

    const BluetoothDevice* device = get_scan_result(0);
    if (!device) {
        ESP_LOGW(TAG, "Bluetooth scan result was unavailable");
        return ESP_ERR_NOT_FOUND;
    }

    esp_bd_addr_t remote;
    std::memcpy(remote, device->bda, sizeof(remote));
    ESP_LOGI(TAG, "Connecting to scanned device [%02x:%02x:%02x:%02x:%02x:%02x]",
             remote[0], remote[1], remote[2], remote[3], remote[4], remote[5]);
    while (xSemaphoreTake(s_connection_semaphore, 0) == pdTRUE) {}
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        s_conn_info.status = ConnectionStatus::CONNECTING;
        s_conn_status = ConnectionStatus::CONNECTING;
        xSemaphoreGive(s_mutex);
    }
    if (s_status_cb) {
        s_status_cb(ConnectionStatus::CONNECTING, s_call_state);
    }
    s_connection_attempt_in_progress.store(true, std::memory_order_relaxed);
    const esp_err_t result = esp_hf_client_connect(remote);
    if (result != ESP_OK) {
        s_connection_attempt_in_progress.store(false, std::memory_order_relaxed);
        ESP_LOGW(TAG, "HFP connection request failed: %s", esp_err_to_name(result));
        return result;
    }
    if (xSemaphoreTake(s_connection_semaphore, pdMS_TO_TICKS(15000)) != pdTRUE) {
        ESP_LOGW(TAG, "HFP connection timed out waiting for connection event");
        return ESP_ERR_TIMEOUT;
    }
    return s_conn_status == ConnectionStatus::CONNECTED ? ESP_OK : ESP_FAIL;
}
} // namespace bluetooth_hfp
