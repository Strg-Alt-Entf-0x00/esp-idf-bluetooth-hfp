#pragma once

// ============================================================================
// Bluetooth HFP component public API.
// ============================================================================

#include <esp_err.h>
#include <cstdint>
#include <cstddef>

namespace bluetooth_hfp {

// ============================================================================
// HFP Profile Constants
// ============================================================================

constexpr uint16_t kHFPServiceUUID = 0x111F;    // Headset Service
constexpr uint32_t kHFPServiceClass = 0x00110100;  // Headset Class

// ============================================================================
// HFP Connection Status
// ============================================================================

enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    SCO_ACTIVE,
    RECONNECTING,
    ERROR
};

enum class CallState {
    IDLE,
    INCOMING,
    OUTGOING,
    ACTIVE,
    HELD
};

// ============================================================================
// Device Information
// ============================================================================

struct BluetoothDevice {
    uint8_t bda[6];                    // Bluetooth Address
    char name[250];                    // Device name
    int rssi;                          // Signal strength
    uint32_t cod;                      // Class of Device
    uint8_t addr_type;                 // Address type
};

struct HFPConnectionInfo {
    BluetoothDevice device;
    ConnectionStatus status;
    CallState call_state;
    bool in_band_ringtone;
    bool waiting_call;
    uint32_t connected_time_s;
};

// ============================================================================
// Statistics
// ============================================================================

struct BluetoothStats {
    uint32_t sco_packets_sent;
    uint32_t sco_packets_received;
    uint32_t sco_packet_loss;
    uint32_t disconnects;
    uint32_t reconnects;
    
    int32_t sco_latency_ms;
    uint8_t sco_packet_loss_percent;
};

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize Bluetooth HFP Audio Gateway
 * @param device_name Name of Bluetooth device
 * @return ESP_OK on success
 */
esp_err_t init(const char* device_name = "ESP32-HFP");

/**
 * @brief Start HFP Audio Gateway (listen for connections)
 * @return ESP_OK on success
 */
esp_err_t start_hfp_audio_gateway();

/**
 * @brief Stop HFP Audio Gateway
 * @return ESP_OK
 */
esp_err_t stop_hfp_audio_gateway();

/**
 * @brief Start SCO (audio) connection with paired device
 * Automatic: called when phone connects via HFP
 * @return ESP_OK if SCO started
 */
esp_err_t start_sco();

/**
 * @brief Stop SCO audio stream
 * @return ESP_OK
 */
esp_err_t stop_sco();

/**
 * @brief Send audio samples via SCO to phone
 * @param samples Audio samples (int16_t PCM)
 * @param sample_count Number of samples
 * @return Samples actually sent (may be less if buffer full)
 */
size_t send_sco_audio(const int16_t* samples, size_t sample_count);

/**
 * @brief Receive audio samples from phone via SCO
 * @param out_buffer Output buffer
 * @param sample_count Requested number of samples
 * @return Samples actually received
 */
size_t receive_sco_audio(int16_t* out_buffer, size_t sample_count);

/**
 * @brief Check if SCO is active
 * @return true if audio stream active
 */
bool is_sco_active();

/**
 * @brief Get the current SCO audio sample rate
 * @return 16000 for mSBC (WBS), 8000 for CVSD, 0 if SCO inactive
 */
int get_sco_sample_rate();

/**
 * @brief Get connection information
 * @return Current connection info
 */
const HFPConnectionInfo& get_connection_info();

/**
 * @brief Get Bluetooth statistics
 * @return Reference to statistics structure
 */
const BluetoothStats& get_stats();

/**
 * @brief Scan for Bluetooth devices
 * @param timeout_s Scan timeout in seconds
 * @return Number of devices found
 */
int scan_devices(uint32_t timeout_s = 10);

/**
 * @brief Get last scan results
 * @param index Result index (0 to scan_count-1)
 * @return Pointer to device or nullptr
 */
const BluetoothDevice* get_scan_result(size_t index);

/**
 * @brief Pair with device (insecure - PIN)
 * @param bda Bluetooth address
 * @param pin PIN code (typically "1234")
 * @return ESP_OK on success
 */
esp_err_t pair_device(const uint8_t* bda, const char* pin = "1234");

/**
 * @brief Unpair device
 * @param bda Bluetooth address
 * @return ESP_OK
 */
esp_err_t unpair_device(const uint8_t* bda);

/**
 * @brief Get paired devices
 * @return Number of paired devices
 */
int get_paired_devices_count();

/**
 * @brief Get paired device info
 * @param index Device index
 * @return Pointer to device info or nullptr
 */
const BluetoothDevice* get_paired_device(size_t index);

/**
 * @brief Set auto-connect mode
 * When enabled, automatically connects to last paired device on boot
 * @param enabled Enable auto-connect
 * @return ESP_OK
 */
esp_err_t set_auto_connect(bool enabled);

/**
 * @brief Disconnect current HFP connection
 * @return ESP_OK
 */
esp_err_t disconnect();

/**
 * @brief Deinitialize Bluetooth stack and controller (disable)
 * @return ESP_OK on success
 */
esp_err_t deinit();

/**
 * @brief Set the local Bluetooth device name at runtime
 * @param name New device name
 * @return ESP_OK on success
 */
esp_err_t set_device_name(const char* name);

/**
 * @brief Check whether the Bluetooth stack is initialized
 * @return true if initialized
 */
bool is_initialized();

// ============================================================================
// Application integration callbacks
// ============================================================================

using AudioRxCallback = void (*)(const int16_t* samples, size_t num_samples);
using StatusMessageCallback = void (*)(const char* message);

void register_audio_rx_callback(AudioRxCallback cb);
void register_status_message_callback(StatusMessageCallback cb);
void feed_audio(const int16_t* samples, size_t num_samples);

/**
 * @brief Status change callback type.
 *        Called on every BT connection or call state change.
 */
using StatusCallback = void (*)(ConnectionStatus, CallState);

/**
 * @brief Register a callback to be notified of BT status changes. 
 */
void register_status_callback(StatusCallback cb);

/**
 * @brief Get current speaker volume (0-15).
 */
int get_volume();

/**
 * @brief Set speaker volume (0-15, clamped).
 */
esp_err_t set_volume(int vol);

/**
 * @brief Answer an incoming call.
 */
esp_err_t answer_call();

/**
 * @brief Place an outgoing call through the connected phone.
 * @param number Null-terminated phone number accepted by the phone.
 */
esp_err_t dial_number(const char* number);

/** @brief Send one DTMF digit through the connected phone. */
esp_err_t send_dtmf(char code);

/** @brief Query current calls from the connected phone. */
esp_err_t query_current_calls();

/** @brief Query the current mobile network operator. */
esp_err_t query_operator();

/** @brief Query the subscriber number reported by the phone. */
esp_err_t query_subscriber_number();

/**
 * @brief Hang up the active or incoming call.
 */
esp_err_t hangup_call();

/**
 * @brief Reconnect to the last known BT device (from NVS).
 */
esp_err_t reconnect_last();

/**
 * @brief Start BT scan and auto-connect to first HFP device found.
 */
esp_err_t start_scan_and_connect();

} // namespace bluetooth_hfp
