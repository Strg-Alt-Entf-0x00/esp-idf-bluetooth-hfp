# ESP-IDF Bluetooth HFP

An ESP-IDF Bluetooth Classic Hands-Free Profile client component with SCO audio support.

## Status

This is the initial extracted component release (`0.1.0`). It targets the ESP32 Classic Bluetooth stack and requires an ESP-IDF installation with Bluedroid and the SBC codec sources available.

The component exposes Bluetooth and audio callbacks. It has no dependency on an application UART, PC protocol, or project-specific transport layer.

## Requirements

- ESP-IDF 5.x
- ESP32 target with Bluetooth Classic support
- Bluedroid enabled in the project configuration
- Classic Bluetooth HFP client enabled

The current implementation also contains the existing ESP32 DAC playback path. Board-specific audio output should be made configurable before supporting additional hardware targets.

## Integration

Place the repository in an ESP-IDF project's managed component dependencies or add it as a local component. Include the public header:

```cpp
#include "bluetooth_hfp.h"
```

Register callbacks before starting the profile:

```cpp
bluetooth_hfp::register_audio_rx_callback(on_audio_from_phone);
bluetooth_hfp::register_status_message_callback(on_status_message);
bluetooth_hfp::init("My Headset");
bluetooth_hfp::start_hfp_audio_gateway();
```

Feed application PCM samples toward the phone with:

```cpp
bluetooth_hfp::feed_audio(samples, sample_count);
```

See `include/bluetooth_hfp.h` for the complete API and ownership rules.

## License

Licensed under the Apache License, Version 2.0. See `LICENSE` for the full license text.
