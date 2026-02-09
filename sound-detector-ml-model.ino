/* * 1st Code: Full ESP-32 & INMP441 Functionality 
 * Detection of Vehicle Horns (TRUE/FALSE)
 */

#include <SOUND_SENSOR_inferencing.h>
#include "driver/i2s.h"

// I2S Pin Config for INMP441
#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 16000

// Buffer for audio processing
static int16_t sampleBuffer[2048];

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    // Initialize I2S for the INMP441
    setup_i2s();
    Serial.println("System Initialized: Listening for Horns...");
}

void loop() {
    // 1. Prepare signal for Edge Impulse
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;

    // 2. Run Classifier (Inference)
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

    if (r != EI_IMPULSE_OK) {
        return;
    }

    // 3. Logic: Returns TRUE if horn detected above 80% confidence
    // Note: Check Serial monitor to see if 'horn' is index 0 or 1
    float horn_score = result.classification[0].label == "horn" ? 
                       result.classification[0].value : result.classification[1].value;

    if (horn_score > 0.80) {
        Serial.println("TRUE");
    } else {
        Serial.println("FALSE");
    }

    delay(200); // Prevents serial flooding
}

void setup_i2s() {
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64
    };
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
}

int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t bytes_read;
    i2s_read(I2S_PORT, (char *)sampleBuffer, length * 2, &bytes_read, portMAX_DELAY);
    
    // Convert 16-bit PCM to float for the AI model
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (float)sampleBuffer[i];
    }
    return 0;
}