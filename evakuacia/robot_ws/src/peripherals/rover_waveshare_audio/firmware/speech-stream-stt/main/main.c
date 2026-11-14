#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "bsp_board.h"
#include "rgb_led_driver.h"
#include "tca9555_driver.h"

#define PCM_SAMPLE_RATE          16000
#define PCM_FRAME_SAMPLES        320
#define PCM_INPUT_CHANNELS       4
#define STREAM_TASK_STACK_WORDS  4096
#define STREAM_TASK_PRIORITY     5
#define PLAYBACK_TASK_STACK_WORDS 4096
#define PLAYBACK_TASK_PRIORITY    4
#define BUTTON_TASK_STACK_WORDS   4096
#define BUTTON_TASK_PRIORITY      3
#define STREAM_MAGIC             0x314D4350u  /* "PCM1" little-endian */
#define PLAYBACK_MAGIC           0x314B5053u  /* "SPK1" little-endian */
#define PTT_BUTTON_MASK          IO_EXPANDER_PIN_NUM_10
#define VOLUME_UP_BUTTON_MASK    IO_EXPANDER_PIN_NUM_9
#define VOLUME_DOWN_BUTTON_MASK  IO_EXPANDER_PIN_NUM_11
#define PTT_RELEASE_GRACE_MS     2000
#define USB_STREAM_TX_BUFFER_SIZE 4096
#define USB_STREAM_RX_BUFFER_SIZE 4096
#define USB_PLAYBACK_READ_CHUNK  256
#define USB_PLAYBACK_BUFFER_CAPACITY 4096
#define USB_PLAYBACK_MAX_SAMPLES 1024
#define PLAYBACK_CHANNELS        2
#define PLAYBACK_IDLE_MUTE_MS    120
#define PLAYBACK_UNMUTE_SETTLE_MS 10
#define PLAYBACK_MUTE_SETTLE_MS  10
#define PLAYBACK_PA_SETTLE_MS    10
#define PLAYBACK_VOLUME          95
#define PLAYBACK_LOCK_TIMEOUT_MS 500
#define BUTTON_POLL_MS           20
#define BUTTON_DEBOUNCE_SAMPLES  2
#define VOLUME_STEP              10
#define VOLUME_MIN               0
#define VOLUME_MAX               100
#define FEEDBACK_TONE_DURATION_MS 120
#define FEEDBACK_TONE_FADE_MS    12
#define FEEDBACK_TONE_CHUNK_FRAMES 160
#define FEEDBACK_TONE_AMPLITUDE  9000.0f
#define FEEDBACK_TONE_FREQ_MIN_HZ 550.0f
#define FEEDBACK_TONE_FREQ_MAX_HZ 1450.0f
#define FEEDBACK_TONE_MUTED_HZ   420.0f
#define FEEDBACK_MUTED_VOLUME    8
#define PI_F                     3.14159265358979323846f

static const char *TAG = "pcm_stream";
static SemaphoreHandle_t s_playback_mutex = NULL;
static bool s_playback_muted = true;
static int s_playback_volume = PLAYBACK_VOLUME;
static int64_t s_last_playback_activity_us = 0;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t sample_rate;
    uint16_t sample_count;
    uint32_t sequence;
} pcm_frame_header_t;

static int usb_stream_write_all(const void *src, size_t size, TickType_t ticks_to_wait)
{
    const uint8_t *cursor = (const uint8_t *)src;
    size_t remaining = size;

    while (remaining > 0) {
        int written = usb_serial_jtag_write_bytes(cursor, remaining, ticks_to_wait);
        if (written <= 0) {
            break;
        }
        cursor += written;
        remaining -= written;
    }

    return (int)(size - remaining);
}

static void usb_stream_init(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = {
            .tx_buffer_size = USB_STREAM_TX_BUFFER_SIZE,
            .rx_buffer_size = USB_STREAM_RX_BUFFER_SIZE,
        };
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
    }
}

static void send_stream_banner(void)
{
    static const char banner[] =
        "\nPCM_STREAM_READY v1 sr=16000 mono=s16le frame=320 format=framed\n";
    usb_stream_write_all(banner, sizeof(banner) - 1, pdMS_TO_TICKS(20));
}

static bool is_button_pressed(uint32_t mask)
{
    return !Read_EXIO(mask);
}

static bool is_ptt_button_pressed(void)
{
    /* User button on the TCA9555 is active low. */
    return is_button_pressed(PTT_BUTTON_MASK);
}

static bool should_stream_audio(void)
{
    static int64_t stream_active_until_us = 0;
    int64_t now_us = esp_timer_get_time();

    if (is_ptt_button_pressed()) {
        stream_active_until_us = now_us + (PTT_RELEASE_GRACE_MS * 1000LL);
        return true;
    }

    return now_us < stream_active_until_us;
}

static bool is_valid_audio_header(const pcm_frame_header_t *header, uint32_t expected_magic)
{
    return header->magic == expected_magic &&
           header->sample_rate == PCM_SAMPLE_RATE &&
           header->sample_count > 0 &&
           (header->sample_count % PLAYBACK_CHANNELS) == 0 &&
           header->sample_count <= USB_PLAYBACK_MAX_SAMPLES;
}

static int find_magic_offset(const uint8_t *buffer, size_t buffer_len, uint32_t magic)
{
    const uint8_t *magic_bytes = (const uint8_t *)&magic;
    for (size_t i = 0; i + sizeof(magic) <= buffer_len; ++i) {
        if (memcmp(buffer + i, magic_bytes, sizeof(magic)) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static void update_stream_indicator(bool usb_connected, bool stream_enabled)
{
    typedef enum {
        STREAM_INDICATOR_DISCONNECTED = 0,
        STREAM_INDICATOR_READY,
        STREAM_INDICATOR_ACTIVE,
    } stream_indicator_state_t;

    static stream_indicator_state_t last_state = -1;
    stream_indicator_state_t next_state = STREAM_INDICATOR_DISCONNECTED;

    if (!usb_connected) {
        next_state = STREAM_INDICATOR_DISCONNECTED;
    } else if (stream_enabled) {
        next_state = STREAM_INDICATOR_ACTIVE;
    } else {
        next_state = STREAM_INDICATOR_READY;
    }

    if (next_state == last_state) {
        return;
    }

    switch (next_state) {
        case STREAM_INDICATOR_DISCONNECTED:
            set_rgb_color(RGB_COLOR_WHITE);
            set_rgb_mode(RGB_MODE_IDLE);
            break;
        case STREAM_INDICATOR_READY:
            set_rgb_color(RGB_COLOR_WHITE);
            set_rgb_mode(RGB_MODE_IDLE);
            break;
        case STREAM_INDICATOR_ACTIVE:
            set_rgb_color(RGB_COLOR_GREEN);
            set_rgb_mode(RGB_MODE_REC_COMMAND);
            break;
    }

    last_state = next_state;
}

static void set_playback_amp_enabled(bool enabled)
{
    static bool current_state = false;

    if (enabled == current_state) {
        return;
    }

    Set_EXIO(IO_EXPANDER_PIN_NUM_8, enabled);
    vTaskDelay(pdMS_TO_TICKS(PLAYBACK_PA_SETTLE_MS));
    current_state = enabled;
}

static bool lock_playback(TickType_t ticks_to_wait)
{
    return s_playback_mutex != NULL &&
           xSemaphoreTake(s_playback_mutex, ticks_to_wait) == pdTRUE;
}

static void unlock_playback(void)
{
    if (s_playback_mutex != NULL) {
        xSemaphoreGive(s_playback_mutex);
    }
}

static void begin_playback_output_locked(void)
{
    if (s_playback_muted) {
        set_playback_amp_enabled(true);
        esp_audio_set_play_mute(false);
        vTaskDelay(pdMS_TO_TICKS(PLAYBACK_UNMUTE_SETTLE_MS));
        s_playback_muted = false;
    }
}

static void stop_playback_output_locked(void)
{
    if (s_playback_muted) {
        return;
    }

    esp_audio_play_silence(40);
    esp_audio_set_play_mute(true);
    vTaskDelay(pdMS_TO_TICKS(PLAYBACK_MUTE_SETTLE_MS));
    set_playback_amp_enabled(false);
    s_playback_muted = true;
}

static esp_err_t write_playback_samples_locked(const int16_t *samples, size_t payload_bytes, TickType_t ticks_to_wait)
{
    begin_playback_output_locked();

    esp_err_t ret = esp_audio_play(samples, payload_bytes, ticks_to_wait);
    if (ret == ESP_OK) {
        s_last_playback_activity_us = esp_timer_get_time();
    }
    return ret;
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float volume_to_feedback_frequency(int volume)
{
    float normalized = (float)clamp_int(volume, VOLUME_MIN, VOLUME_MAX) / (float)VOLUME_MAX;
    return FEEDBACK_TONE_FREQ_MIN_HZ +
           ((FEEDBACK_TONE_FREQ_MAX_HZ - FEEDBACK_TONE_FREQ_MIN_HZ) * normalized);
}

static esp_err_t play_feedback_tone_locked(float frequency_hz, uint32_t duration_ms)
{
    int16_t chunk[FEEDBACK_TONE_CHUNK_FRAMES * PLAYBACK_CHANNELS];
    const size_t total_frames = ((size_t)PCM_SAMPLE_RATE * duration_ms) / 1000U;
    const size_t fade_frames = ((size_t)PCM_SAMPLE_RATE * FEEDBACK_TONE_FADE_MS) / 1000U;
    const float phase_increment = (2.0f * PI_F * frequency_hz) / (float)PCM_SAMPLE_RATE;
    float phase = 0.0f;

    for (size_t frame_offset = 0; frame_offset < total_frames; frame_offset += FEEDBACK_TONE_CHUNK_FRAMES) {
        size_t frames_this_chunk = FEEDBACK_TONE_CHUNK_FRAMES;
        if (frame_offset + frames_this_chunk > total_frames) {
            frames_this_chunk = total_frames - frame_offset;
        }

        for (size_t i = 0; i < frames_this_chunk; ++i) {
            size_t frame_index = frame_offset + i;
            float envelope = 1.0f;

            if (fade_frames > 0 && frame_index < fade_frames) {
                envelope = (float)frame_index / (float)fade_frames;
            }

            if (fade_frames > 0) {
                size_t frames_to_end = total_frames - frame_index - 1;
                if (frames_to_end < fade_frames) {
                    float release_envelope = (float)frames_to_end / (float)fade_frames;
                    if (release_envelope < envelope) {
                        envelope = release_envelope;
                    }
                }
            }

            int16_t sample = (int16_t)(sinf(phase) * FEEDBACK_TONE_AMPLITUDE * envelope);
            chunk[(i * PLAYBACK_CHANNELS) + 0] = sample;
            chunk[(i * PLAYBACK_CHANNELS) + 1] = sample;

            phase += phase_increment;
            if (phase >= (2.0f * PI_F)) {
                phase -= (2.0f * PI_F);
            }
        }

        esp_err_t ret = write_playback_samples_locked(
            chunk,
            frames_this_chunk * PLAYBACK_CHANNELS * sizeof(int16_t),
            pdMS_TO_TICKS(50));
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

static void set_runtime_playback_volume_locked(int volume)
{
    s_playback_volume = clamp_int(volume, VOLUME_MIN, VOLUME_MAX);
    esp_err_t ret = esp_audio_set_play_vol(s_playback_volume);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set playback volume to %d: %s", s_playback_volume, esp_err_to_name(ret));
    }
}

static void play_volume_feedback_locked(int target_volume)
{
    int indicator_volume = target_volume;
    float indicator_frequency = volume_to_feedback_frequency(target_volume);

    if (target_volume == 0) {
        indicator_volume = FEEDBACK_MUTED_VOLUME;
        indicator_frequency = FEEDBACK_TONE_MUTED_HZ;
    }

    esp_err_t ret = esp_audio_set_play_vol(indicator_volume);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set feedback volume to %d: %s", indicator_volume, esp_err_to_name(ret));
        return;
    }

    ret = play_feedback_tone_locked(indicator_frequency, FEEDBACK_TONE_DURATION_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to play volume feedback tone: %s", esp_err_to_name(ret));
    }

    if (indicator_volume != target_volume) {
        ret = esp_audio_set_play_vol(target_volume);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restore target volume %d: %s", target_volume, esp_err_to_name(ret));
        }
    }
}

static void change_playback_volume(int delta)
{
    if (!lock_playback(pdMS_TO_TICKS(PLAYBACK_LOCK_TIMEOUT_MS))) {
        ESP_LOGW(TAG, "Timed out waiting for playback lock during volume change");
        return;
    }

    int next_volume = clamp_int(s_playback_volume + delta, VOLUME_MIN, VOLUME_MAX);
    set_runtime_playback_volume_locked(next_volume);
    play_volume_feedback_locked(s_playback_volume);
    stop_playback_output_locked();

    ESP_LOGI(TAG, "Playback volume is now %d", s_playback_volume);
    unlock_playback();
}

static inline int16_t mix_mics_to_mono(const int16_t *raw_frame, size_t index)
{
    int32_t mic_left = raw_frame[index * PCM_INPUT_CHANNELS + 1];
    int32_t mic_right = raw_frame[index * PCM_INPUT_CHANNELS + 3];
    int32_t mixed = (mic_left + mic_right) / 2;
    if (mixed > INT16_MAX) {
        mixed = INT16_MAX;
    } else if (mixed < INT16_MIN) {
        mixed = INT16_MIN;
    }
    return (int16_t)mixed;
}

static void pcm_stream_task(void *arg)
{
    size_t raw_bytes = PCM_FRAME_SAMPLES * PCM_INPUT_CHANNELS * sizeof(int16_t);
    int16_t *raw = heap_caps_malloc(raw_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    int16_t *mono = heap_caps_malloc(PCM_FRAME_SAMPLES * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    uint32_t sequence = 0;
    bool banner_sent = false;

    if (!raw || !mono) {
        ESP_LOGE(TAG, "Failed to allocate audio buffers");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        bool usb_connected = usb_serial_jtag_is_connected();
        if (!usb_connected) {
            banner_sent = false;
            update_stream_indicator(false, false);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (!banner_sent) {
            send_stream_banner();
            banner_sent = true;
        }

        ESP_ERROR_CHECK(esp_get_feed_data(true, raw, raw_bytes));
        bool stream_enabled = should_stream_audio();
        update_stream_indicator(true, stream_enabled);
        if (!stream_enabled) {
            continue;
        }

        for (size_t i = 0; i < PCM_FRAME_SAMPLES; ++i) {
            mono[i] = mix_mics_to_mono(raw, i);
        }

        pcm_frame_header_t header = {
            .magic = STREAM_MAGIC,
            .sample_rate = PCM_SAMPLE_RATE,
            .sample_count = PCM_FRAME_SAMPLES,
            .sequence = sequence++,
        };

        int header_written = usb_stream_write_all(&header, sizeof(header), 0);
        int payload_written = usb_stream_write_all(mono, PCM_FRAME_SAMPLES * sizeof(int16_t), 0);

        if (header_written != sizeof(header) || payload_written != PCM_FRAME_SAMPLES * (int)sizeof(int16_t)) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

static void volume_button_task(void *arg)
{
    typedef struct {
        uint32_t mask;
        int delta;
        bool last_raw_pressed;
        bool stable_pressed;
        uint8_t stable_samples;
    } volume_button_state_t;

    volume_button_state_t buttons[] = {
        { .mask = VOLUME_UP_BUTTON_MASK, .delta = VOLUME_STEP },
        { .mask = VOLUME_DOWN_BUTTON_MASK, .delta = -VOLUME_STEP },
    };

    for (;;) {
        uint32_t levels = 0;
        esp_err_t ret = esp_io_expander_get_level(
            io_expander,
            VOLUME_UP_BUTTON_MASK | VOLUME_DOWN_BUTTON_MASK,
            &levels);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read volume buttons: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
            continue;
        }

        for (size_t i = 0; i < sizeof(buttons) / sizeof(buttons[0]); ++i) {
            bool raw_pressed = (levels & buttons[i].mask) == 0;

            if (raw_pressed != buttons[i].last_raw_pressed) {
                buttons[i].last_raw_pressed = raw_pressed;
                buttons[i].stable_samples = 1;
            } else if (buttons[i].stable_samples < BUTTON_DEBOUNCE_SAMPLES) {
                buttons[i].stable_samples++;
            }

            if (buttons[i].stable_samples >= BUTTON_DEBOUNCE_SAMPLES &&
                buttons[i].stable_pressed != raw_pressed) {
                buttons[i].stable_pressed = raw_pressed;
                if (raw_pressed) {
                    change_playback_volume(buttons[i].delta);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

static void pcm_playback_task(void *arg)
{
    uint8_t *buffer = heap_caps_malloc(USB_PLAYBACK_BUFFER_CAPACITY, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    uint8_t *chunk = heap_caps_malloc(USB_PLAYBACK_READ_CHUNK, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    int16_t *samples = heap_caps_malloc(USB_PLAYBACK_MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    size_t buffered = 0;

    if (!buffer || !chunk || !samples) {
        ESP_LOGE(TAG, "Failed to allocate playback buffers");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        int bytes_read = usb_serial_jtag_read_bytes(chunk, USB_PLAYBACK_READ_CHUNK, pdMS_TO_TICKS(20));
        if (bytes_read <= 0) {
            if (lock_playback(0)) {
                if (!s_playback_muted) {
                    int64_t idle_us = esp_timer_get_time() - s_last_playback_activity_us;
                    if (idle_us >= (PLAYBACK_IDLE_MUTE_MS * 1000LL)) {
                        stop_playback_output_locked();
                    }
                }
                unlock_playback();
            }
            continue;
        }

        if (buffered + (size_t)bytes_read > USB_PLAYBACK_BUFFER_CAPACITY) {
            buffered = 0;
        }

        memcpy(buffer + buffered, chunk, bytes_read);
        buffered += (size_t)bytes_read;

        while (buffered > 0) {
            int start = find_magic_offset(buffer, buffered, PLAYBACK_MAGIC);
            if (start < 0) {
                if (buffered > sizeof(uint32_t) - 1) {
                    size_t keep = sizeof(uint32_t) - 1;
                    memmove(buffer, buffer + buffered - keep, keep);
                    buffered = keep;
                }
                break;
            }

            if (start > 0) {
                memmove(buffer, buffer + start, buffered - (size_t)start);
                buffered -= (size_t)start;
            }

            if (buffered < sizeof(pcm_frame_header_t)) {
                break;
            }

            pcm_frame_header_t header;
            memcpy(&header, buffer, sizeof(header));
            if (!is_valid_audio_header(&header, PLAYBACK_MAGIC)) {
                memmove(buffer, buffer + 1, buffered - 1);
                buffered -= 1;
                continue;
            }

            size_t payload_bytes = (size_t)header.sample_count * sizeof(int16_t);
            size_t packet_size = sizeof(header) + payload_bytes;
            if (buffered < packet_size) {
                break;
            }

            memcpy(samples, buffer + sizeof(header), payload_bytes);
            if (lock_playback(pdMS_TO_TICKS(PLAYBACK_LOCK_TIMEOUT_MS))) {
                esp_err_t ret = write_playback_samples_locked(samples, payload_bytes, pdMS_TO_TICKS(50));
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Playback write failed: %s", esp_err_to_name(ret));
                }
                unlock_playback();
            } else {
                ESP_LOGW(TAG, "Dropped playback chunk while waiting for playback lock");
            }

            if (buffered > packet_size) {
                memmove(buffer, buffer + packet_size, buffered - packet_size);
            }
            buffered -= packet_size;
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_board_init(PCM_SAMPLE_RATE, 2, 16));
    tca9555_driver_init();
    RGB_Example();
    set_rgb_color(RGB_COLOR_WHITE);
    set_rgb_mode(RGB_MODE_IDLE);
    set_playback_amp_enabled(false);
    usb_stream_init();
    s_playback_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_playback_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(esp_audio_set_play_vol(PLAYBACK_VOLUME));
    ESP_ERROR_CHECK(esp_audio_set_play_mute(true));

    ESP_LOGI(TAG, "Starting PCM stream task");
    ESP_LOGI(TAG, "Output: 16 kHz mono s16le over USB serial");
    ESP_LOGI(TAG, "Frame size: %d samples", PCM_FRAME_SAMPLES);
    ESP_LOGI(TAG, "Push-to-talk: hold user button on EXIO10, release grace: %d ms", PTT_RELEASE_GRACE_MS);
    ESP_LOGI(TAG, "Playback volume: %d, auto mute after %d ms idle", PLAYBACK_VOLUME, PLAYBACK_IDLE_MUTE_MS);
    ESP_LOGI(TAG, "Playback input: 16 kHz stereo s16le over USB serial");
    ESP_LOGI(TAG, "Volume buttons: EXIO9 louder, EXIO11 quieter, step %d", VOLUME_STEP);

    xTaskCreatePinnedToCore(pcm_stream_task, "pcm_stream", STREAM_TASK_STACK_WORDS, NULL, STREAM_TASK_PRIORITY, NULL, 1);
    xTaskCreatePinnedToCore(pcm_playback_task, "pcm_playback", PLAYBACK_TASK_STACK_WORDS, NULL, PLAYBACK_TASK_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(volume_button_task, "volume_buttons", BUTTON_TASK_STACK_WORDS, NULL, BUTTON_TASK_PRIORITY, NULL, 0);
}
