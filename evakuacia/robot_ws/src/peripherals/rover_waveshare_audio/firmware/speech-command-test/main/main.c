#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "bsp_board.h"
#include "rgb_led_driver.h"
#include "tca9555_driver.h"
#include "mic_speech.h"

static char *TAG = "app main";

typedef struct {
    const char *phrase;
    const char *effect;
    RGB_example_color_t color;
} command_info_t;

static const command_info_t k_test_commands[] = {
    [0] = {
        .phrase = "turn on the backlight",
        .effect = "set RGB ring to white",
        .color = RGB_COLOR_WHITE,
    },
    [1] = {
        .phrase = "turn off the backlight",
        .effect = "turn RGB ring off",
        .color = RGB_COLOR_OFF,
    },
    [2] = {
        .phrase = "backlight is brightest",
        .effect = "set RGB ring to green",
        .color = RGB_COLOR_GREEN,
    },
    [3] = {
        .phrase = "backlight is darkest",
        .effect = "set RGB ring to red",
        .color = RGB_COLOR_RED,
    },
};

static void log_test_commands(void)
{
    ESP_LOGI(TAG, "Wake word: hi esp");
    for (size_t i = 0; i < sizeof(k_test_commands) / sizeof(k_test_commands[0]); ++i) {
        ESP_LOGI(TAG, "Command %u: \"%s\" -> %s", (unsigned)i, k_test_commands[i].phrase, k_test_commands[i].effect);
    }
}

static void Speech_event_callback(esp_sr_rec_event_t event,esp_sr_evt_data_t evt_data, void *user_data)
{
    switch (event)
    {
        case ESP_SR_EVT_AWAKEN:
            ESP_LOGI(TAG, "Wake word detected on channel %d, waiting for a command", evt_data.awaken_channel);
            set_rgb_mode(RGB_MODE_REC_COMMAND);
            break;
        case ESP_SR_EVT_CMD:
            if (evt_data.sr_cmd >= 0 && evt_data.sr_cmd < (int)(sizeof(k_test_commands) / sizeof(k_test_commands[0]))) {
                const command_info_t *cmd = &k_test_commands[evt_data.sr_cmd];
                ESP_LOGI(TAG, "Recognized command %d: \"%s\" -> %s", evt_data.sr_cmd, cmd->phrase, cmd->effect);
                set_rgb_color(cmd->color);
                set_rgb_mode(RGB_MODE_IDLE);
            } else {
                ESP_LOGW(TAG, "Recognized unsupported command id %d", evt_data.sr_cmd);
            }
            break;
        case ESP_SR_EVT_CMD_TIMEOUT:
            ESP_LOGI(TAG, "Command timeout, returning to wake-word mode");
            set_rgb_mode(RGB_MODE_IDLE);
            break;
        default:
            break;
    }
}


void app_main()
{
    ESP_ERROR_CHECK(esp_board_init(16000, 2, 16));
    tca9555_driver_init();
    RGB_Example();
    set_rgb_color(RGB_COLOR_BLUE);
    set_rgb_mode(RGB_MODE_IDLE);
    log_test_commands();
    Speech_Init();
    Speech_register_callback(Speech_event_callback);
}
