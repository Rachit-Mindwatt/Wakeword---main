#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
//#include "driver/gpio.h"
#include "led_strip.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

/* I2S Microphone Pin Configuration */
#define STD_BCLK        (gpio_num_t) CONFIG_I2S_CLK_GPIO
#define STD_WS          (gpio_num_t) CONFIG_I2S_WS_GPIO
#define STD_DIN         (gpio_num_t) CONFIG_I2S_DATA_GPIO

#define TAG "AINAK_KWS"
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define LED_EVENT_RED   2
#define LED_EVENT_GREEN 1



#define CLASSIFIER_YES 0  // Your AINAK keyword
#define CLASSIFIER_NO  1  // Noise

#define BIT24_MAX_FLOAT 8388607.0f

static i2s_chan_handle_t rx_chan;
static uint32_t *r_buf;

static TaskHandle_t handle_wake_word_task, handle_led_task;
static QueueHandle_t led_queue;

/* LED Controller Struct */
typedef struct {
    uint8_t red, green, blue;
    led_strip_handle_t led_strip;
    bool direction;
    uint8_t s_led_state;
} led_control_t;

/* Set RGB Color */
static void set_led_rgb(led_control_t *led_control) {
    if (led_control->s_led_state) {
        if (led_control->blue < 1 || led_control->blue > 20)
            led_control->direction = !led_control->direction;
        led_control->blue += (led_control->direction ? 1 : -1);
        led_control->red = (led_control->red > 0) ? led_control->red - 5 : 0;
        led_control->green = (led_control->green > 0) ? led_control->green - 5 : 0;
        led_strip_set_pixel(led_control->led_strip, 0, led_control->red, led_control->green, led_control->blue + 4);
        led_strip_refresh(led_control->led_strip);
    } else {
        led_strip_clear(led_control->led_strip);
    }
}

/* LED Initialization */
static void configure_led(led_control_t *led_control) {
    led_control->s_led_state = 1;
    led_control->red = 0;
    led_control->green = 0;
    led_control->blue = 0;
    led_control->direction = 0;

    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_control->led_strip));
    led_strip_clear(led_control->led_strip);
}

/* LED Task */
void led_task(void *args) {
    led_control_t led_control;
    uint8_t event;
    led_queue = xQueueCreate(5, sizeof(uint8_t));
    configure_led(&led_control);

    while (1) {
        if (xQueueReceive(led_queue, &event, 0) == pdTRUE) {
            if (event == LED_EVENT_RED) led_control.red = 255;
            else if (event == LED_EVENT_GREEN) led_control.green = 255;
            led_control.blue = 1;
            led_control.direction = 1;
        }
        set_led_rgb(&led_control);
        vTaskDelay(70 / portTICK_PERIOD_MS);
    }
}

/* I2S Mic Initialization */
static void i2s_init_std_simplex(void) {
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EI_CLASSIFIER_FREQUENCY),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = STD_BCLK,
            .ws = STD_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = STD_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
}

/* Read Microphone Data into float buffer */
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    size_t r_bytes;
    if (i2s_channel_read(rx_chan, r_buf, length * sizeof(uint32_t), &r_bytes, portMAX_DELAY) == ESP_OK) {
        for (size_t i = 0; i < length; i++) {
            int32_t sample = (int32_t)(r_buf[i] >> 8);
            sample = (sample & 0x800000) ? (sample | 0xFF000000) : sample;
            out_ptr[i] = (sample < -BIT24_MAX_FLOAT) ? -1.0f : (sample > BIT24_MAX_FLOAT) ? 1.0f : (float)sample / BIT24_MAX_FLOAT;
        }
    }
    return EIDSP_OK;
}

/* Keyword Detection Task */
static void key_word_task(void *args) {
    signal_t signal;
    ei_impulse_result_t result;
    EI_IMPULSE_ERROR ret;
    uint8_t led_event = 0;

    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;
    r_buf = (uint32_t *)calloc(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(uint32_t));
    assert(r_buf);

    run_classifier_init();
    i2s_init_std_simplex();
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    while (1) {
        ret = run_classifier_continuous(&signal, &result);
        if (ret != EI_IMPULSE_OK) {
            ESP_LOGE(TAG, "Classifier returned error %d", ret);
        } else {
            float score_yes = result.classification[CLASSIFIER_YES].value;
            float score_no = result.classification[CLASSIFIER_NO].value;

            ESP_LOGI(TAG, "AINAK: %.2f | Noise: %.2f", score_yes, score_no);

            if (score_yes > 0.85f) {
                ESP_LOGI(TAG, "✅ Wake word 'AINAK' DETECTED!");
                led_event = LED_EVENT_GREEN;
            } else if (score_no > 0.85f) {
                ESP_LOGI(TAG, "❌ Just noise.");
                led_event = LED_EVENT_RED;
            } else {
                ESP_LOGI(TAG, "🔍 Listening... uncertain.");
                led_event = 0;
            }

            if (led_event) {
                xQueueSend(led_queue, &led_event, 2);
            }
        }
    }

    free(r_buf);
    vTaskDelete(NULL);
}

/* Main App Entry */
extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(key_word_task, "task_keyword", 8192, NULL, 5, &handle_wake_word_task, 0);
    xTaskCreatePinnedToCore(led_task, "task_led", 4096, NULL, 5, &handle_led_task, 1);

    while (1) {
        ESP_LOGD(TAG, "LED Task Watermark: %u", uxTaskGetStackHighWaterMark(handle_led_task));
        ESP_LOGD(TAG, "KWS Task Watermark: %u", uxTaskGetStackHighWaterMark(handle_wake_word_task));
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
