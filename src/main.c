#include <stdio.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "ulp_main.h"

#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "soc/rtc.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static RTC_DATA_ATTR struct timeval sleep_enter_time;

static void init_ulp_program(void);

void app_main(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause())
    {
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_ULP:
    {
        printf("ULP wakeup\n");
        uint32_t HX711HiWord_ulp = (ulp_HX711HiWord & UINT16_MAX);
        printf("Valor ADMSB: %d\n", HX711HiWord_ulp);
        uint32_t HX711LoWord_ulp = (ulp_HX711LoWord & UINT16_MAX);
        printf("Valor ADLSB: %d\n", HX711LoWord_ulp);
        uint32_t HX711Total = (HX711HiWord_ulp << 16) + HX711LoWord_ulp;
        printf("Valor Total: %d\n", HX711Total);
        uint32_t thresholdType_ulp = (ulp_thresholdType & UINT16_MAX);
        printf("Tipo threshold: %d\n", thresholdType_ulp);
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        printf("Not a deep sleep reset, initializing ULP\n");
        init_ulp_program();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = 20;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    const int ext_wakeup_pin_1 = 2;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const int ext_wakeup_pin_2 = 4;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;
    const int ext_wakeup_pin_3 = 15;
    const uint64_t ext_wakeup_pin_3_mask = 1ULL << ext_wakeup_pin_3;

    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2, ext_wakeup_pin_3);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    printf("Enabling ULP wakeup\n");
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

    esp_deep_sleep_start();
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for do input. */
    gpio_num_t gpio_num_addo = GPIO_NUM_25;
    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num_addo);
    rtc_gpio_set_direction(gpio_num_addo, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_addo);
    rtc_gpio_pullup_dis(gpio_num_addo);
    rtc_gpio_hold_en(gpio_num_addo);

    /* GPIO used for sk output. */
    gpio_num_t gpio_num_adsk = GPIO_NUM_26;
    /* Initialize selected GPIO as RTC IO, enable output, disable pullup and pulldown */
    rtc_gpio_init(gpio_num_adsk);
    rtc_gpio_set_direction(gpio_num_adsk, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_adsk);
    rtc_gpio_pullup_dis(gpio_num_adsk);
    rtc_gpio_hold_en(gpio_num_adsk);

    // Acorda quando o valor medido ?? maior que o definido por Over
    ulp_trshHoldOverADMSB = 211;
    ulp_trshHoldOverADLSB = 30196;
    // Acorda quando o valor medido ?? menor que o definido por Under
    ulp_trshHoldUnderADMSB = 209;
    ulp_trshHoldUnderADLSB = 60000;

    ulp_set_wakeup_period(0, 1000000); // Set ULP wake up period T = 1s

    /* Start the program */
    err = ulp_run(&ulp_main - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}