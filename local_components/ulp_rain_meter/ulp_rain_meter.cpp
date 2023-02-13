#include "ulp_rain_meter.h"
#include "ulp_main.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "esp32/ulp.h"

#include "esphome/core/log.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *const TAG = "ulp_rain_meter.sensor";

namespace esphome {
namespace ulp_rain_meter {

/*
 * PUBLIC
*/
void UlpRainMeter::setup()
{
  setupUlpProgram();
}

void UlpRainMeter::update()
{
  ESP_LOGD(TAG, "Entered update.");

  const char* ns = "pulsecnt";
  const char* count_key = "count";

  /* ULP program counts signal edges, convert that to the number of pulses */
  uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
  /* In case of an odd number of edges, keep one until next time */
  ulp_edge_count = ulp_edge_count % 2;
  printf("Pulse count from ULP: %5" PRIu32 "\n", pulse_count_from_ulp);

  this->pulse_count += pulse_count_from_ulp;
  publish_state(this->pulse_count);
}

/*
 * PRIVATE
*/
void UlpRainMeter::setupUlpProgram()
{
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
          (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  /* GPIO used for pulse counting. */
  gpio_num_t gpio_num = GPIO_NUM_0;
  int rtcio_num = rtc_io_number_get(gpio_num);
  assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");

  /* Initialize some variables used by ULP program.
    * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
    * These variables are declared in an auto generated header file,
    * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
    * These variables are located in RTC_SLOW_MEM and can be accessed both by the
    * ULP and the main CPUs.
    *
    * Note that the ULP reads only the lower 16 bits of these variables.
    */
  ulp_debounce_counter = 3;
  ulp_debounce_max_count = 3;
  ulp_next_edge = 0;
  ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
  ulp_edge_count_to_wake_up = 10;

  /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
  rtc_gpio_init(gpio_num);
  rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio_num);
  rtc_gpio_pullup_dis(gpio_num);
  rtc_gpio_hold_en(gpio_num);

#if CONFIG_IDF_TARGET_ESP32
  /* Disconnect GPIO12 and GPIO15 to remove current drain through
    * pullup/pulldown resistors on modules which have these (e.g. ESP32-WROVER)
    * GPIO12 may be pulled high to select flash voltage.
    */
  rtc_gpio_isolate(GPIO_NUM_12);
  rtc_gpio_isolate(GPIO_NUM_15);
#endif // CONFIG_IDF_TARGET_ESP32

  esp_deep_sleep_disable_rom_logging(); // suppress boot messages

  /* Set ULP wake up period to T = 20ms.
    * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
    */
  // ulp_set_wakeup_period(0, 20000);

  /* Start the program */
  err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
  ESP_ERROR_CHECK(err);
}

/*
void UlpRainMeter::setupUlpProgram(uint32_t periodUs)
{
  ESP_LOGD(TAG, "Setting up ULP routine.");

  RTC_SLOW_MEM[12] = 0;
  ulp_set_wakeup_period(0, periodUs);
  const ulp_insn_t  ulp_blink[] = {
    I_MOVI(R3, 12),                         // #12 -> R3
    I_LD(R0, R3, 0),                        // R0 = RTC_SLOW_MEM[R3(#12)] 
    M_BL(1, 1),                             // GOTO M_LABEL(1) IF R0 < 1
    I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 1),  // RTC_GPIO12 = 1
    I_SUBI(R0, R0, 1),                      // R0 = R0 - 1, R0 = 1, R0 = 0
    I_ST(R0, R3, 0),                        // RTC_SLOW_MEM[R3(#12)] = R0
    M_BX(2),                                // GOTO M_LABEL(2)
    M_LABEL(1),                             // M_LABEL(1)
      I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 0),// RTC_GPIO12 = 0
      I_ADDI(R0, R0, 1),                    // R0 = R0 + 1, R0 = 0, R0 = 1
      I_ST(R0, R3, 0),                      // RTC_SLOW_MEM[R3(#12)] = R0
    M_LABEL(2),                             // M_LABEL(2)
    I_HALT()                                // HALT COPROCESSOR
  };
  const gpio_num_t led_gpios[] = {
    GPIO_NUM_2,
    GPIO_NUM_0,
    GPIO_NUM_4
  };
  for (size_t i = 0; i < sizeof(led_gpios) / sizeof(led_gpios[0]); ++i) {
    rtc_gpio_init(led_gpios[i]);
    rtc_gpio_set_direction(led_gpios[i], RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(led_gpios[i], 0);
  }
  size_t size = sizeof(ulp_blink) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(0, ulp_blink, &size);
  ulp_run(0);
}
*/

} // namespace ulp_rain_meter
} // namespace esphome
