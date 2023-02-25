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

void UlpRainMeter::pulseCount()
{
  	/* Define variables, which go into .bss section (zero-initialized data) */
    .bss
    /* Next input signal edge expected: 0 (negative) or 1 (positive) */
    .global next_edge
  next_edge:
    .long 0

    /* Counter started when signal value changes.
      Edge is "debounced" when the counter reaches zero. */
    .global debounce_counter
  debounce_counter:
    .long 0

    /* Value to which debounce_counter gets reset.
      Set by the main program. */
    .global debounce_max_count
  debounce_max_count:
    .long 0

    /* Total number of signal edges acquired */
    .global edge_count
  edge_count:
    .long 0

    /* Number of edges to acquire before waking up the SoC.
      Set by the main program. */
    .global edge_count_to_wake_up
  edge_count_to_wake_up:
    .long 0

    /* RTC IO number used to sample the input signal.
      Set by main program. */
    .global io_number
  io_number:
    .long 0

    /* Code goes into .text section */
    .text
    .global entry
  entry:
    /* Load io_number */
    move r3, io_number
    ld r3, r3, 0

#if CONFIG_IDF_TARGET_ESP32S2
    /* ESP32S2 powers down RTC periph when entering deep sleep and thus by association SENS_SAR_IO_MUX_CONF_REG */
	WRITE_RTC_FIELD(SENS_SAR_IO_MUX_CONF_REG, SENS_IOMUX_CLK_GATE_EN, 1)
#elif CONFIG_IDF_TARGET_ESP32S3
    /* ESP32S3 powers down RTC periph when entering deep sleep and thus by association SENS_SAR_PERI_CLK_GATE_CONF_REG */
    WRITE_RTC_FIELD(SENS_SAR_PERI_CLK_GATE_CONF_REG, SENS_IOMUX_CLK_EN, 1);
#endif

  const ulp_insn_t ulp_pulse_count[] = {
    /* Lower 16 IOs and higher need to be handled separately,
    * because r0-r3 registers are 16 bit wide.
    * Check which IO this is.
    */
    I_MOVR(r0, r3),           // move dest = src
    M_BRANCH(1),              // jumpr GE
    I_BSGE(0, 16),            // M_BSGE isn't defined in the macros, so these 2 steps are the equivalent

    /* Read the value of lower 16 RTC IOs into R0 */
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16),
    I_RSHR(r0, r0, r3),       // rsh
    M_BX(2),                  // jump label

    /* Read the value of RTC IOs 16-17, into R0 */
    M_LABEL(1), // read_io_high:
      READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2),
      I_SUBI(r3, r3, 16),       // sub immediate
      I_RSHR(r0, r0, r3),

    M_LABEL(2), // read_done:
      I_ANDI(r0, r0, 1),        // and immediate
      /* State of input changed? */
      I_MOVI(r3, next_edge),    // move dest = immediate
      I_LD(r3, r3, 0),          // ld reg reg offset
      I_ADDR(r3, r0, r3),       // add reg reg reg
      I_ANDI(r3, r3, 1),
      M_BXZ(3),                 // branch (jump) to label if ALU is 0
      /* Not changed */
      /* Reset debounce_counter to debounce_max_count */
      I_MOVI(r3, debounce_max_count),
      I_MOVI(r2, debounce_counter),
      I_LD(r3, r3, 0),
      I_ST(r3, r2, 0),          // st reg reg offset
      /* End program */
      I_HALT(),

      // .global changed <- no fucking clue what that does
    M_LABEL(3), // changed:
      /* Input state changed */
      /* Has debounce_counter reached zero? */
      I_MOVI(r3, debounce_counter),
      I_LD(r2, r3, 0),
      /* dummy ADD to use "jump if ALU result is zero" */
      I_ADDI(r2, r2, 0),        // add reg reg immediate
      M_BXZ(4),
      /* Not yet. Decrement debounce_counter */
      I_SUBI(r2, r2, 1),
      I_ST(r2, r3, 0),
      /* End program */
      I_HALT(),

      // .global edge_detected <- no fucking clue what that does
    M_LABEL(4), // edge_detected:
      /* Reset debounce_counter to debounce_max_count */
      I_MOVI(r3, debounce_max_count),
      I_MOVI(r2, debounce_counter),
      I_LD(r3, r3, 0),
      I_ST(r3, r2, 0),
      /* Flip next_edge */
      I_MOVR(r3, next_edge),
      I_LD(r2, r3, 0),
      I_ADDI(r2, r2, 1),
      I_ADDI(r2, r2, 1),
      I_ST(r2, r3, 0),
      /* Increment edge_count */
      I_MOVR(r3, edge_count),
      I_LD(r2, r3, 0),
      I_ADDI(r2, r2, 1),
      I_ST(r2, r3, 0),
      /* Compare edge_count to edge_count_to_wake_up */
      I_MOVI(r3, edge_count_to_wake_up),
      I_LD(r3, r3, 0),
      I_SUBR(r3, r3, r2),       // sub dest leftOp rightOp
      M_BXZ(wake_up)
      /* Not yet. End program */
      I_HALT()
  };
}

} // namespace ulp_rain_meter
} // namespace esphome
