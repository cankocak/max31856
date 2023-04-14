# max31856
max31856 library to be used with STM32 HAL library (targeted to be used with STM32CubeMX projects)

Written by Can Kocak. BSD license, check LICENSE for more information All text above must be included in any redistribution

## Usage:
```
#include "max31856.h"
....
max31856_t therm = {&hspi2, {THERM_CS_GPIO_Port, THERM_CS_Pin}};
max31856_init(&therm);
max31856_set_noise_filter(&therm, CR0_FILTER_OUT_50Hz);
max31856_set_cold_junction_enable(&therm, CR0_CJ_ENABLED);
max31856_set_thermocouple_type(&therm, CR1_TC_TYPE_K);
max31856_set_average_samples(&therm, CR1_AVG_TC_SAMPLES_2);
max31856_set_open_circuit_fault_detection(&therm, CR0_OC_DETECT_ENABLED_TC_LESS_2ms);
max31856_set_conversion_mode(&therm, CR0_CONV_CONTINUOUS);
....
max31856_read_fault(&therm);
if (therm.sr.val) {
  /* Handle thermocouple error */
}
float temp = max31856_read_TC_temp(&therm);
```
