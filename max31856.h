/**
 * MAX31856 library to be used with STM32 HAL
 *
 * Author: Can Kocak
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef MAX31856_H_
#define MAX31856_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Register addresses */
#define MAX31856_CR0 0x00
#define MAX31856_CR1 0x01
#define MAX31856_MASK 0x02
#define MAX31856_CJHF 0x03   // Default 7Fh
#define MAX31856_CJLF 0x04   // Default C0h
#define MAX31856_LTHFTH 0x05 // Default 7Fh
#define MAX31856_LTHFTL 0x06 // Default FFh
#define MAX31856_LTLFTH 0x07 // Default 80h
#define MAX31856_LTLFTL 0x08 // Default 00h
#define MAX31856_CJTO 0x09   // Default 00h
#define MAX31856_CJTH 0x0A   // Default 00h
#define MAX31856_CJTL 0x0B   // Default 00h
#define MAX31856_LTCBH 0x0C
#define MAX31856_LTCBM 0x0D
#define MAX31856_LTCBL 0x0E
#define MAX31856_SR 0x0F

/* Register bits */
typedef union {
    struct {
        uint8_t noise_filter : 1;    // Default: 0
        uint8_t fault_clear : 1;     // Default: 0
        uint8_t fault_mode : 1;      // Default: 0
        uint8_t cj_enable : 1;       // Default: 0
        uint8_t oc_fault_enable : 2; // Default: 0x0
        uint8_t one_shot_mode : 1;   // Default: 0
        uint8_t conv_mode : 1;       // Default: 0
    } bits;
    uint8_t val; // Default: 0x00
} max31856_cr0_t;

typedef union {
    struct {
        uint8_t thermo_type : 4;   // Default: 0x3 (K-type)
        uint8_t conv_avg_mode : 3; // Default: 0
    } bits;
    uint8_t val; // Default: 0x03
} max31856_cr1_t;

typedef union {
    struct {
        uint8_t oc_fault : 1;          // Default: 0
        uint8_t ov_uv__fault : 1;      // Default: 0
        uint8_t thermo_low_fault : 1;  // Default: 0
        uint8_t thermo_high_fault : 1; // Default: 0
        uint8_t cj_low_fault : 1;      // Default: 0
        uint8_t cj_high_fault : 1;     // Default: 0
        uint8_t thermo_range : 1;      // Default: 0
        uint8_t cj_range : 1;          // Default: 0
    } bits;
    uint8_t val; // Default: 0xFF
} max31856_mask_t;

typedef union {
    struct {
        uint8_t oc_fault : 1;          // Default: 1
        uint8_t ov_uv_fault : 1;       // Default: 1
        uint8_t thermo_low_fault : 1;  // Default: 1
        uint8_t thermo_high_fault : 1; // Default: 1
        uint8_t cj_low_fault : 1;      // Default: 1
        uint8_t cj_high_fault : 1;     // Default: 1
    } bits;
    uint8_t val; // Default: 0x00
} max31856_sr_t; // Read-only

typedef enum {
    CR0_FILTER_OUT_60Hz = 0b0,
    CR0_FILTER_OUT_50Hz = 0b1,
} max31856_noise_filter_t;

typedef enum {
    CR0_FAULT_COMPARATOR_MODE = 0x0,
    CR0_FAULT_INTERRUPT_MODE = 0x1,
} max31856_fault_mode_t;

typedef enum {
    CR0_CJ_ENABLED = 0x0,
    CR0_CJ_DISABLED = 0x1,
} max31856_cj_enable;

typedef enum {
    CR0_OC_DETECT_DISABLED = 0x0,
    CR0_OC_DETECT_ENABLED_R_LESS_5k = 0x1,
    CR0_OC_DETECT_ENABLED_TC_LESS_2ms = 0x2,
    CR0_OC_DETECT_ENABLED_TC_MORE_2ms = 0x3,
} max31856_oc_fault_t;

typedef enum {
    CR0_CONV_OFF = 0x0,
    CR0_CONV_CONTINUOUS = 0x1,
} max31856_conversion_mode_t;

typedef enum {
    CR1_AVG_TC_SAMPLES_1 = 0x0,
    CR1_AVG_TC_SAMPLES_2 = 0x1,
    CR1_AVG_TC_SAMPLES_4 = 0x2,
    CR1_AVG_TC_SAMPLES_8 = 0x3,
    CR1_AVG_TC_SAMPLES_16 = 0x4,
} max31856_sampling_t;

typedef enum {
    CR1_TC_TYPE_B = 0x0,
    CR1_TC_TYPE_E = 0x1,
    CR1_TC_TYPE_J = 0x2,
    CR1_TC_TYPE_K = 0x3,
    CR1_TC_TYPE_N = 0x4,
    CR1_TC_TYPE_R = 0x5,
    CR1_TC_TYPE_S = 0x6,
    CR1_TC_TYPE_T = 0x7,
    CR1_TC_TYPE_VOLT_MODE_GAIN_8 = 0x8,
    CR1_TC_TYPE_VOLT_MODE_GAIN_32 = 0xC,
} max31856_thermocouple_t;

typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
} max31856_gpio_t;

typedef struct {
    SPI_HandleTypeDef *spi_handle; // SCK = 5MHz
    max31856_gpio_t cs_pin;
    max31856_gpio_t fault_pin;
    max31856_gpio_t drdy_pin;
    volatile max31856_cr0_t cr0;
    volatile max31856_cr1_t cr1;
    volatile max31856_mask_t mask;
    volatile max31856_sr_t sr;
} max31856_t;

void max31856_init(max31856_t *max31856);

void max31856_set_conversion_mode(max31856_t *max31856, max31856_conversion_mode_t conv_mode);
void max31856_trigger_one_shot(max31856_t *max31856);

void max31856_set_noise_filter(max31856_t *max31856, max31856_noise_filter_t noise_filter);
void max31856_set_cold_junction_enable(max31856_t *max31856, max31856_cj_enable cj_enable);
void max31856_set_thermocouple_type(max31856_t *max31856, max31856_thermocouple_t therm_typ);
void max31856_set_average_samples(max31856_t *max31856, max31856_sampling_t samples);

float max31856_read_TC_temp(max31856_t *max31856);
float max31856_read_CJ_temp(max31856_t *max31856);

void max31856_set_fault_mode(max31856_t *max31856, max31856_fault_mode_t fault_mode);
void max31856_set_open_circuit_fault_detection(max31856_t *max31856, max31856_oc_fault_t oc_fault);
void max31856_read_fault(max31856_t *max31856);
void max31856_clear_fault_status(max31856_t *max31856);

void max31856_write_register(max31856_t *max31856, uint8_t reg_addr, uint8_t reg_val);
void max31856_write_nregisters(max31856_t *max31856, uint8_t reg_addr, uint8_t *buff, uint16_t len);
uint8_t max31856_read_register(max31856_t *max31856, uint8_t reg_addr);
void max31856_read_nregisters(max31856_t *max31856, uint8_t reg_addr, uint8_t *buff, uint16_t len);

#ifdef __cplusplus
 }
#endif

#endif /* MAX31856_H_ */
