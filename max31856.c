/**
 * MAX31856 library to be used with STM32 HAL
 *
 * Author: Can Kocak
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "max31856.h"

#define MAX31856_SPI_TIMEOUT 50U

void max31856_init(max31856_t *max31856)
{
    /*
     * Default:
     * - Thermocouple type: K
     * - Number of samples for average: 1
     * - Cold junction temperature offset: 0
     */
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    max31856->cr1.val = max31856_read_register(max31856, MAX31856_CR1);

    // Enable all faults
    max31856->mask.val = 0;
    max31856_write_register(max31856, MAX31856_MASK, max31856->mask.val);
}

void max31856_set_conversion_mode(max31856_t *max31856, max31856_conversion_mode_t conv_mode)
{
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    max31856->cr0.bits.conv_mode = conv_mode;
    max31856_write_register(max31856, MAX31856_CR0, max31856->cr0.val);
}

void max31856_trigger_one_shot(max31856_t *max31856)
{
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    if (max31856->cr0.bits.conv_mode == CR0_CONV_OFF) {
        max31856->cr0.bits.one_shot_mode = 1;
        max31856_write_register(max31856, MAX31856_CR0, max31856->cr0.val);
        max31856->cr0.bits.one_shot_mode = 0;
    }
}

void max31856_set_noise_filter(max31856_t *max31856, max31856_noise_filter_t noise_filter)
{
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    max31856->cr0.bits.noise_filter = noise_filter;
    max31856_write_register(max31856, MAX31856_CR0, max31856->cr0.val);
}

void max31856_set_cold_junction_enable(max31856_t *max31856, max31856_cj_enable cj_enable)
{
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    max31856->cr0.bits.cj_enable = cj_enable;
    max31856_write_register(max31856, MAX31856_CR0, max31856->cr0.val);
}

void max31856_set_thermocouple_type(max31856_t *max31856, max31856_thermocouple_t therm_typ)
{
    max31856->cr1.val = max31856_read_register(max31856, MAX31856_CR1);
    max31856->cr1.bits.thermo_type = therm_typ;
    max31856_write_register(max31856, MAX31856_CR1, max31856->cr1.val);
}

void max31856_set_average_samples(max31856_t *max31856, max31856_sampling_t samples)
{
    max31856->cr1.val = max31856_read_register(max31856, MAX31856_CR1);
    max31856->cr1.bits.conv_avg_mode = samples;
    max31856_write_register(max31856, MAX31856_CR1, max31856->cr1.val);
}

float max31856_read_TC_temp(max31856_t *max31856)
{
    uint8_t raw_val[3] = {};
    max31856_read_nregisters(max31856, MAX31856_LTCBH, raw_val, 3);
    int32_t raw_val_signed = (raw_val[0] << 16) | (raw_val[1] << 8) | raw_val[0];
    // First 5 bits aren't unused
    raw_val_signed >>= 5;
    return raw_val_signed * 0.0078125;
}

float max31856_read_CJ_temp(max31856_t *max31856)
{
    uint8_t raw_val[2] = {};
    max31856_read_nregisters(max31856, MAX31856_CJTH, raw_val, 2);
    int16_t raw_val_signed = (raw_val[0] << 8) | raw_val[1];
    return raw_val_signed / 256.0;
}

void max31856_set_fault_mode(max31856_t *max31856, max31856_fault_mode_t fault_mode)
{
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    max31856->cr0.bits.fault_mode = fault_mode;
    max31856_write_register(max31856, MAX31856_CR0, max31856->cr0.val);
}

void max31856_set_open_circuit_fault_detection(max31856_t *max31856, max31856_oc_fault_t oc_fault)
{
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    max31856->cr0.bits.oc_fault_enable = oc_fault;
    max31856_write_register(max31856, MAX31856_CR0, max31856->cr0.val);
}

void max31856_read_fault(max31856_t *max31856)
{
    max31856->sr.val = max31856_read_register(max31856, MAX31856_SR);
}

void max31856_clear_fault_status(max31856_t *max31856)
{
    max31856->cr0.val = max31856_read_register(max31856, MAX31856_CR0);
    if (max31856->cr0.bits.fault_mode == CR0_FAULT_INTERRUPT_MODE) {
        max31856->cr0.bits.fault_clear = 1;
        max31856_write_register(max31856, MAX31856_CR0, max31856->cr0.val);
        max31856->cr0.bits.fault_clear = 0;
    }
}

void max31856_write_register(max31856_t *max31856, uint8_t reg_addr, uint8_t reg_val)
{
    reg_addr += 0x80;
    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(max31856->spi_handle, &reg_addr, 1, MAX31856_SPI_TIMEOUT);
    HAL_SPI_Transmit(max31856->spi_handle, &reg_val, 1, MAX31856_SPI_TIMEOUT);
    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_SET);
}

void max31856_write_nregisters(max31856_t *max31856, uint8_t reg_addr, uint8_t *buff, uint16_t len)
{
    reg_addr += 0x80;
    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(max31856->spi_handle, &reg_addr, 1, MAX31856_SPI_TIMEOUT);
    HAL_SPI_Transmit(max31856->spi_handle, buff, len, MAX31856_SPI_TIMEOUT);
    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_SET);
}

uint8_t max31856_read_register(max31856_t *max31856, uint8_t reg_addr)
{
    uint8_t reg_val;

    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(max31856->spi_handle, &reg_addr, 1, MAX31856_SPI_TIMEOUT);
    HAL_SPI_Receive(max31856->spi_handle, &reg_val, 1, MAX31856_SPI_TIMEOUT);
    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_SET);

    return reg_val;
}

void max31856_read_nregisters(max31856_t *max31856, uint8_t reg_addr, uint8_t *buff, uint16_t len)
{
    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(max31856->spi_handle, &reg_addr, 1, MAX31856_SPI_TIMEOUT);
    HAL_SPI_Receive(max31856->spi_handle, buff, len, MAX31856_SPI_TIMEOUT);
    HAL_GPIO_WritePin(max31856->cs_pin.gpio_port, max31856->cs_pin.gpio_pin, GPIO_PIN_SET);
}
