/*
 * max31865.c
 *
 *  Created on: Mar 25, 2026
 *      Author: Karukosa
 */

#include "max31865.h"

#include <math.h>

#define MAX31865_WRITE_ADDR_MASK 0x80U

static uint8_t max31865ReadRegisterN(Max31865Handle *handle, uint8_t addr, uint8_t *buffer, uint8_t n);
static uint8_t max31865ReadRegister8(Max31865Handle *handle, uint8_t addr, uint8_t *value);
static uint8_t max31865WriteRegister8(Max31865Handle *handle, uint8_t addr, uint8_t value);

void Max31865_Init(Max31865Handle *handle,
                   SPI_HandleTypeDef *spi,
                   GPIO_TypeDef *csPort,
                   uint16_t csPin,
                   float rRefOhms,
                   float rNominalOhms)
{
    if (handle == NULL) {
        return;
    }

    handle->spi = spi;
    handle->csPort = csPort;
    handle->csPin = csPin;
    handle->rRefOhms = rRefOhms;
    handle->rNominalOhms = rNominalOhms;
    handle->wires = MAX31865_2WIRE;
    handle->filter50Hz = 1U;

    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
}

uint8_t Max31865_Begin(Max31865Handle *handle, Max31865NumWires wires, uint8_t filter50Hz)
{
    if (handle == NULL || handle->spi == NULL) {
        return 0U;
    }

    handle->filter50Hz = (filter50Hz != 0U) ? 1U : 0U;
    Max31865_SetWires(handle, wires);
    Max31865_Enable50Hz(handle, handle->filter50Hz);
    Max31865_ClearFault(handle);

    return 1U;
}

void Max31865_SetWires(Max31865Handle *handle, Max31865NumWires wires)
{
    uint8_t config;

    if (handle == NULL || max31865ReadRegister8(handle, MAX31865_CONFIG_REG, &config) == 0U) {
        return;
    }

    if (wires == MAX31865_3WIRE) {
        config |= MAX31865_CONFIG_3WIRE;
    }
    else {
        config &= (uint8_t)~MAX31865_CONFIG_3WIRE;
    }

    handle->wires = wires;
    (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
}

void Max31865_Enable50Hz(Max31865Handle *handle, uint8_t enable50Hz)
{
    uint8_t config;

    if (handle == NULL || max31865ReadRegister8(handle, MAX31865_CONFIG_REG, &config) == 0U) {
        return;
    }

    if (enable50Hz != 0U) {
        config |= MAX31865_CONFIG_FILT50HZ;
    }
    else {
        config &= (uint8_t)~MAX31865_CONFIG_FILT50HZ;
    }

    handle->filter50Hz = (enable50Hz != 0U) ? 1U : 0U;
    (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
}

void Max31865_EnableBias(Max31865Handle *handle, uint8_t enable)
{
    uint8_t config;

    if (handle == NULL || max31865ReadRegister8(handle, MAX31865_CONFIG_REG, &config) == 0U) {
        return;
    }

    if (enable != 0U) {
        config |= MAX31865_CONFIG_BIAS;
    }
    else {
        config &= (uint8_t)~MAX31865_CONFIG_BIAS;
    }

    (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
}

void Max31865_AutoConvert(Max31865Handle *handle, uint8_t enable)
{
    uint8_t config;

    if (handle == NULL || max31865ReadRegister8(handle, MAX31865_CONFIG_REG, &config) == 0U) {
        return;
    }

    if (enable != 0U) {
        config |= MAX31865_CONFIG_MODEAUTO;
    }
    else {
        config &= (uint8_t)~MAX31865_CONFIG_MODEAUTO;
    }

    (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
}

void Max31865_ClearFault(Max31865Handle *handle)
{
    uint8_t config;

    if (handle == NULL || max31865ReadRegister8(handle, MAX31865_CONFIG_REG, &config) == 0U) {
        return;
    }

    config &= (uint8_t)~0x2CU;
    config |= MAX31865_CONFIG_FAULTSTAT;

    (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
}

uint8_t Max31865_ReadFault(Max31865Handle *handle, Max31865FaultCycle faultCycle)
{
    uint8_t fault = 0U;

    if (handle == NULL) {
        return 0U;
    }

    if (faultCycle == MAX31865_FAULT_AUTO) {
        Max31865_ClearFault(handle);
        (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG,
                                     (uint8_t)(MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO |
                                               ((handle->filter50Hz != 0U) ? MAX31865_CONFIG_FILT50HZ : 0U)));
        HAL_Delay(1U);
    }
    else if (faultCycle == MAX31865_FAULT_MANUAL_RUN) {
        uint8_t config = (uint8_t)(MAX31865_CONFIG_BIAS |
                                   ((handle->filter50Hz != 0U) ? MAX31865_CONFIG_FILT50HZ : 0U));
        (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
        HAL_Delay(1U);
    }
    else if (faultCycle == MAX31865_FAULT_MANUAL_FINISH) {
        (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG,
                                     (uint8_t)(((handle->filter50Hz != 0U) ? MAX31865_CONFIG_FILT50HZ : 0U)));
    }

    (void)max31865ReadRegister8(handle, MAX31865_FAULTSTAT_REG, &fault);
    return fault;
}

void Max31865_SetThresholds(Max31865Handle *handle, uint16_t lower, uint16_t upper)
{
    if (handle == NULL) {
        return;
    }

    (void)max31865WriteRegister8(handle, MAX31865_HFAULTMSB_REG, (uint8_t)(upper >> 8));
    (void)max31865WriteRegister8(handle, MAX31865_HFAULTLSB_REG, (uint8_t)(upper & 0xFFU));
    (void)max31865WriteRegister8(handle, MAX31865_LFAULTMSB_REG, (uint8_t)(lower >> 8));
    (void)max31865WriteRegister8(handle, MAX31865_LFAULTLSB_REG, (uint8_t)(lower & 0xFFU));
}

uint16_t Max31865_ReadRTD(Max31865Handle *handle)
{
    uint8_t buffer[2] = {0, 0};
    uint8_t config;
    uint16_t rtd;

    if (handle == NULL || max31865ReadRegister8(handle, MAX31865_CONFIG_REG, &config) == 0U) {
        return 0U;
    }

    Max31865_ClearFault(handle);

    config |= MAX31865_CONFIG_BIAS;
    (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
    HAL_Delay(10U);

    config |= MAX31865_CONFIG_1SHOT;
    (void)max31865WriteRegister8(handle, MAX31865_CONFIG_REG, config);
    HAL_Delay(65U);

    if (max31865ReadRegisterN(handle, MAX31865_RTDMSB_REG, buffer, 2U) == 0U) {
        return 0U;
    }

    rtd = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    rtd >>= 1;

    Max31865_EnableBias(handle, 0U);

    return rtd;
}

float Max31865_CalculateTemperature(uint16_t rawRtd, float rtdNominal, float refResistor)
{
    float Z1, Z2, Z3, Z4, Rt, temp;

    Rt = (float)rawRtd;
    Rt /= 32768.0f;
    Rt *= refResistor;

    Z1 = -3.9083e-3f;
    Z2 = 3.9083e-3f * 3.9083e-3f - (4.0f * -5.775e-7f);
    Z3 = (4.0f * -5.775e-7f) / rtdNominal;
    Z4 = 2.0f * -5.775e-7f;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrtf(temp) + Z1) / Z4;

    if (temp >= 0.0f) {
        return temp;
    }

    Rt /= rtdNominal;
    Rt *= 100.0f;

    temp = -242.02f;
    temp += 2.2228f * Rt;
    temp += 2.5859e-3f * Rt * Rt;
    temp -= 4.8260e-6f * Rt * Rt * Rt;
    temp -= 2.8183e-8f * Rt * Rt * Rt * Rt;
    temp += 1.5243e-10f * Rt * Rt * Rt * Rt * Rt;

    return temp;
}

uint8_t Max31865_ReadTemperatureC(Max31865Handle *handle, float *temperatureC)
{
    uint16_t rawRtd;

    if (handle == NULL || temperatureC == NULL) {
        return 0U;
    }

    rawRtd = Max31865_ReadRTD(handle);
    *temperatureC = Max31865_CalculateTemperature(rawRtd, handle->rNominalOhms, handle->rRefOhms);

    return 1U;
}

uint8_t Max31865_ReadTemperatureTenthsC(Max31865Handle *handle, int16_t *temperatureTenths)
{
    float temperatureC;

    if (temperatureTenths == NULL) {
        return 0U;
    }

    if (Max31865_ReadTemperatureC(handle, &temperatureC) == 0U) {
        return 0U;
    }

    *temperatureTenths = (int16_t)(temperatureC * 10.0f);
    return 1U;
}

static uint8_t max31865ReadRegisterN(Max31865Handle *handle, uint8_t addr, uint8_t *buffer, uint8_t n)
{
    if (handle == NULL || buffer == NULL || n == 0U) {
        return 0U;
    }

    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(handle->spi, &addr, 1U, 20U) != HAL_OK) {
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
        return 0U;
    }

    if (HAL_SPI_Receive(handle->spi, buffer, n, 20U) != HAL_OK) {
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
        return 0U;
    }

    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
    return 1U;
}

static uint8_t max31865ReadRegister8(Max31865Handle *handle, uint8_t addr, uint8_t *value)
{
    return max31865ReadRegisterN(handle, addr, value, 1U);
}

static uint8_t max31865WriteRegister8(Max31865Handle *handle, uint8_t addr, uint8_t value)
{
    uint8_t txData[2];

    if (handle == NULL) {
        return 0U;
    }

    txData[0] = addr | MAX31865_WRITE_ADDR_MASK;
    txData[1] = value;

    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(handle->spi, txData, 2U, 20U) != HAL_OK) {
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);
        return 0U;
    }
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, GPIO_PIN_SET);

    return 1U;
}
