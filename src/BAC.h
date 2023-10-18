#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "I2C.h"
#include "Timers.h"

void BAC_readADC(uint8_t adcData[]);
uint16_t BAC_getADCData(void);
static float BAC_ethanolInCo(uint16_t co);
static float BAC_ppmTomgl(uint16_t ppm);
float BAC_getData(void);
void BAC_read(void);
