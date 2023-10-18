#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void init_HR(void);
void HR_startModule(void);
void HR_readMode(void);
void HR_SetOutputMode(void);
void HR_setFifoThreshold(void);
void HR_agcAlgoControl(void);
void HR_max30101Control(void);
void HR_maximFastAlgoControl(void);
void HR_algoSampleAvg(void);
void HR_readAlgoSamples(void);
void HR_readSensorLEDs(void);
void HR_setMFIOInt(void);
void HR_checkStatus(void);
void HR_numSamplesOutFifo(void);
void HR_readFillArray(uint8_t bpmArr[]);
void HR_readBPM(void);
