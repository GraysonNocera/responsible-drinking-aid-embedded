#include "stm32f0xx.h"
#include <stdint.h>
#include "HR.h"
#include "I2C.h"
#include "Interrupt.h"
#include "Timers.h"
#include "USART.h"
#include "BAC.h"

void init_ADC(void);
void ADC_read(void);
void PWM_update(void);
void ADC_initBatLEDs(void);
void BAT_changeLEDs(int batLevel);
int BAT_PercLookup(uint16_t batVal);
