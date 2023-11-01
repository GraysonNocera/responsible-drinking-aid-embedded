#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

const char* ADD_DRINK = "Add Drink";
const char* SUBTRACT_DRINK = "Subtract Drink";
const char* CLEAR_DRINKS = "Clear Drinks"; //set drink count to 0
const char* DRINK_TIMER_START = "Drink Timer Start"; //when half hour timer starts
const char* DRINK_TIMER_RESET = "Drink Timer Reset"; //when half hour timer is reset either from button triggered ethanol sensor power or 30 mins have passed
const char* ETHANOL_SENSOR_ON = "Ethanol Sensor On"; //ethanol sensor on, will last 20 seconds
const char* ETHANOL_SENSOR_OFF = "Ethanol Sensor Off"; //ethanol sensor has turned off
const char* BAC_READ = "BAC: %d";
const char* ADC_READ = "ADC: %d";
const char* HEART_RATE_READ = "Heart Rate: %d";
