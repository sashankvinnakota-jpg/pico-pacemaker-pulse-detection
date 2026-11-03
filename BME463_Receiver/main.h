/**
 * @file main.h
 * @author Zerind Spore (zspore@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2025-10-8
 * 
 */


#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "Drivers/Filters.h"
#include "Drivers/Serial.h"
#include "Drivers/SevSeg_Driver.h"
#include "hardware/gpio.h"
#include <math.h>
#include "Drivers/DAC_I2C_Driver.h"



extern char NAME[];
extern char APP_DESCRIPTION[];

/*
Function to initilize required hardware within lab, can be left unused
*/
void app_init_hw(void);

/*
Main function call for each lab
*/
void app_main(void);



#endif


