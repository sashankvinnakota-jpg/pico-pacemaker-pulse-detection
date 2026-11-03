
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
// Drivers/Serial.h
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void serial_init(void);
void serial_poll(void);          
bool serial_usb_ready(void);

void cdc_print(const char *fmt, ...);
void cdc_println(const char *fmt, ...);

void cdc_initial_print(const char *fmt, ...);
void cdc_initial_println(const char *fmt, ...);
void first_print_flag(void);

void serial_start_boot_banner(const char *app_desc, const char *name);

#ifdef __cplusplus
}
#endif