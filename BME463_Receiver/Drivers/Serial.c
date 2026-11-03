#include "Serial.h"


static volatile bool s_usb_ready = false;
static volatile bool initial_print = false;

static repeating_timer_t s_banner_timer;
static const char *s_app_desc = NULL;
static const char *s_name     = NULL;


void serial_init(void) {
    // Do NOT block here. Just reset state.
    s_usb_ready = false;
}

void serial_poll(void) {
    // Non-blocking: once USB is connected, latch it true forever.
    if (!s_usb_ready && stdio_usb_connected()) {
        s_usb_ready = true;
        // Optional: announce once the terminal opens
        // printf("\r\n[USB serial connected]\r\n");
        // fflush(stdout);
    }
}

bool serial_usb_ready(void) {
    return s_usb_ready;
}

void cdc_print(const char *fmt, ...) {
    if (!s_usb_ready) return;

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
}

void cdc_println(const char *fmt, ...) {
    if (!s_usb_ready) return;

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    // Use CRLF for terminals that expect it
    printf("\r\n");
    fflush(stdout);
}

void cdc_initial_print(const char *fmt, ...) {
    if (!s_usb_ready) return;
    if (initial_print) return; // Only print once, ignore subsequent calls

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
}

void cdc_initial_println(const char *fmt, ...) {
    if (!s_usb_ready) return;
    if (initial_print) return; // Only print once, ignore subsequent calls
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    // Use CRLF for terminals that expect it
    printf("\r\n");
    fflush(stdout);
}

void first_print_flag(void) {
    if (!s_usb_ready) return;
    initial_print = true;
}


static bool banner_timer_cb(repeating_timer_t *t) {
    (void)t;

    // Not connected yet -> keep trying
    if (!stdio_usb_connected()) return true;

    // Latch ready and print exactly once
    s_usb_ready = true;

    // Print banner (CRLF safe)
    if (s_app_desc) cdc_println("\r\n%s\r", s_app_desc);
    if (s_name)     cdc_println("Name: %s\r", s_name);
    cdc_println("\r\n");
    fflush(stdout);

    // Return false to STOP the repeating timer
    return false;
}

void serial_start_boot_banner(const char *app_desc, const char *name) {
    s_usb_ready = false;
    s_app_desc = app_desc;
    s_name = name;

    // Poll every 50ms from alarm IRQ context
    add_repeating_timer_ms(50, banner_timer_cb, NULL, &s_banner_timer);
}
