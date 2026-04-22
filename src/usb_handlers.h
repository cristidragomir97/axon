#ifndef USB_HANDLERS_H
#define USB_HANDLERS_H

#include <stdint.h>
#include <stdbool.h>

// Initialize USB protocol handlers (call after hardware init)
void usb_handlers_init(void);

// CAN bulk transfer task - bridges USB <-> CAN
// Call from main loop
void can_usb_task(void);

#endif // USB_HANDLERS_H
