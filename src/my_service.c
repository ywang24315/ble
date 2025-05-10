#include "my_service.h"
#include <zephyr/bluetooth/gatt.h>
#include <string.h>

static struct date_t current_date = {
    .year = 2025,
    .month = 5,
    .day = 6,
};

void my_service_set_date(uint16_t year, uint8_t month, uint8_t day)
{
    // current_date.year = year;
    // current_date.month = month;
    // current_date.day = day;
}