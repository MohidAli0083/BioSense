#ifndef AD8232_H
#define AD8232_H

#include <stdint.h>
#include <stdbool.h>

void ad8232_init(void);
bool ad8232_read_data(uint16_t *ecg_value, bool *lead_off);

#endif // AD8232_H
