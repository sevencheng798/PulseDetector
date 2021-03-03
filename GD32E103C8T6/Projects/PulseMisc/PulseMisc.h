/*HP5804.h*/
#ifndef __PULSE_MISC_H
#define __PULSE_MISC_H
#include <stdint.h>

typedef enum {
  false = 0,
  true = !false
} bool;

/**
 * This is HP sensor read data method.
 */
bool HPReadPressTemp(uint8_t *PreData);

/**
 * This is ZG sensor read data method.
 */
bool ZGReadPressTemp(uint8_t *PreTemData);

/**
 * This is Misc sensor read data method.
 */
//bool ReadPress();

void CreateWorkerTask(void);
void StartWorkerTask(void);
#endif