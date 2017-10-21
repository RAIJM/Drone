#ifndef PEL_LOG_H
#define PEL_LOG_H

#include <Arduino.h>

#ifdef NDEBUG   /* production */
#define pel_log_debug(arg)
#else   /* not NDEBUG */
#define pel_log_debug(arg)      Serial.print(arg)
#endif  /* NDEBUG */
#endif  /* PEL_LOG_H */
