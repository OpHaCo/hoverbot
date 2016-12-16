#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

/** In arduino we cannot specify any project specific define,
 * this hack defines log level, file needing logs have just to 
 * include it */

/** Global defined must be defined here  */
#define LOG_LEVEL LOG_LEVEL_DEBUG
#include <logger.h>

#endif /** GLOBAL_DEFINES_H */
