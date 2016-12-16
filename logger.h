/******************************************************************************
 * @file    Logger.h 
 * @author  Rémi Pincent - INRIA
 * @date    29 janv. 2014   
 *
 * @brief LOG macros. Memory calls do not occupy memory space
 * when log level below application log level.
 * 
 * Project : logger library
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 * 
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#ifndef LOGGER_H_
#define LOGGER_H_

#include <logging.h>

#ifndef LOG_LEVEL
	#warning "No log level defined "
#endif

#if (LOG_LEVEL >= LOG_LEVEL_NOOUTPUT)
    #define	 LOG_INIT(level) Log.Init(level)
    #define	 LOG_INIT_STREAM(level, stream) Log.Init(level, stream)
    #define	 ASSERT(expr) ((expr) ? (void)0 : Log.Assert(__func__, F(__FILE__), __LINE__, F(#expr)))
#else
    #define	 LOG_INIT(level)
    #define	 LOG_INIT_STREAM(level, stream)
    #define	 ASSERT(expr)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_ERRORS)
	#define LOG_ERROR(msg, 	arguments...) Log.Error(msg, ## arguments)
	#define LOG_ERROR_ID_ARGS(errorId, argsFormat, arguments...) Log.Error(errorId, F(__FILE__), __LINE__, argsFormat, ## arguments)
	#define LOG_ERROR_ID(errorId) Log.Error(errorId, F(__FILE__), __LINE__)
#else
	#define LOG_ERROR(msg, arguments...)
    #define LOG_ERROR_ID(errorId)
	#define LOG_ERROR_ID_ARGS(errorId, argsFormat, arguments...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_INFOS)
	#define LOG_INFO(msg, 	arguments...) Log.Info(msg, ## arguments)
	#define LOG_INFO_LN(msg, 	arguments...) Log.InfoLn(msg, ## arguments)
	#define LOG_INFO_STR(msg) Log.InfoStr(msg)
	#define LOG_INFO_STR_LN(msg) Log.InfoStrLn(msg)
#else
	#define LOG_INFO(msg, arguments...)
	#define LOG_INFO_LN(msg, 	arguments...)
	#define LOG_INFO_STR(msg)
	#define LOG_INFO_STR_LN(msg)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
	#define LOG_DEBUG(msg, 	arguments...) Log.Debug(msg, ## arguments)
	#define LOG_DEBUG_LN(msg, 	arguments...) Log.DebugLn(msg, 	##arguments)
	#define LOG_DEBUG_STR(msg) Log.DebugStr(msg)
	#define LOG_DEBUG_STR_LN(msg) Log.DebugStrLn(msg)
#else
	#define LOG_DEBUG(msg, arguments...)
	#define LOG_DEBUG_LN(msg, 	arguments...)
	#define LOG_DEBUG_STR(msg)
	#define LOG_DEBUG_STR_LN(msg)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_VERBOSE)
	#define LOG_VERBOSE(msg, 	arguments...) Log.Verbose(msg, ## arguments)
	#define LOG_VERBOSE_LN(msg, 	arguments...) Log.VerboseLn(msg, ## arguments)
	#define LOG_VERBOSE_STR(msg) Log.VerboseStr(msg)
	#define LOG_VERBOSE_STR_LN(msg) Log.VerboseStrLn(msg)
#else
	#define LOG_VERBOSE(msg, arguments...)
	#define LOG_VERBOSE_LN(msg, arguments...)
	#define LOG_VERBOSE_STR(msg)
	#define LOG_VERBOSE_STR_LN(msg)
#endif

#endif /* LOGGER_H_ */
