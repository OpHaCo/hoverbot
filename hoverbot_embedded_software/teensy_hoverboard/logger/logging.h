/******************************************************************************
 * @file    logging.h
 * @author  Rémi Pincent - INRIA
 * @date    15 janv. 2014
 *
 * @brief Helper class to get logs over Arduino serial interface
 * based on https://github.com/mrRobot62/Arduino-logging-library
 *
 * Project : logger library
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * All methods are able to handle any number of output parameters.
 * All methods print out a formated string (like printf).<br>
 * To reduce output and program size, reduce loglevel.
 * <br>
 * Output format string can contain below wildcards. Every wildcard
 * must be start with percent sign (\%)
 *
 * <b>Depending on loglevel, source code is excluded from compile !</b><br>
 * <br>
 * <b>Wildcards</b><br>
 * <ul>
 * <li><b>\%s</b>	replace with an string (char*)</li>
 * <li><b>\%c</b>	replace with an character</li>
 * <li><b>\%d</b>	replace with an integer value</li>
 * <li><b>\%l</b>	replace with an long value</li>
 * <li><b>\%f</b>	replace with a float value</li>
 * <li><b>\%x</b>	replace and convert integer value into hex</li>
 * <li><b>\%X</b>	like %x but combine with <b>0x</b>123AB</li>
 * <li><b>\%b</b>	replace and convert integer value into binary</li>
 * <li><b>\%B</b>	like %x but combine with <b>0b</b>10100011</li>
 * <li><b>\%t</b>	replace and convert boolean value into <b>"t"</b> or <b>"f"</b></li>
 * <li><b>\%T</b>	like %t but convert into <b>"true"</b> or <b>"false"</b></li>
 * </ul><br>
 * <b>Loglevels</b><br>
 * <table border="0">
 * <tr><td>0</td><td>LOG_LEVEL_NOOUTPUT</td><td>no output </td></tr>
 * <tr><td>1</td><td>LOG_LEVEL_ERRORS</td><td>only errors </td></tr>
 * <tr><td>2</td><td>LOG_LEVEL_INFOS</td><td>errors and info </td></tr>
 * <tr><td>3</td><td>LOG_LEVEL_DEBUG</td><td>errors, info and debug </td></tr>
 * <tr><td>4</td><td>LOG_LEVEL_VERBOSE</td><td>all </td></tr>
 * </table>
 *
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#ifndef LOGGING_H
#define LOGGING_H

#include <inttypes.h>
#include <stdarg.h>
#include <WString.h>
#include "Stream.h"
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define LOG_LEVEL_NOOUTPUT 0
#define LOG_LEVEL_ERRORS 1
#define LOG_LEVEL_INFOS 2
#define LOG_LEVEL_DEBUG 3
#define LOG_LEVEL_VERBOSE 4

#define CR "\r\n"
#define LOGGING_VERSION 1

class Logging {
private:
	Stream*  _p_output_stream;
    uint8_t _u8_logLevel;
public:
    /*! 
	 * default Constructor
	 */
    Logging(): _p_output_stream(&Serial), _u8_logLevel(LOG_LEVEL_NOOUTPUT){} ;
	

    /**
     * Assert given expression. Abort in case of failure
     * @param func
     * @param file
     * @param lineno
     * @param failedexpr
     */
    void Assert(const char func[], const __FlashStringHelper * file, int lineno, const __FlashStringHelper *expr);

    /**
     * Assert given expression. Abort in case of failure
     * @param func
     * @param file
     * @param lineno
     * @param failedexpr
     */
    void Assert(const char func[], const char file[], int lineno, const char expr[]);

    /** 
	* Initializing, must be called as first.
	* Given stream must have been initialized.
	* e.g : for serial begin() method must have been called
	* \param void
	* \return void
	*
	*/
	void Init(int level = LOG_LEVEL_INFOS, Stream*  _p_output_stream = &Serial);

	
    /**
	* Output an error message. Output message contains
	* ERROR: followed by original msg
	* Error messages are printed out, at every loglevel
	* except 0 ;-)
	* \param msg format string to output
	* \param ... any number of variables
	* \return void
	*/
    void Error(const char msg[], ...);
    void Error( const __FlashStringHelper * msg, ...);

    /**
	* Output an error message. Output message contains
	* ERROR: followed by error id, file, line locations
	* Error messages are printed out, at every loglevel
	* except 0 ;-)
	* \param errorId id of error according to file location
	* \param file where occurred error id of error according to file location
	* \param line in file where occurred error
	* \return void
	*/
    void Error(char errorId, const __FlashStringHelper *, int line);

    /**
	* Output an error message. Output message contains
	* ERROR: followed by error id, file, line locations
	* and error arguments
	* Error messages are printed out, at every loglevel
	* except 0 ;-)
	* \param errorId id of error according to file location
	* \param file where occurred error id of error according to file location
	* \param line in file where occurred error
	* \param argsFormat argument format
	* \param ... any number of variables
	* \return void
	*/
    void Error(char errorId, const __FlashStringHelper *, int line, const __FlashStringHelper * argsFormat, ...);

    /**
	* Output an info message. Output message contains
	* Info messages are printed out at l
	* loglevels >= LOG_LEVEL_INFOS
	*
	* \param msg format string to output
	* \param ... any number of variables
	* \return void
	*/

   void Info(const char msg[], ...);
   void Info( const __FlashStringHelper * msg, ...);
   void InfoLn(const char msg[], ...);
   void InfoLn( const __FlashStringHelper * msg, ...);
   void InfoStr(const __FlashStringHelper * msg);
   void InfoStrLn(const __FlashStringHelper * msg);
	
    /**
	* Output an debug message. Output message contains
	* Debug messages are printed out at l
	* loglevels >= LOG_LEVEL_DEBUG
	*
	* \param msg format string to output
	* \param ... any number of variables
	* \return void
	*/

    void Debug(const char msg[], ...);
    void Debug( const __FlashStringHelper * msg, ...);
    void DebugLn( const __FlashStringHelper * msg, ...);
    void DebugLn(const char msg[], ...);
    void DebugStr(const __FlashStringHelper * msg);
    void DebugStrLn(const __FlashStringHelper * msg);
	
    /**
	* Output an verbose message. Output message contains
	* Debug messages are printed out at l
	* loglevels >= LOG_LEVEL_VERBOSE
	*
	* \param msg format string to output
	* \param ... any number of variables
	* \return void
	*/

    void Verbose(const char msg[], ...);
    void Verbose( const __FlashStringHelper * msg, ...);
    void VerboseLn( const __FlashStringHelper * msg, ...);
    void VerboseStr(const __FlashStringHelper * msg);
    void VerboseStrLn(const __FlashStringHelper * msg);

    
private:
    void print(const char format[], va_list args);
    void print(const __FlashStringHelper * arg_ps8FlashFormatfor, va_list args);
    void printArg(char arg_s8Char, va_list& args);
};

extern Logging Log;
#endif




