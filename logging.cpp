/******************************************************************************
 * @file    logging.cpp
 * @author  Rémi Pincent - INRIA
 * @date    15 janv. 2014
 *
 * @brief Implementation of Logging class
 *
 * Project : logger library
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#include "logging.h"
#include <assert.h>

static const char* ERROR_STR = "ERROR: ";
static const char* BL = "\n";
static const char* IN_FILE = "\nfile: ";
static const char* LINE = "-l";

extern void __assert(const char *__func, const char *__file,
		     int __lineno, const char *__sexp);

void Logging::Init(int level, Stream*  arg_p_output_stream)
{
	_p_output_stream = arg_p_output_stream;
	_u8_logLevel = constrain(level,LOG_LEVEL_NOOUTPUT,LOG_LEVEL_VERBOSE);
}

/**
 * MAP assert on Logging::Assert
 * @param __func
 * @param __file
 * @param __lineno
 * @param __sexp
 */
void __assert(const char *__func, const char *__file,
	     int __lineno, const char *__sexp)
{
	Log.Assert(__func, __file, __lineno, __sexp);
}

void Logging::Assert(const char func[], const char file[], int lineno, const char expr[])
{
	 // transmit diagnostic informations through serial link.
	_p_output_stream->print(F("ASSERTION FAILED :"));
	_p_output_stream->print(expr);
	_p_output_stream->print(BL);
	_p_output_stream->print(F("At "));
	_p_output_stream->print(func);
	_p_output_stream->print(F(" in "));
	_p_output_stream->print(file);
	_p_output_stream->print(F(" l."));
	_p_output_stream->print(lineno, DEC);
	_p_output_stream->flush();
	// abort program execution.
	abort();
}

void Logging::Assert(const char func[], const __FlashStringHelper * file, int lineno, const __FlashStringHelper *expr)
{
	 // transmit diagnostic informations through serial link.
	_p_output_stream->print(F("ASSERTION FAILED : "));
	_p_output_stream->print(expr);
	_p_output_stream->print(BL);
	_p_output_stream->print(F("At "));
	_p_output_stream->print(func);
	_p_output_stream->print(F(" in "));
	_p_output_stream->print(file);
	_p_output_stream->print(F(" l."));
	_p_output_stream->print(lineno, DEC);
	_p_output_stream->flush();
	// abort program execution.
	abort();
}

void Logging::Error(const char msg[], ...){
	if (LOG_LEVEL_ERRORS <= _u8_logLevel) {
		_p_output_stream->print (ERROR_STR);
		va_list args;
		va_start(args, msg);
		print(msg,args);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::Error(const __FlashStringHelper * msg, ...){
	if (LOG_LEVEL_ERRORS <= _u8_logLevel) {
		_p_output_stream->print (ERROR_STR);
		va_list args;
		va_start(args, msg);
		print(msg,args);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::Error(char errorId, const __FlashStringHelper * file, int line){
	if (LOG_LEVEL_ERRORS <= _u8_logLevel) {
		_p_output_stream->print (ERROR_STR);
		_p_output_stream->print(F("id = "));
		_p_output_stream->print((int)errorId, 10);
		_p_output_stream->print(IN_FILE);
		_p_output_stream->print(file);
		_p_output_stream->print(LINE);
		_p_output_stream->print(line);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::Error(char errorId, const __FlashStringHelper * file, int line, const __FlashStringHelper * argsFormat, ...){
	if (LOG_LEVEL_ERRORS <= _u8_logLevel) {
		_p_output_stream->print (ERROR_STR);
		_p_output_stream->print(F("id = "));
		_p_output_stream->print((int)errorId, 10);
		_p_output_stream->print(IN_FILE);
		_p_output_stream->print(file);
		_p_output_stream->print(LINE);
		_p_output_stream->print(line);
		_p_output_stream->print(BL);
		va_list args;
		va_start(args, argsFormat);
		print(argsFormat,args);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}
void Logging::Info(const char msg[], ...){
	if (LOG_LEVEL_INFOS <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
	}
}

void Logging::Info(const __FlashStringHelper * msg, ...){
	if (LOG_LEVEL_INFOS <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
	}
}

void Logging::InfoLn(const char msg[], ...){
	if (LOG_LEVEL_INFOS <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::InfoLn(const __FlashStringHelper * msg, ...){
	if (LOG_LEVEL_INFOS <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::InfoStr(const __FlashStringHelper * msg){
	if (LOG_LEVEL_INFOS <= _u8_logLevel) {
		_p_output_stream->print(msg);
		_p_output_stream->flush();
	}
}

void Logging::InfoStrLn(const __FlashStringHelper * msg){
	if (LOG_LEVEL_INFOS <= _u8_logLevel) {
		_p_output_stream->print(msg);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::Debug(const char msg[], ...){
	if (LOG_LEVEL_DEBUG <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
	}
}

void Logging::DebugLn(const __FlashStringHelper * msg, ...){
	if (LOG_LEVEL_DEBUG <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::Debug(const __FlashStringHelper * msg, ...){
	if (LOG_LEVEL_DEBUG <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
	}
}

void Logging::DebugStr(const __FlashStringHelper * msg){
	if (LOG_LEVEL_DEBUG <= _u8_logLevel) {
		_p_output_stream->print(msg);
		_p_output_stream->flush();
	}
}

void Logging::DebugStrLn(const __FlashStringHelper * msg){
	if (LOG_LEVEL_DEBUG <= _u8_logLevel) {
		_p_output_stream->print(msg);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::Verbose(const char msg[], ...){
	if (LOG_LEVEL_VERBOSE <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
	}
}

void Logging::Verbose(const __FlashStringHelper * msg, ...){
	if (LOG_LEVEL_VERBOSE <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
	}
}

void Logging::VerboseLn(const __FlashStringHelper * msg, ...){
	if (LOG_LEVEL_VERBOSE <= _u8_logLevel) {
		va_list args;
		va_start(args, msg);
		print(msg,args);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::VerboseStr(const __FlashStringHelper * msg){
	if (LOG_LEVEL_VERBOSE <= _u8_logLevel) {
		_p_output_stream->print(msg);
		_p_output_stream->flush();
	}
}

void Logging::VerboseStrLn(const __FlashStringHelper * msg){
	if (LOG_LEVEL_VERBOSE <= _u8_logLevel) {
		_p_output_stream->print(msg);
		_p_output_stream->print(BL);
		_p_output_stream->flush();
	}
}

void Logging::printArg(char arg_s8Char, va_list& args) {
	if (arg_s8Char == '%') {
		_p_output_stream->print(arg_s8Char);
	}
	if( arg_s8Char == 's' ) {
		register char *s = (char *)va_arg( args, int );
		_p_output_stream->print(s);
	}
	if( arg_s8Char == 'd' || arg_s8Char == 'i') {
		_p_output_stream->print(va_arg( args, int ),DEC);
	}
	if( arg_s8Char == 'u') {
		_p_output_stream->print((unsigned int) va_arg( args, int ),DEC);
	}
	if( arg_s8Char == 'x' ) {
		_p_output_stream->print("0x");
		_p_output_stream->print(va_arg( args, int ),HEX);
	}
	if( arg_s8Char == 'X' ) {
		_p_output_stream->print("0x");
		_p_output_stream->print(va_arg( args, int ),HEX);
	}
	if( arg_s8Char == 'b' ) {
		_p_output_stream->print(va_arg( args, int ),BIN);
	}
	if( arg_s8Char == 'B' ) {
		_p_output_stream->print("0b");
		_p_output_stream->print(va_arg( args, int ),BIN);
	}
	if( arg_s8Char == 'l' ) {
		_p_output_stream->print(va_arg( args, long ),DEC);
	}

	if( arg_s8Char == 'c' ) {
		char s = (char)va_arg( args, int );
		_p_output_stream->print(s);
	}
	if( arg_s8Char == 't' ) {
		if (va_arg( args, int ) == 1) {
			_p_output_stream->print("T");
		}
		else {
			_p_output_stream->print("F");
		}
	}
	if( arg_s8Char == 'T' ) {
		if (va_arg( args, int ) == 1) {
			_p_output_stream->print("true");
		}
		else {
			_p_output_stream->print("false");
		}
	}
	if( arg_s8Char == 'f' ) {
		_p_output_stream->print((float) va_arg( args, double ), 8);
	}
}

void Logging::print(const __FlashStringHelper * arg_ps8FlashString, va_list args) {
	const char *loc_ps8CurrByte = (const char *)arg_ps8FlashString;
	char loc_s8CurrentChar;

	// loop through format string
	while(1)
	{
		loc_s8CurrentChar = pgm_read_byte(loc_ps8CurrByte++);
		if (loc_s8CurrentChar == '\0') break;
		if (loc_s8CurrentChar == '%') {
			loc_s8CurrentChar = pgm_read_byte(loc_ps8CurrByte++);
			if (loc_s8CurrentChar == '\0') break;
			printArg(loc_s8CurrentChar, args);
		}
		else
		{
			_p_output_stream->print(loc_s8CurrentChar);
		}
	}
	_p_output_stream->flush();
}

void Logging::print(const char *format, va_list args) {
	//
	// loop through format string
	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			if (*format == '\0') break;
			printArg(*format, args);
		}
		else
		{
			_p_output_stream->print(*format);
		}
	}
	_p_output_stream->flush();
}

Logging Log = Logging();
