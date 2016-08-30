#ifndef LOGGER_H
#define LOGGER_H

#include "Arduino.h"

#define LOGGER_SEVERITY_DEBUG 0
#define LOGGER_SEVERITY_INFO  1
#define LOGGER_SEVERITY_WARN  2
#define LOGGER_SEVERITY_ERROR 3
#define LOGGER_SEVERITY_FATAL 4
#define LOGGER_SEVERITY_NONE  5

#ifndef LOGGER_MIN_SEVERITY
#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_DEBUG
#endif

// Debug
#if (LOGGER_MIN_SEVERITY > LOGGER_SEVERITY_DEBUG)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_NAMED(...)
#define DEBUG_PRINTLN_NAMED(...)
#define DEBUG_PRINT_RAW(...)
#define DEBUG_PRINTLN_RAW(...)
#else
#define DEBUG_PRINT_RAW(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN_RAW(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINT(...) {Serial.print("D/"); Serial.print(__VA_ARGS__);}
#define DEBUG_PRINTLN(...) {Serial.print("D/"); Serial.println(__VA_ARGS__);}
#define DEBUG_PRINT_NAMED(name, ...) {Serial.print("D/"); Serial.print(name"/");Serial.print(__VA_ARGS__);}
#define DEBUG_PRINTLN_NAMED(name, ...) {Serial.print("D/"); Serial.print(name"/");Serial.println(__VA_ARGS__);}
#endif

// Info
#if (LOGGER_MIN_SEVERITY > LOGGER_SEVERITY_INFO)
#define INFO_PRINT(...)
#define INFO_PRINTLN(...)
#define INFO_PRINT_NAMED(...)
#define INFO_PRINTLN_NAMED(...)
#define INFO_PRINT_RAW(...)
#define INFO_PRINTLN_RAW(...)
#else
#define INFO_PRINT_RAW(...) Serial.print(__VA_ARGS__)
#define INFO_PRINTLN_RAW(...) Serial.println(__VA_ARGS__)
#define INFO_PRINT(...) {Serial.print("I/"); Serial.print(__VA_ARGS__);}
#define INFO_PRINTLN(...) {Serial.print("I/"); Serial.println(__VA_ARGS__);}
#define INFO_PRINT_NAMED(name, ...) {Serial.print("I/"); Serial.print(name"/");Serial.print(__VA_ARGS__);}
#define INFO_PRINTLN_NAMED(name, ...) {Serial.print("I/"); Serial.print(name"/");Serial.println(__VA_ARGS__);}
#endif

// Warn
#if (LOGGER_MIN_SEVERITY > LOGGER_SEVERITY_WARN)
#define WARN_PRINT(...)
#define WARN_PRINTLN(...)
#define WARN_PRINT_NAMED(...)
#define WARN_PRINTLN_NAMED(...)
#define WARN_PRINT_RAW(...)
#define WARN_PRINTLN_RAW(...)
#else
#define WARN_PRINT_RAW(...) Serial.print(__VA_ARGS__)
#define WARN_PRINTLN_RAW(...) Serial.println(__VA_ARGS__)
#define WARN_PRINT(...) {Serial.print("W/"); Serial.print(__VA_ARGS__);}
#define WARN_PRINTLN(...) {Serial.print("W/"); Serial.println(__VA_ARGS__);}
#define WARN_PRINT_NAMED(name, ...) {Serial.print("W/"); Serial.print(name"/");Serial.print(__VA_ARGS__);}
#define WARN_PRINTLN_NAMED(name, ...) {Serial.print("W/"); Serial.print(name"/");Serial.println(__VA_ARGS__);}
#endif

// Error
#if (LOGGER_MIN_SEVERITY > LOGGER_SEVERITY_ERROR)
#define ERROR_PRINT(...)
#define ERROR_PRINTLN(...)
#define ERROR_PRINT_NAMED(...)
#define ERROR_PRINTLN_NAMED(...)
#define ERROR_PRINT_RAW(...)
#define ERROR_PRINTLN_RAW(...)
#else
#define ERROR_PRINT_RAW(...) Serial.print(__VA_ARGS__)
#define ERROR_PRINTLN_RAW(...) Serial.println(__VA_ARGS__)
#define ERROR_PRINT(...) {Serial.print("E/"); Serial.print(__VA_ARGS__);}
#define ERROR_PRINTLN(...) {Serial.print("E/"); Serial.println(__VA_ARGS__);}
#define ERROR_PRINT_NAMED(name, ...) {Serial.print("E/"); Serial.print(name"/");Serial.print(__VA_ARGS__);}
#define ERROR_PRINTLN_NAMED(name, ...) {Serial.print("E/"); Serial.print(name"/");Serial.println(__VA_ARGS__);}
#endif

// Fatal
#if (LOGGER_MIN_SEVERITY > LOGGER_SEVERITY_FATAL)
#define FATAL_PRINT(...)
#define FATAL_PRINTLN(...)
#define FATAL_PRINT_NAMED(...)
#define FATAL_PRINTLN_NAMED(...)
#define FATAL_PRINT_RAW(...)
#define FATAL_PRINTLN_RAW(...)
#else
#define FATAL_PRINT_RAW(...) Serial.print(__VA_ARGS__)
#define FATAL_PRINTLN_RAW(...) Serial.println(__VA_ARGS__)
#define FATAL_PRINT(...) {Serial.print("F/"); Serial.print(__VA_ARGS__);}
#define FATAL_PRINTLN(...) {Serial.print("F/"); Serial.println(__VA_ARGS__);}
#define FATAL_PRINT_NAMED(name, ...) {Serial.print("F/"); Serial.print(name"/");Serial.print(__VA_ARGS__);}
#define FATAL_PRINTLN_NAMED(name, ...) {Serial.print("F/"); Serial.print(name"/");Serial.println(__VA_ARGS__);}
#endif

#endif

