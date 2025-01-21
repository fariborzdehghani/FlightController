#ifndef __TOOLS_H
#define __TOOLS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void LogNumberToPC(int step);
void LogStringToPC(char *Data);
void LogFLStringToPC(char *Data, int Length);
void LogBinaryToPC(int Data);
void LogError(int Code, char *Message);
void LogInformation(int Code, char *Message);
char *int_to_binary_string(int num);
char* escape_quotes(const char* input);
char* doubleToString(double value, int precision);
void Disable_MPU6050();
void Enable_MPU6050();
#ifdef __cplusplus
}
#endif

#endif /* __TOOLS_H */
