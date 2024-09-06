#ifndef __TOOLS_H
#define __TOOLS_H

#include "main.h"

void LogNumberToPC(int step);
void LogStringToPC(char *Data);
void LogFLStringToPC(char *Data, int Length);
void LogBinaryToPC(int Data);
void LogError(int Code, char *Message);
void LogInformation(int Code, char *Message);
char *int_to_binary_string(int num);
char* escape_quotes(const char* input);

#ifdef __cplusplus
}
#endif

#endif /* __TOOLS_H */
