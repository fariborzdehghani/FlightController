#ifndef __GENERAL_H
#define __GENERAL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

enum ResponseType
{
    ConfigurationSet = 1,
    DroneStarted = 2,
    DroneStopped = 3
};

extern int8_t isConfigurationSet;
extern int8_t isDroneStarted;

void HandlePackage(uint8_t *data);
void ReplyPackage(enum ResponseType responseType);


#ifdef __cplusplus
}
#endif

#endif /* __GENERAL_H */
