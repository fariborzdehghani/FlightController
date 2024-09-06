#include "General.h"
#include "Tools.h"
#include "cJSON.h"

void HandlePackage(char* dataStr)
{
    cJSON *data = cJSON_Parse(dataStr);
    cJSON *Content = cJSON_GetObjectItem(data, "Content");
    char *BaseSpeedStr = cJSON_GetObjectItem(Content, "BaseSpeed")->valuestring;

    LogInformation(1001, BaseSpeedStr);
}