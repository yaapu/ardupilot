#pragma once

#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if  HAL_MSP_ENABLED

// use betaflight 4.1.0 for compatibility with DJI OSD
#define GIT_SHORT_REVISION_LENGTH   8
#define BUILD_DATE_LENGTH           11
#define BUILD_TIME_LENGTH           8

#define FC_VERSION_MAJOR            4
#define FC_VERSION_MINOR            1
#define FC_VERSION_PATCH_LEVEL      0

#endif  //HAL_MSP_ENABLED