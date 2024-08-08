#ifndef __UTILS_H__
#define __UTILS_H__


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

uint8_t dallas_crc8(const uint8_t *pdata, const uint32_t size);
uint8_t crc8_itu(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__UTILS_H__
