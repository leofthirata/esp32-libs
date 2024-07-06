#ifndef __SENSORS_HPP__
#define __SENSORS_HPP__

typedef enum: uint8_t
{
    BAT_ABSENT = 0,
    BAT_DISCHARGING,
    BAT_CHARGING,
    BAT_CHARGED,
} SensorsBatStatus_t;

typedef struct
{
    uint8_t batStatus;
    float temperature;
    int batVoltage;
    int32_t acc[3];
} SensorsStatus_t;

#define SENSORS_STATUS_TIMEOUT_MS  10000
#define SENSORS_ACC_CHECK_TIMEOUT_MS   200

#define DEBOUNCE_MOVING 10000
#define DEBOUNCE_STOP 12000

void sensorsTask(void* parameter);
const char* printBatStatus(SensorsBatStatus_t type);

#endif //__SENSORS_HPP__