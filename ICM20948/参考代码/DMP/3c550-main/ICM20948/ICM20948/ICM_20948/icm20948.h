#ifndef __ICM_20948_H
#define __ICM_20948_H
#include "sys.h"

struct hal_s_
{
    long report;          // What to report?
    unsigned char  debug_reg_on;     // with '\' as a command this turns ON
    int expected_data;
    volatile unsigned char new_gyro;
};

extern struct hal_s_ hal;

typedef struct
{
	float accel_float[3];
	float gyro_float[3];
	float compass_float[3];
	float orientation[3];
	float temperature;
}icm20948_data_t;

int ICM_20948_Init(void);
void fifo_handler(void);
int handle_char_input(char c);
void gyro_data_ready_cb(void);

void self_test(void);

float ICM20948_Get_Temperature(void);
void ICM20948_Get_Data(icm20948_data_t *data);

#endif	/* __ICM_20948_H */





