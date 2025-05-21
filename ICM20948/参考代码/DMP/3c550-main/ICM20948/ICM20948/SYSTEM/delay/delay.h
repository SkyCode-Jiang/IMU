#ifndef __DELAY_H
#define __DELAY_H 			   

//////////////////////////////////////////////////////////////////////////////////	 
#include "sys.h"  

#define configTICK_RATE_HZ (1000u)

//////////////////////////////////////////////////////////////////////////////////	 
void delay_init(void);


void delay_ns(void);
void delay_us(u32 nus);
void delay_ms(u16 nms);

void mdelay(unsigned long nTime);
int get_tick_count(long long *count);
void TimingDelay_Decrement(void);



#endif





























