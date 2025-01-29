#ifndef TIM_QDEC_H
#define TIM_QDEC_H

#include <stm32f303xc.h>

typedef struct{
	uint16_t enc1;
	uint16_t enc2;
	uint16_t enc3;
	uint16_t enc4;
}encoder_cnt_t;

void tim_qdec_init();
void tim_qdec_getCount(encoder_cnt_t *cnt_ptr);

#endif