/*
 * adc_fir.h
 *
 *  Created on: 2020年11月21日
 *      Author: 高宇昊
 */

#ifndef ADC_FIR_H_
#define ADC_FIR_H_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>



void WIJ_multi(void);
void WIJ_add(void);
void LV_Sample();
void LV_Get_Val();
void swap(uint32_t *a,uint32_t *b);

#endif /* ADC_FIR_H_ */
