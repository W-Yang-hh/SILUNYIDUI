/*
 * adc_fir.c
 *
 *  Created on: 2020年11月21日
 *      Author: 高宇昊
 */
#include "adc_fir.h"
#include "sc_adc.h"
#include "sys_extint.hpp"
#include "fsl_adc16.h"
#include "peripherals.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "hitsic_common.h"
#include "sc_host.h"
#include "sc_ftm.h"



uint32_t LV_Temp[9][10]={};
float LV[9]={};
float AD[9]={};
float MinLVGot=1;
const uint32_t channels[8] = {16, 23, 17, 18, 10, 11, 12, 13};
static float servo_adc = 0;
static float servo_adc_1[2];
static float servo_adc_2[2];
static float KP_S_AD = 0.022;
static float KD_S_AD = 0.012;
static float pwm_adc = 0;
static float LIMIT_S_High = 8.37;
static float LIMIT_S_Low = 6.78;
uint8_t SampleTimes=10;

void WIJ_multi(void)
{
    servo_adc_1[0]=(AD[4]-AD[2])/(AD[2]*AD[4]);
    servo_adc = servo_adc_1[0] * KP_S_AD + (servo_adc_1[0] - servo_adc_2[0]) * KD_S_AD;
    pwm_adc = servo_adc + 7.64;
    if (pwm_adc > LIMIT_S_High)
    {
        pwm_adc = LIMIT_S_High;
    }
    else if (pwm_adc < LIMIT_S_Low)
    {
        pwm_adc = LIMIT_S_Low;
    }
    servo_adc_2[0] = servo_adc_1[0];
    SCFTM_PWM_ChangeHiRes(FTM3, kFTM_Chnl_7 ,50U, pwm_adc);
}

void WIJ_add(void)
{
    servo_adc_1[1]=(AD[4]-AD[2])/(AD[4]+AD[2]);
    servo_adc = servo_adc_1[1] * KP_S_AD + (servo_adc_1[1] - servo_adc_2[1]) * KD_S_AD;
    pwm_adc = servo_adc + 7.64;
    if (pwm_adc > LIMIT_S_High)
    {
        pwm_adc = LIMIT_S_High;
    }
    else if (pwm_adc < LIMIT_S_Low)
    {
        pwm_adc = LIMIT_S_Low;
    }
    servo_adc_2[1] = servo_adc_1[1];
    SCFTM_PWM_ChangeHiRes(FTM3, kFTM_Chnl_7 ,50U, pwm_adc);
}

void swap(uint32_t *a,uint32_t *b)
{
  uint32_t temp=*a;
  *a=*b;
  *b=temp;
}


void LV_Sample()                             // ad采集函数
{
  for(int i=0;i<=SampleTimes-1;i++)
  {
    /*获取采样初值*/
      LV_Temp[0][i]=SCADC_Sample(ADC0,0,17);//这里只有两个电感，所以这个只有两行  (左边)
      LV_Temp[1][i]=SCADC_Sample(ADC0,0,10);//ADC_Get是指的是采集到的初始电感值（右）
  }
}

void LV_Get_Val()//约0.3mS                  //对采集的值滤波
{
 // 有时会在0-65535(255)间跳动
  for(int i=0;i<=8;i++)
  {
    for(int j=0;j<=SampleTimes-1;j++)
    {
         if(LV_Temp[i][j]>500)//剔除毛刺信号
         {
             LV_Temp[i][j]=500;
         }
    }
  }

  //排序
  for(int k=0;k<=8;k++)
  {
    for(int i=0;i<=SampleTimes-2;i++)
    {
      for(int j=i+1;j<=SampleTimes-1;j++)
      {
          if(LV_Temp[k][i]>LV_Temp[k][j])
            swap(&LV_Temp[k][i],&LV_Temp[k][j]);//交换，swap函数自己写
      }
    }
  }

  for(int k=0;k<=8;k++)
  {
    LV[k]=0;
    for(int i=3;i<=SampleTimes-4;i++)
    {
         LV[k]+=(float)LV_Temp[k][i];
    }
    LV[k]=LV[k]/(SampleTimes-6);
    if( LV[k] < MinLVGot )
    {
       LV[k] = MinLVGot;
    }
  }
  AD[0] = LV[0];
  AD[1] = LV[1];
  AD[2] = LV[2];
  AD[3] = LV[3];
  AD[4] = LV[4];
  AD[5] = LV[5];
  AD[6] = LV[6];
  AD[7] = LV[7];
  AD[8] = LV[8];//注意这里不要直接用LV数组
}








