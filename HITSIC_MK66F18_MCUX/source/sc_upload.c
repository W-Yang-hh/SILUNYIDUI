/******************************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,哈工大智能车创新俱乐部
 * All rights reserved.
 *
 * @file            UART
 * @company         哈工大智能车创新俱乐部
 * @author          李洋qq2367381108
 * @version         v1.0
 * @Software        MCUXpresso IDE v11.1.1
 * @Target core     K66
 * @date            2020.11.09
 *
 * @note：
        NUM_VAR为需要发送到上位机的变量个数。Send_Variable()适用于名优科创上位机，
                void Img_Upload(void)适用于红树伟业上位机

        哈尔滨工业大学智能车创新俱乐部专用，请勿泄露
***************************************************************************************************************************/

#include "sc_upload.h"
#include "drv_cam_zf9v034.hpp"

#define VAR_NUM 7//发送数组个数
extern uartMgr_t *uartMgr0;
float Variable[VAR_NUM];//发送缓存数组

void SCHOST_Send_Begin(void)
{
    uint8_t begin_cmd[3] = {0x55, 0xaa, 0x11};
    SCHOST_UART_Tx(begin_cmd, sizeof(begin_cmd));
}

void  SCHOST_Send_Variable(void)
{
  uint8_t cmdf[7] = {0x55, 0xaa, 0x11, 0x55, 0xaa,0xff, 0x01};
  uint8_t cmdr = 0x01;
  uint8_t temp[4] = {0};
  uint8_t var_num = VAR_NUM;
  SCHOST_Send_Begin();

  Variable[0] = 0;          //changing your data here
  Variable[1] = 1;
  Variable[2] = 2;
  Variable[3] = 3;
  Variable[4] = 4;
  Variable[5] = 5;
  Variable[6] = 6;

  SCHOST_UART_Tx(cmdf, sizeof(cmdf));
  SCHOST_UART_Tx(&var_num, 1);
 for(uint8_t i=0;i<VAR_NUM;i++)
  {
    memcpy(temp,Variable+i,4);
    SCHOST_UART_Tx(temp, sizeof(temp));
  }
 SCHOST_UART_Tx(&cmdr, 1);
}

void SCHOST_Img_Upload(uint8_t* upload_img, uint8_t row, uint8_t col)
{
    uint8_t cmd = 3;
    uint8_t cmdf[2] = { cmd, ~cmd };
    uint8_t cmdr[2] = { ~cmd, cmd };
    SCHOST_UART_Tx(cmdf,sizeof(cmdf));
    SCHOST_UART_Tx(upload_img, row*col);
    SCHOST_UART_Tx(cmdr, sizeof(cmdr));

}
