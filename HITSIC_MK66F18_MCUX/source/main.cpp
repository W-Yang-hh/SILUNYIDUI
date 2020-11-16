/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright 2018 - 2020 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hitsic_common.h"

/** HITSIC_Module_DRV */
#include "drv_ftfx_flash.hpp"
#include "drv_disp_ssd1306.hpp"
#include "drv_imu_invensense.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"

/** HITSIC_Module_SYS */
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sys_uartmgr.hpp"
#include "cm_backtrace.h"
//#include "easyflash.h"

/** HITSIC_Module_LIB */
#include "lib_graphic.hpp"

/** HITSIC_Module_APP */
#include "app_menu.hpp"
#include "app_svbmp.hpp"

/** FATFS */
#include "ff.h"
#include "sdmmc_config.h"
FATFS fatfs;                                   //逻辑驱动器的工作区

#include "sc_adc.h"
#include "sc_ftm.h"

/** HITSIC_Module_TEST */
#include "drv_cam_zf9v034_test.hpp"
#include "app_menu_test.hpp"
#include "drv_imu_invensense_test.hpp"
#include "sys_fatfs_test.hpp"
#include "sys_fatfs_diskioTest.hpp"

/** SCLIB_TEST */
#include "image.h"
#include "sc_host.h"


static float KP_M = 0.0;
static float KI_M = 0.0;
static float KP_S = 0.021;
static float KD_S = 0.012;
static float LIMIT_S_High = 8.37;
static float LIMIT_S_Low = 6.78;
static float servo_pid;
static float pwm_servo;
static float pwm_motor_l = 30;
static float pwm_motor_r = 30;
static int S_run = 0;
static int mode_change = 0;
//cam_zf9v034_configPacket_t cameraCfg;
//dmadvp_config_t dmadvpCfg;
//dmadvp_handle_t dmadvpHandle;

void motor(void);
void servo(void);
void modechange(void);
void startrun(void);
void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);

inv::i2cInterface_t imu_i2c(nullptr, IMU_INV_I2cRxBlocking, IMU_INV_I2cTxBlocking);
inv::mpu6050_t imu_6050(imu_i2c);

disp_ssd1306_frameBuffer_t dispBuffer;
graphic::bufPrint0608_t<disp_ssd1306_frameBuffer_t> bufPrinter(dispBuffer);

void main(void)
{
    /** 初始化阶段，关闭总中断 */
    HAL_EnterCritical();

    /** BSP（板级支持包）初始化 */
    RTECLK_HsRun_180MHz();
    RTEPIN_Basic();
    RTEPIN_Digital();
    RTEPIN_Analog();
    RTEPIN_LPUART0_DBG();
    RTEPIN_UART0_WLAN();
    RTEPIP_Basic();
    RTEPIP_Device();

    /** 初始化调试组件 */
    DbgConsole_Init(0U, 921600U, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Welcome to HITSIC !\n");
    PRINTF("GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    cm_backtrace_init("HITSIC_MK66F18", "2020-v3.0", "v4.1.1");
    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    DISP_SSD1306_delay_ms(1000);
    /** 初始 bveaa化ftfx_Flash */
    FLASH_SimpleInit();
    /** 初始化PIT中断管理器 */
    pitMgr_t::init();
    /** 初始化I/O中断管理器 */
    extInt_t::init();
    /** 初始化菜单 */
    MENU_Init();
    MENU_Data_NvmReadRegionConfig();
    MENU_Data_NvmRead(menu_currRegionNum);
    /** 菜单挂起 */
    MENU_Suspend();
    /** 初始化摄像头 */


    cam_zf9v034_configPacket_t cameraCfg;
    dmadvp_config_t dmadvpCfg;
    dmadvp_handle_t dmadvpHandle;
    CAM_ZF9V034_GetDefaultConfig(&cameraCfg);    //设置摄像头配置
    CAM_ZF9V034_CfgWrite(&cameraCfg);             //写入配置
    CAM_ZF9V034_GetReceiverConfig(&dmadvpCfg, &cameraCfg);   //生成对应接收器的配置数据，使用此数据初始化接受器并接收图像数据。
    DMADVP_Init(DMADVP0, &dmadvpCfg);
    DMADVP_TransferCreateHandle(&dmadvpHandle, DMADVP0, CAM_ZF9V034_UnitTestDmaCallback);
    uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
    uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];
    //uint8_t *fullBuffer = NULL;
    disp_ssd1306_frameBuffer_t *dispBuffer = new disp_ssd1306_frameBuffer_t;
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer0);
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer1);
    DMADVP_TransferStart(DMADVP0, &dmadvpHandle);


   /** 初始化IMU */
   //TODO: 在这里初始化IMU（MPU6050）
   /** 菜单就绪 */
   //MENU_Resume();
   //MENU_Suspend();
   /** 控制环初始化 */
    //TODO: 在这里初始化控制环

    /** 内置DSP函数测试 */
    float f = arm_sin_f32(0.6f);     //什么意思？


    PORT_SetPinInterruptConfig(PORTA, 11U, kPORT_InterruptFallingEdge);
    extInt_t::insert(PORTA, 11U, startrun);
    pitMgr_t::insert(6U, 3, motor, pitMgr_t::enable);//电机的定时中断
    pitMgr_t::insert(20U, 5, servo, pitMgr_t::enable);//舵机的定时中断


    /** 初始化结束，开启总中断 */         //开启总中断应该是主函数进入死循环前的最后一条语句。
       HAL_ExitCritical();              //开总中断，打开这个才能做其他的事儿

    while (true)
    {
        while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));
        //SCHOST_ImgUpload(120, 188, fullBuffer);//单缓冲上传图片函数
        THRE();
        head_clear();
        image_main();
                dispBuffer->Clear();
                const uint8_t imageTH = 200;
                for (int i = 0; i < cameraCfg.imageRow; i += 2)
                {
                    int16_t imageRow = i >> 1;//除以2 为了加速;
                    int16_t dispRow = (imageRow / 8) + 1, dispShift = (imageRow % 8);
                    for (int j = 0; j < cameraCfg.imageCol; j += 2)
                    {
                        int16_t dispCol = j >> 1;
                        if (IMG[i][j] > imageTH)
                        {
                            dispBuffer->SetPixelColor(dispCol, imageRow, 1);
                        }
                    }
                }
                modechange();//通过拨码改变模式（调参和图像）
                if(GPIO_PinRead(GPIOA, 9U) == 0)
                {
                    DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
                }
//                if(S_run == 1)
//                {
//                    DMADVP_TransferStart(DMADVP0, &dmadvpHandle);
//                    S_run = 0;
//                }
                DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, fullBuffer);
                DMADVP_TransferStart(DMADVP0, &dmadvpHandle);

        //TODO: 在这里添加车模保护代码
     }

}


/**
 * @brief 菜单构建
 * 该函数会在初始化时被MENU_Init(void);函数调用，禁止手动调用
 */


void ExampleHandler(menu_keyOp_t* const _op)
{
    *_op = 0;
}
//static float KP_M = 0.0;
//static float KI_M = 0.0;
//static float KP_S = 0.015;
//static float KD_S = 0.01;
//static float LIMIT_S_High = 8.65;
//static float LIMIT_S_Low = 7.05;
//static float servo_pid;
//static float pwm_servo;
//static float pwm_motor_l = 30;
//static float pwm_motor_r = 30;
void MENU_DataSetUp(void)
{
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "TEST", 0, 0));
    /** 创建子菜单指针 */
    menu_list_t* myList_1;
/** 子菜单指针初始化 */
    myList_1 = MENU_ListConstruct(
        "myList_1",     ///> 菜单标题，在菜单列表中的第一行显示，最大12字符。
        50,             ///> 菜单列表的大小，须预留1位用于返回上一级的[back]。
        menu_menuRoot   ///> 该菜单的上级菜单指针。注意：该指针仅用于返回上级菜单，并不会将子菜单插入上级菜单。
    );
/** 检查内存分配是否成功 */
    assert(myList_1);
/** 将子菜单的跳转入口插入其上级菜单 */
    MENU_ListInsert(
        menu_menuRoot,  ///> 要插入的上级菜单。
        MENU_ItemConstruct(
        menuType,   ///> 类型标识，指明这是一个菜单跳转类型的菜单项。
        myList_1,   ///> 数据指针，这里指向要跳转到的菜单列表。
        "TestList", ///> 菜单项名称，在菜单列表中显示。
        0,          ///> 数据的保存位置，对于非数据类型填0即可。
        0           ///> 属性Flag，无任何属性填0。
    ));
    {   //这里加这组括号只是为了缩进方便，其内部的语句用于向myList_1插入菜单项。
        MENU_ListInsert(myList_1, MENU_ItemConstruct(
            variType,  ///> 类型标识，指明这是一个整数类型的菜单项
            &prospect,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
            "Prospect",   ///> 菜单项名称，在菜单列表中显示。
            10,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                       ///> 全局数据区0~9的地址为保留地址，不能使用。
            menuItem_data_global
                       ///> 属性flag。此flag表示该变量存储于全局数据区，且为只读变量。
        ));
        MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    variType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &threshold,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "threshold",   ///> 菜单项名称，在菜单列表中显示。
                    11,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                               ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                               ///> 属性flag。此flag表示该变量存储于全局数据区，且为只读变量。
                ));
        MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &KP_S,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "KP_S",   ///> 菜单项名称，在菜单列表中显示。
                    12,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                               ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                               ///> 属性flag。此flag表示该变量存储于全局数据区，且为只读变量。
                ));
        MENU_ListInsert(myList_1, MENU_ItemConstruct(
                            varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                            &KD_S,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                            "KD_S",   ///> 菜单项名称，在菜单列表中显示。
                            13,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                                       ///> 全局数据区0~9的地址为保留地址，不能使用。
                            menuItem_data_global
                                       ///> 属性flag。此flag表示该变量存储于全局数据区，且为只读变量。
                        ));
        MENU_ListInsert(myList_1, MENU_ItemConstruct(
                            varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                            &pwm_motor_r,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                            "pwm_motor_r",   ///> 菜单项名称，在菜单列表中显示。
                            14,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                                       ///> 全局数据区0~9的地址为保留地址，不能使用。
                            menuItem_data_global
                                       ///> 属性flag。此flag表示该变量存储于全局数据区，且为只读变量。
                        ));
        MENU_ListInsert(myList_1, MENU_ItemConstruct(
                            varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                            &pwm_motor_l,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                            "pwm_motor_l",   ///> 菜单项名称，在菜单列表中显示。
                            15,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                                       ///> 全局数据区0~9的地址为保留地址，不能使用。
                            menuItem_data_global
                                       ///> 属性flag。此flag表示该变量存储于全局数据区，且为只读变量。
                        ));

    }
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(
          procType,  ///> 类型标识，指明这是一个浮点类型的菜单项
          &ExampleHandler,///> 数据指针，这里指向要操作的整数。必须是float类型。
          "T_proc ", ///> 菜单项名称，在菜单列表中显示。
          0,         ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
          menuItem_proc_runOnce
                     ///> 属性flag。此flag表示该该程序运行一次就退出。
    ));
}

void servo(void)
{
        error_now_s = 90-(int)mid_line[prospect];
        servo_pid = KP_S *error_now_s  + KD_S*(error_now_s - error_last_s );
        pwm_servo = servo_pid + 7.64;
        if(pwm_servo>LIMIT_S_High)
        {
            pwm_servo = LIMIT_S_High;
        }
        else if(pwm_servo<LIMIT_S_Low)
        {
            pwm_servo = LIMIT_S_Low;
        }
        error_last_s = error_now_s;
        SCFTM_PWM_ChangeHiRes(FTM3, kFTM_Chnl_7 ,50U, pwm_servo);
}


void motor(void)
{
    if(S_run == 1)
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_0 ,20000U, pwm_motor_l);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_1 ,20000U, 0);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_2, 20000U, pwm_motor_r);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_3, 20000U, 0);
    }
}

void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    //TODO: 补完本回调函数，双缓存采图。

    //TODO: 添加图像处理（转向控制也可以写在这里）
}

void modechange(void)
{
    if (GPIO_PinRead(GPIOA, 9U) == 1 && mode_change == 0)//拨码下降沿时开启菜单
    {
        MENU_Resume();
        mode_change = 1;
    }
    else if (GPIO_PinRead(GPIOA, 9U) == 0 && mode_change == 1)//拨码上升沿时挂起菜单
    {
        MENU_Suspend();
        mode_change = 0;
    }
    //SDK_DelayAtLeastUs(1000 * 1000, CLOCK_GetFreq(kCLOCK_CoreSysClk));//延时1秒，防止一次拨动进入该函数多次
}

void startrun(void)
{
    SDK_DelayAtLeastUs(4000 * 1000, CLOCK_GetFreq(kCLOCK_CoreSysClk));//延时2秒发车
    S_run = 1;
}
