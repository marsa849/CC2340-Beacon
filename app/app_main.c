/******************************************************************************

@file  app_main.c

@brief This file contains the application main functionality

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

#include <ti/drivers/ADC.h>
#include <icall_ble_api.h>
#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

// Parameters that should be given as input to the BLEAppUtil_init function
BLEAppUtil_GeneralParams_t appMainParams =
{
    .taskPriority = 1,
    .taskStackSize = 1024,
    .profileRole = (BLEAppUtil_Profile_Roles_e)(HOST_CONFIG),
    .addressMode = DEFAULT_ADDRESS_MODE,
    .deviceNameAtt = attDeviceName,
};

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
BLEAppUtil_PeriCentParams_t appMainPeriCentParams =
{
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
 .connParamUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION,
#endif //#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
 .gapBondParams = &gapBondParams
};
#else //observer || broadcaster
BLEAppUtil_PeriCentParams_t appMainPeriCentParams;
#endif //#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )

#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_pmctl.h)
#define SystemReset()        __disable_irq();  HWREG(PMCTL_BASE + PMCTL_O_RSTCTL) |= PMCTL_RSTCTL_SYSRST_SET;  while (1) {}


typedef enum
{
    ADV_INT_875MS = 1,
    ADV_INT_500MS = 2,
    ADV_INT_300MS = 3,
    ADV_INT_250MS = 4,
    ADV_INT_200MS = 5,
    ADV_INT_100MS = 10,
    ADV_INT_50MS = 20,
    ADV_INT_30MS = 30,
    ADV_INT_20MS = 50,
    ADV_INT_1000MS = 21,
    ADV_INT_2000MS = 22
} AdvInt;

typedef enum
{
    TX_POWER_NEGATIVE_20 = 0,
    TX_POWER_NEGATIVE_16 = 1,
    TX_POWER_NEGATIVE_12 = 2,
    TX_POWER_NEGATIVE_8 = 3,
    TX_POWER_NEGATIVE_4 = 4,
    TX_POWER_NEGATIVE_2 = 5,
    TX_POWER_0 = 6,
    TX_POWER_4 = 7,
    TX_POWER_8 = 8
} AdvPwr;

#define SNV_ID_APP 0x100

#define FLASH_LEN 46 // len store beacon param

#define FLASH_UUID                          0xfd, 0xa5, 0x06, 0x93, 0xa4, 0xe2, 0x4f, 0xb1, \
                                            0xaf, 0xcf, 0xc6, 0xeb, 0x07, 0x64, 0x78, 0x25,
#define FLASH_TXPOWER                       0x6,
#define FLASH_MAJOR                         0x27, 0x11,
#define FLASH_MINOR                         0x00, 0x01,
#define FLASH_ADVINT                        ADV_INT_300MS,
#define FLASH_RXP                           0xc5,
#define FLASH_PASSWORD                      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
#define FLASH_NAME                          'B','e','e','L','i','n','k','e','r','0','0','0','0','0',
#define FLASH_NAMELEN                       9

uint8 storage[FLASH_LEN] = {FLASH_UUID FLASH_TXPOWER FLASH_MAJOR FLASH_MINOR FLASH_ADVINT FLASH_RXP FLASH_PASSWORD FLASH_NAME FLASH_NAMELEN};

uint8 madvData[30]={0x02,0x01,0x24,0x1a,0xff,
                    0x4c,0x00,0x02,0x15,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,
                    0x00,0x00,
                    0x00};
uint8 mrspData[31]={0x03,0x03,0xf0,0xff};
uint16 adv_interval;
uint16 battery_vol;
int8 tx_power;

TimerHandle_t resetTimer;
//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      criticalErrorHandler
 *
 * @brief   Application task entry point
 *
 * @return  none
 */
void criticalErrorHandler(int32 errorCode , void* pInfo)
{
//    trace();
//
//#ifdef DEBUG_ERR_HANDLE
//
//    while (1);
//#else
//    SystemReset();
//#endif

}

static int8_t get_tx_pwr(uint8_t index)
{
    int8_t m_tx_pwr;
    switch (index)
    {
    case TX_POWER_NEGATIVE_20:
        m_tx_pwr = -20;
        break;
    case TX_POWER_NEGATIVE_16:
        m_tx_pwr = -16;
        break;
    case TX_POWER_NEGATIVE_12:
        m_tx_pwr = -12;
        break;
    case TX_POWER_NEGATIVE_8:
        m_tx_pwr = -8;
        break;
    case TX_POWER_NEGATIVE_4:
        m_tx_pwr = -4;
        break;
    case TX_POWER_NEGATIVE_2:
        m_tx_pwr = 0;
        break;
    case TX_POWER_0:
        m_tx_pwr = 0;
        break;
    case TX_POWER_4:
        m_tx_pwr = 4;
        break;
    case TX_POWER_8:
        m_tx_pwr = 8;
        break;

        default:
        break;
    }
    return m_tx_pwr;
}

static uint16_t get_adv_int(uint8_t index)
{
    uint16_t m_adv_int=0;
    switch (index)
    {
    case ADV_INT_875MS:
        m_adv_int = 875;
        break;
    case ADV_INT_500MS:
        m_adv_int = 500;
        break;
    case ADV_INT_300MS:
        m_adv_int = 300;
        break;
    case ADV_INT_250MS:
        m_adv_int = 250;
        break;
    case ADV_INT_200MS:
        m_adv_int = 200;
        break;
    case ADV_INT_100MS:
        m_adv_int = 100;
        break;
    case ADV_INT_50MS:
        m_adv_int = 50;
        break;
    case ADV_INT_30MS:
        m_adv_int = 30;
        break;
    case ADV_INT_20MS:
        m_adv_int = 20;
        break;
    case ADV_INT_1000MS:
        m_adv_int = 1000;
        break;
    case ADV_INT_2000MS:
        m_adv_int = 2000;
        break;
        default:
        break;
    }
    return (m_adv_int*1000/625);
}

void set_param(void)
{
    memcpy(madvData+9,storage,16);              //adv uuid
    tx_power = get_tx_pwr(storage[16]);         //tx power
    memcpy(madvData+25,storage+17,2);           //adv major
    memcpy(madvData+27,storage+19,2);           //adv minor
    adv_interval = get_adv_int(storage[21]);    //adv interval
    madvData[29] = storage[22];                 //adv rxp

    mrspData[4] = storage[45]+1;                  //rsp name len
    mrspData[5] = 0x09;
    memcpy(mrspData+6,storage+31,storage[45]);  //rsp name
    mrspData[6+storage[45]] = 0x0a;
    mrspData[7+storage[45]] = 0x16;
    memcpy(mrspData+8+storage[45],storage+14,2);//rsp uuid
    memcpy(mrspData+10+storage[45],storage+17,2);//rsp major
    memcpy(mrspData+12+storage[45],storage+19,2);//rsp minor

    mrspData[14+storage[45]] = battery_vol%256;            //battery
    mrspData[15+storage[45]] = battery_vol/256;            //battery
    mrspData[16+storage[45]] = 0x16;            //temporary setting, device id
}

uint16 adc_get(void)                            //unit: Centivolt
{
    uint16 result;
    uint32 result_sum=0;
    uint32 vol=0;

    ADC_init();
    ADC_Params params;
    ADC_Params_init(&params);
    params.isProtected = true;
    ADC_Handle adcHandle = ADC_open(0, &params);
    for(int i=0;i<6;i++)
    {
        ADC_convert(adcHandle, &result);
        result_sum += result;
    }
    vol = ADC_convertToMicroVolts(adcHandle, result_sum/6);
    vol=vol*3/10000;
    ADC_close(adcHandle);
    return (uint16)vol;
}

void resetTimerCallback( TimerHandle_t xTimer )
{
    SystemReset();
}

/*********************************************************************
 * @fn      App_StackInitDone
 *
 * @brief   This function will be called when the BLE stack init is
 *          done.
 *          It should call the applications modules start functions.
 *
 * @return  none
 */
void App_StackInitDoneHandler(gapDeviceInitDoneEvent_t *deviceInitDoneData)
{
    bStatus_t status = SUCCESS;

    status = osal_snv_read(SNV_ID_APP, FLASH_LEN, (uint8 *)storage);
    if(status != SUCCESS)
    {
        osal_snv_write(SNV_ID_APP, FLASH_LEN, (uint8 *)storage);
    }

    battery_vol = adc_get();
    set_param();

    resetTimer = xTimerCreate("Timer 1",pdMS_TO_TICKS( 1000*60*30 ),pdTRUE,( void * ) 1,&resetTimerCallback );
    xTimerStart(resetTimer,0);

    // Menu
    Menu_start();

    // Print the device ID address
    MenuModule_printf(APP_MENU_DEVICE_ADDRESS, 0, "BLE ID Address: "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_GREEN "%s" MENU_MODULE_COLOR_RESET,
                      BLEAppUtil_convertBdAddr2Str(deviceInitDoneData->devAddr));

    if ( appMainParams.addressMode > ADDRMODE_RANDOM)
    {
      // Print the RP address
        MenuModule_printf(APP_MENU_DEVICE_RP_ADDRESS, 0,
                     "BLE RP Address: "
                     MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_GREEN "%s" MENU_MODULE_COLOR_RESET,
                     BLEAppUtil_convertBdAddr2Str(GAP_GetDevAddress(FALSE)));
    }

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
    // Any device that accepts the establishment of a link using
    // any of the connection establishment procedures referred to
    // as being in the Peripheral role.
    // A device operating in the Peripheral role will be in the
    // Peripheral role in the Link Layer Connection state.
    status = Peripheral_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( BROADCASTER_CFG ) )
    // A device operating in the Broadcaster role is a device that
    // sends advertising events or periodic advertising events
    status = Broadcaster_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    // A device that supports the Central role initiates the establishment
    // of an active physical link. A device operating in the Central role will
    // be in the Central role in the Link Layer Connection state.
    // A device operating in the Central role is referred to as a Central.
    status = Central_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( OBSERVER_CFG ) )
    // A device operating in the Observer role is a device that
    // receives advertising events or periodic advertising events
    status = Observer_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
    status = Connection_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = Pairing_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = Data_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = DevInfo_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = SimpleGatt_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ))  &&  defined(OAD_CFG)
    status =  OAD_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif
}

/*********************************************************************
 * @fn      appMain
 *
 * @brief   Application main function
 *
 * @return  none
 */
void appMain(void)
{
    // Call the BLEAppUtil module init function
    BLEAppUtil_init(&criticalErrorHandler, &App_StackInitDoneHandler,
                    &appMainParams, &appMainPeriCentParams);
}
