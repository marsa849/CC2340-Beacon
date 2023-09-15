/******************************************************************************

@file  BLEAppUtil_task.c

@brief This file contains the BLEAppUtil module task function and related
       functionality

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_internal.h>

/*********************************************************************
 * MACROS
 */


/*********************************************************************
* CONSTANTS
*/
// Task configuration
#define appTaskStack            NULL
#define BLEAPPUTIL_QUEUE_EVT    0x40000000
#define EVENT_PEND_FOREVER      0xFFFFFFFF

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* LOCAL FUNCTIONS
*/
void *BLEAppUtil_Task(void *arg);

/*********************************************************************
 * EXTERN FUNCTIONS
*/

/*********************************************************************
* CALLBACKS
*/

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
* LOCAL FUNCTIONS
*/

/*********************************************************************
 * @fn      BLEAppUtil_createBLEAppUtilTask
 *
 * @brief   Create the BLEAppUtil task.
 *
 * @return  SUCCESS, otherwise retVal error.
 */
int BLEAppUtil_createBLEAppUtilTask(void)
{
    int retVal = 0;
    pthread_attr_t param_attribute;
    struct sched_param param;

    retVal =  pthread_attr_init(&param_attribute);
    param.sched_priority = BLEAppUtilLocal_GeneralParams->taskPriority;

    retVal |= pthread_attr_setschedparam(&param_attribute, &param);
    retVal |= pthread_attr_setstack(&param_attribute, appTaskStack, BLEAppUtilLocal_GeneralParams->taskStackSize);
    retVal |= pthread_attr_setdetachstate(&param_attribute, PTHREAD_CREATE_DETACHED);

    retVal |= pthread_create(&BLEAppUtil_theardEntity.threadId,
                             &param_attribute,
                             &BLEAppUtil_Task,
                             NULL);
    return retVal;
}

/*********************************************************************
 * @fn      BLEAppUtil_enqueueMsg
 *
 * @brief   Enqueue the message from the BLE stack to the application queue.
 *
 * @param   event - message event.
 * @param   pData - pointer to the message from BLE stack.
 *
 * @return  SUCCESS   - message was enqueued successfully
 * @return  otherwise - error value is returned
 */
status_t BLEAppUtil_enqueueMsg(uint8_t event, void *pData)
{
    int8_t status = SUCCESS;
    BLEAppUtil_appEvt_t msg;

    // Check if the queue is valid
    if (BLEAppUtil_theardEntity.queueHandle == (mqd_t)-1)
    {
        return(bleNotReady);
    }

    msg.event = event;
    msg.pData = pData;

    // Send the msg to the application queue
    status = mq_send(BLEAppUtil_theardEntity.queueHandle,(char*)&msg,sizeof(msg),1);

    return status;
}

#include <app_main.h>
#include <string.h>
#include <ti/drivers/UART2.h>

#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_pmctl.h)
#define SystemReset()        __disable_irq();  HWREG(PMCTL_BASE + PMCTL_O_RSTCTL) |= PMCTL_RSTCTL_SYSRST_SET;  while (1) {}

extern uint8 uartReadBuffer[];
extern uint8 storage[];
extern UART2_Handle uart;

const uint8 D_FRT[10] ={'2','0','2','3','-','0','8','-','0','9'};
const uint8 D_FR[14]={'B','L','M','5','2','1','0','_','N','_','0','8','0','9'};

const  uint8 D_CKey[16]={0xDE,0x48,0x2B,0x1C,0x22,0x1C,0x6C,0x30,0x3C,0xF0,0x50,0xEB,0x00,0x20,0xB0,0xBD};

void set_name_len(flash_info * add)
{
    uint8 countzero=0;
    for(uint8 i=0;i<14;i++)
    {
        if(add->beacon.adv_name[i]==0)
            countzero++;
    }
    add->beacon.adv_name_len=14-countzero;
}
void process_uart(uint8 *data)
{
    uint8 rsp_buf[43];
    flash_info *mflash = (flash_info *)storage;
    if(memcmp(uartReadBuffer,"AT+2=",5) == 0)
    {
        memcpy(mflash->beacon.uuid,uartReadBuffer+5,16);
        memcpy(mflash->beacon.major,uartReadBuffer+21,2);
        memcpy(mflash->beacon.minor,uartReadBuffer+23,2);
        memcpy(mflash->mdate,uartReadBuffer+25,10);
        memcpy(mflash->hwvr,uartReadBuffer+35,4);
        memcpy(&mflash->beacon.txpower,&uartReadBuffer[39],1);
        memcpy(&mflash->beacon.advint,&uartReadBuffer[40],1);
        memcpy(mflash->beacon.password,uartReadBuffer+47,8);
        memcpy(mflash->beacon.adv_name,uartReadBuffer+55,14);
        set_name_len(mflash);
        memcpy(&mflash->beacon.rxp,&uartReadBuffer[69],1);

        osal_snv_write(0x100, 61, (uint8 *)storage);
        UART2_write(uart,"OK+1\r\n",6,0);
    }
    else if(memcmp(uartReadBuffer,"AT+?",4) == 0)
    {
        mflash->isConfigured=1;
        osal_snv_write(0x100, 61, (uint8 *)storage);
        UART2_write(uart,"OK+",3,0);
        memcpy(rsp_buf,GAP_GetDevAddress( TRUE ),6);
        memcpy(rsp_buf+6,&mflash->beacon.txpower,1);
        memcpy(rsp_buf+7,&mflash->beacon.advint,1);
        memcpy(rsp_buf+8,&mflash->beacon.major,2);
        memcpy(rsp_buf+10,&mflash->beacon.minor,2);
        memcpy(rsp_buf+12,&mflash->beacon.uuid,16);
        memcpy(rsp_buf+28,&mflash->mdate,10);
        memcpy(rsp_buf+38,&mflash->beacon.rxp,1);
        memcpy(rsp_buf+39,&mflash->hwvr,4);
        UART2_write(uart,rsp_buf,43,0);
        UART2_write(uart,D_FRT,sizeof(D_FRT),0);
        UART2_write(uart,D_FR+10,4,0);
        UART2_write(uart,D_CKey,sizeof(D_CKey),0);

        SystemReset();
    }
}
/*********************************************************************
 * @fn      BLEAppUtil_Task
 *
 * @brief   The BLEAppUtil task function.
 *          This function registers to get stack events, call the stack
 *          initializing function and process the events that are
 *          enqueued to it's queue.
 *          Note: The data is freed in this function, the application
 *                needs to copy the data in order to save it in order
 *                to use it outside the event handler of the application.
 *
 * @return  None
 */

void *BLEAppUtil_Task(void *arg)
{
    // Register to the stack and create queue and event
    BLEAppUtil_stackRegister();

    // Init the ble stack
    BLEAppUtil_stackInit();

    // Application main loop
    for (;;)
    {
        BLEAppUtil_appEvt_t pAppEvt;

        // wait until receive queue message
        if (mq_receive(BLEAppUtil_theardEntity.queueHandle, (char*)&pAppEvt, sizeof(pAppEvt), NULL) > 0)
        {
            BLEAppUtil_msgHdr_t *pMsgData = (BLEAppUtil_msgHdr_t *)pAppEvt.pData;
            bool freeMsg = FALSE;

            switch (pAppEvt.event)
            {
               case 0x55:
                    process_uart(pAppEvt.pData);
                    break;
              case BLEAPPUTIL_EVT_STACK_CALLBACK:
              {
                  // Set the flag to true to indicate that BLEAppUtil_freeMsg
                  // should be used to free the msg
                  freeMsg = TRUE;
                  switch (pMsgData->event)
                  {
                      case GAP_MSG_EVENT:
                          BLEAppUtil_processGAPEvents(pMsgData);
                          break;

                      case GATT_MSG_EVENT:
                          BLEAppUtil_processGATTEvents(pMsgData);
                          break;

                      case L2CAP_DATA_EVENT:
                          BLEAppUtil_processL2CAPDataMsg(pMsgData);
                          break;

                      case L2CAP_SIGNAL_EVENT:
                          BLEAppUtil_processL2CAPSignalEvents(pMsgData);
                          break;

                      case HCI_GAP_EVENT_EVENT:
                          BLEAppUtil_processHCIGAPEvents(pMsgData);
                          break;

                      case HCI_DATA_EVENT:
                          BLEAppUtil_processHCIDataEvents(pMsgData);
                          break;

                      case HCI_SMP_EVENT_EVENT:
                          BLEAppUtil_processHCISMPEvents(pMsgData);
                          break;

                      case HCI_SMP_META_EVENT_EVENT:
                          BLEAppUtil_processHCISMPMetaEvents(pMsgData);
                          break;

                    case HCI_CTRL_TO_HOST_EVENT:
                    {
                        BLEAppUtil_processHCICTRLToHostEvents(pMsgData);
                        hciPacket_t *pBuf = (hciPacket_t *)pMsgData;
                        switch (pBuf->pData[0])
                        {
                          case HCI_ACL_DATA_PACKET:
                          case HCI_SCO_DATA_PACKET:
                            BM_free(pBuf->pData);
                          default:
                            break;
                        }
                        break;
                    }


                    default:
                        break;
                }
                break;
            }
              case BLEAPPUTIL_EVT_ADV_CB_EVENT:
              {
                  BLEAppUtil_processAdvEventMsg(pMsgData);
                  if (((BLEAppUtil_AdvEventData_t *)pMsgData)->event != BLEAPPUTIL_ADV_INSUFFICIENT_MEMORY &&
                      ((BLEAppUtil_AdvEventData_t *)pMsgData)->pBuf)
                  {
                      BLEAppUtil_free(((BLEAppUtil_AdvEventData_t *)pMsgData)->pBuf);
                  }
                  break;
              }

              case BLEAPPUTIL_EVT_SCAN_CB_EVENT:
              {
                  BLEAppUtil_processScanEventMsg(pMsgData);
                  if (((BLEAppUtil_ScanEventData_t *)pMsgData)->event == BLEAPPUTIL_ADV_REPORT &&
                      ((BLEAppUtil_ScanEventData_t *)pMsgData)->pBuf->pAdvReport.pData)
                  {
                      BLEAppUtil_free(((BLEAppUtil_ScanEventData_t *)pMsgData)->pBuf->pAdvReport.pData);
                  }
                  if (((BLEAppUtil_ScanEventData_t *)pMsgData)->event != BLEAPPUTIL_SCAN_INSUFFICIENT_MEMORY &&
                      ((BLEAppUtil_ScanEventData_t *)pMsgData)->pBuf)
                  {
                      BLEAppUtil_free(((BLEAppUtil_ScanEventData_t *)pMsgData)->pBuf);
                  }
                  break;
              }

              case BLEAPPUTIL_EVT_PAIRING_STATE_CB:
                  BLEAppUtil_processPairStateMsg(pMsgData);
                  break;

              case BLEAPPUTIL_EVT_PASSCODE_NEEDED_CB:
                  BLEAppUtil_processPasscodeMsg(pMsgData);
                  break;

              case BLEAPPUTIL_EVT_CONN_EVENT_CB:
                  BLEAppUtil_processConnEventMsg(pMsgData);
                  break;

              case BLEAPPUTIL_EVT_CALL_IN_BLEAPPUTIL_CONTEXT:
              {
                  ((BLEAppUtil_CallbackToInvoke_t *)pMsgData)->callback(((BLEAppUtil_CallbackToInvoke_t *)pMsgData)->data);

                  // Verify that the data is not NULL before freeing it
                  if(((BLEAppUtil_CallbackToInvoke_t *)pMsgData)->data != NULL)
                  {
                      BLEAppUtil_free(((BLEAppUtil_CallbackToInvoke_t *)pMsgData)->data);
                  }
                  break;
              }

              default:
                  break;
            }

            // Free the data
            if (pMsgData && freeMsg)
            {
                // Use freeMsg
                BLEAppUtil_freeMsg(pMsgData);
            }
            else if (pMsgData)
            {
                // Use free
                BLEAppUtil_free(pMsgData);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// Help functions
/////////////////////////////////////////////////////////////////////////

/*********************************************************************
 * @fn      BLEAppUtil_convertBdAddr2Str
 *
 * @brief   Convert Bluetooth address to string.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
char *BLEAppUtil_convertBdAddr2Str(uint8_t *pAddr)
{
    uint8_t     charCnt;
    char        hex[] = "0123456789ABCDEF";
    static char str[(2*B_ADDR_LEN)+3];
    char        *pStr = str;

    *pStr++ = '0';
    *pStr++ = 'x';

    // Start from end of addr
    pAddr += B_ADDR_LEN;

    for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--)
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }
  *pStr = 0;

    return str;
}
