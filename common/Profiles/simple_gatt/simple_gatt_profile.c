/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
        for use with the BLE sample application.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2010 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "common/Profiles/simple_gatt/simple_gatt_profile.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
void SimpleGattProfile_callback( uint8 paramID  );
void SimpleGattProfile_invokeFromFWContext( char *pData );

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern uint8 storage[];
extern uint16 battery_vol;

// Simple GATT Profile Service UUID: 0xFFF0
GATT_BT_UUID(simpleGattProfile_ServUUID, SIMPLEGATTPROFILE_SERV_UUID);

// Characteristic 1 UUID: 0xFFF1
GATT_BT_UUID(simpleGattProfile_char1UUID, SIMPLEGATTPROFILE_CHAR1_UUID);

// Characteristic 2 UUID: 0xFFF2
GATT_BT_UUID(simpleGattProfile_char2UUID, SIMPLEGATTPROFILE_CHAR2_UUID);

// Characteristic 3 UUID: 0xFFF3
GATT_BT_UUID(simpleGattProfile_char3UUID, SIMPLEGATTPROFILE_CHAR3_UUID);

// Characteristic 4 UUID: 0xFFF4
GATT_BT_UUID(simpleGattProfile_char4UUID, SIMPLEGATTPROFILE_CHAR4_UUID);

// Characteristic 5 UUID: 0xFFF5
GATT_BT_UUID(simpleGattProfile_char5UUID, SIMPLEGATTPROFILE_CHAR5_UUID);

// Characteristic 6 UUID: 0xFFF6
GATT_BT_UUID(simpleGattProfile_char6UUID, SIMPLEGATTPROFILE_CHAR6_UUID);

// Characteristic 7 UUID: 0xFFF7
GATT_BT_UUID(simpleGattProfile_char7UUID, SIMPLEGATTPROFILE_CHAR7_UUID);

// Characteristic 8 UUID: 0xFFF8
GATT_BT_UUID(simpleGattProfile_char8UUID, SIMPLEGATTPROFILE_CHAR8_UUID);

// Characteristic 9 UUID: 0xFFF9
GATT_BT_UUID(simpleGattProfile_char9UUID, SIMPLEGATTPROFILE_CHAR9_UUID);

// Characteristic 10 UUID: 0xFF60
GATT_BT_UUID(simpleGattProfile_char10UUID, SIMPLEGATTPROFILE_CHAR10_UUID);


GATT_BT_UUID(passwordGattProfile_ServUUID, PASSWORD_VERIFICATION_SERV_UUID);
GATT_BT_UUID(passwordGattProfile_char1UUID, 0xeee1);
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static SimpleGattProfile_CBs_t *simpleGattProfile_appCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple GATT Profile Service attribute
static CONST gattAttrType_t simpleGattProfile_Service = { ATT_BT_UUID_SIZE, simpleGattProfile_ServUUID };


static uint8 simpleGattProfile_Char1Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char1[SIMPLEGATTPROFILE_CHAR1_LEN] = {0};

static uint8 simpleGattProfile_Char2Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char2[SIMPLEGATTPROFILE_CHAR2_LEN] = {0};

static uint8 simpleGattProfile_Char3Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char3[SIMPLEGATTPROFILE_CHAR3_LEN] = {0};

static uint8 simpleGattProfile_Char4Props = GATT_PROP_READ;
static uint8 simpleGattProfile_Char4[SIMPLEGATTPROFILE_CHAR4_LEN] = {0x68,0x01};

static uint8 simpleGattProfile_Char5Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char5[SIMPLEGATTPROFILE_CHAR5_LEN] = {0};

static uint8 simpleGattProfile_Char6Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char6[SIMPLEGATTPROFILE_CHAR6_LEN] = {0};

static uint8 simpleGattProfile_Char7Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char7[SIMPLEGATTPROFILE_CHAR7_LEN] = {0};

static uint8 simpleGattProfile_Char8Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char8[SIMPLEGATTPROFILE_CHAR8_LEN] = {0};

static uint8 simpleGattProfile_Char9Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char9[SIMPLEGATTPROFILE_CHAR9_LEN] = {0};

static uint8 simpleGattProfile_Char10Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 simpleGattProfile_Char10[SIMPLEGATTPROFILE_CHAR10_LEN] = {0};


static CONST gattAttrType_t passwordGattProfile_Service = { ATT_BT_UUID_SIZE, passwordGattProfile_ServUUID };
static uint8 passwordGattProfile_Char1Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 passwordGattProfile_Char1[18] = {0};
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleGattProfile_attrTbl[] =
{
 /*------------------type-----------------*/ /*-----------permissions-----------*/ /*-----------------pValue----------------*/
   // Simple Profile Service
   GATT_BT_ATT( primaryServiceUUID,           GATT_PERMIT_READ,                      (uint8 *) &simpleGattProfile_Service ),

   // Characteristic 1 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char1Props ),
   // Characteristic Value 1
   GATT_BT_ATT( simpleGattProfile_char1UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char1 ),

   // Characteristic 2 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char2Props ),
   // Characteristic Value 2
   GATT_BT_ATT( simpleGattProfile_char2UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char2 ),

   // Characteristic 3 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char3Props ),
   // Characteristic Value 3
   GATT_BT_ATT( simpleGattProfile_char3UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char3 ),

   // Characteristic 4 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char4Props ),
   // Characteristic Value 4
   GATT_BT_ATT( simpleGattProfile_char4UUID,  GATT_PERMIT_READ,                      simpleGattProfile_Char4 ),

   // Characteristic 5 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char5Props ),
   // Characteristic Value 5
   GATT_BT_ATT( simpleGattProfile_char5UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char5 ),

   // Characteristic 6 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char6Props ),
   // Characteristic Value 6
   GATT_BT_ATT( simpleGattProfile_char6UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char6 ),

   // Characteristic 7 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char7Props ),
   // Characteristic Value 7
   GATT_BT_ATT( simpleGattProfile_char7UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char7 ),

   // Characteristic 8 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char8Props ),
   // Characteristic Value 8
   GATT_BT_ATT( simpleGattProfile_char8UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char8 ),

   // Characteristic 9 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char9Props ),
   // Characteristic Value 9
   GATT_BT_ATT( simpleGattProfile_char9UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char9 ),

   // Characteristic 10 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char10Props ),
   // Characteristic Value 10
   GATT_BT_ATT( simpleGattProfile_char10UUID, GATT_PERMIT_READ | GATT_PERMIT_WRITE,  simpleGattProfile_Char10 ),

   GATT_BT_ATT( primaryServiceUUID,           GATT_PERMIT_READ,                      (uint8 *) &passwordGattProfile_Service ),
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &passwordGattProfile_Char1Props ),
   GATT_BT_ATT( passwordGattProfile_char1UUID,  GATT_PERMIT_READ | GATT_PERMIT_WRITE,  passwordGattProfile_Char1 ),
};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t SimpleGattProfile_readAttrCB( uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t *pLen,
                                        uint16_t offset, uint16_t maxLen,
                                        uint8_t method );
bStatus_t SimpleGattProfile_writeAttrCB( uint16_t connHandle,
                                         gattAttribute_t *pAttr,
                                         uint8_t *pValue, uint16_t len,
                                         uint16_t offset, uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple GATT Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t simpleGattProfile_CBs =
{
  SimpleGattProfile_readAttrCB,  // Read callback function pointer
  SimpleGattProfile_writeAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleGattProfile_addService
 *
 * @brief   This function initializes the Simple GATT Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t SimpleGattProfile_addService( void )
{
  uint8 status = SUCCESS;



  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( simpleGattProfile_attrTbl,
                                        GATT_NUM_ATTRS( simpleGattProfile_attrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &simpleGattProfile_CBs );

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   appCallbacks - pointer to application callback.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t SimpleGattProfile_registerAppCBs( SimpleGattProfile_CBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    simpleGattProfile_appCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      SimpleGattProfile_setParameter
 *
 * @brief   Set a Simple GATT Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate
 *                  data type (example: data type of uint16 will be cast to
 *                  uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t SimpleGattProfile_setParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t status = SUCCESS;

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_getParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleGattProfile_getParameter( uint8 param, void *value )
{
  bStatus_t status = SUCCESS;

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn          SimpleGattProfile_readAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t SimpleGattProfile_readAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen,
                                       uint16_t offset, uint16_t maxLen,
                                       uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
        case SIMPLEGATTPROFILE_CHAR1_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR1_LEN;
            memcpy(pValue,simpleGattProfile_Char1,SIMPLEGATTPROFILE_CHAR1_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR2_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR2_LEN;
            memcpy(pValue,storage,SIMPLEGATTPROFILE_CHAR2_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR3_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR3_LEN;
            memcpy(pValue,storage+16,SIMPLEGATTPROFILE_CHAR3_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR4_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR4_LEN;
            simpleGattProfile_Char4[0] = battery_vol%256;
            simpleGattProfile_Char4[1] = battery_vol/256;
            memcpy(pValue,simpleGattProfile_Char4,SIMPLEGATTPROFILE_CHAR4_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR5_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR5_LEN;
            memcpy(pValue,storage+17,SIMPLEGATTPROFILE_CHAR5_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR6_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR6_LEN;
            memcpy(pValue,storage+19,SIMPLEGATTPROFILE_CHAR6_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR7_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR7_LEN;
            memcpy(pValue,storage+21,SIMPLEGATTPROFILE_CHAR7_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR8_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR8_LEN;
            memcpy(pValue,storage+23,SIMPLEGATTPROFILE_CHAR8_LEN);
            break;

        case SIMPLEGATTPROFILE_CHAR9_UUID:
            *pLen = storage[45];
            memcpy(pValue,storage+31,storage[45]);
            break;

        case SIMPLEGATTPROFILE_CHAR10_UUID:
            *pLen = SIMPLEGATTPROFILE_CHAR10_LEN;
            memcpy(pValue,storage+22,SIMPLEGATTPROFILE_CHAR10_LEN);
            break;

        case 0xeee1:
            *pLen = 18;
            memcpy(pValue,passwordGattProfile_Char1,18);
            break;

      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_writeAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t SimpleGattProfile_writeAttrCB( uint16_t connHandle,
                                     gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len,
                                     uint16_t offset, uint8_t method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {

      uint8 xor_int;
      uint8 w_data[18];
      uint8 xor_end[8];
      uint8 key[8];

    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SIMPLEGATTPROFILE_CHAR1_UUID:
          if( pValue[0]==0xfe && pValue[1]==0x01)
          {
              simpleGattProfile_Char1[0]=0xfe;
              simpleGattProfile_Char1[1]=0x02;
          }
          break;
      case SIMPLEGATTPROFILE_CHAR2_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          memcpy(storage,pValue,SIMPLEGATTPROFILE_CHAR2_LEN);
          break;
      case SIMPLEGATTPROFILE_CHAR3_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          memcpy(storage+16,pValue,SIMPLEGATTPROFILE_CHAR3_LEN);
          break;
      case SIMPLEGATTPROFILE_CHAR5_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          memcpy(storage+17,pValue,SIMPLEGATTPROFILE_CHAR5_LEN);
          break;
      case SIMPLEGATTPROFILE_CHAR6_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          memcpy(storage+19,pValue,SIMPLEGATTPROFILE_CHAR6_LEN);
          break;
      case SIMPLEGATTPROFILE_CHAR7_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          memcpy(storage+21,pValue,SIMPLEGATTPROFILE_CHAR7_LEN);
          break;
      case SIMPLEGATTPROFILE_CHAR8_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          memcpy(storage+23,pValue,SIMPLEGATTPROFILE_CHAR8_LEN);
          break;
      case SIMPLEGATTPROFILE_CHAR9_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          {
              memcpy(storage+31,pValue,len);
              storage[45]=len;
          }
          break;
      case SIMPLEGATTPROFILE_CHAR10_UUID:
          if(simpleGattProfile_Char1[0]==0xfe && simpleGattProfile_Char1[1]==0x02)
          memcpy(storage+22,pValue,SIMPLEGATTPROFILE_CHAR10_LEN);
          break;

      case 0xeee1:

          memcpy(w_data,pValue,18);
          xor_int = w_data[16] + w_data[17];

          xor_end[0] = w_data[0]+xor_int;
          xor_end[1] = w_data[1]+xor_int;
          xor_end[2] = w_data[2]+xor_int;
          xor_end[3] = w_data[3]+xor_int;
          xor_end[4] = w_data[8]+xor_int;
          xor_end[5] = w_data[9]+xor_int;
          xor_end[6] = w_data[10]+xor_int;
          xor_end[7] = w_data[11]+xor_int;

          key[0] = xor_end[0]^w_data[4];
          key[1] = xor_end[1]^w_data[5];
          key[2] = xor_end[2]^w_data[6];
          key[3] = xor_end[3]^w_data[7];
          key[4] = xor_end[4]^w_data[12];
          key[5] = xor_end[5]^w_data[13];
          key[6] = xor_end[6]^w_data[14];
          key[7] = xor_end[7]^w_data[15];

          if(memcmp(key,storage+23,8) == 0)
          {
              memset(passwordGattProfile_Char1,0x55,18);
          }
          else
          {
              memset(passwordGattProfile_Char1,0xaa,18);
          }
          break;

      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ((notifyApp != 0xFF ) && simpleGattProfile_appCBs && simpleGattProfile_appCBs->pfnSimpleGattProfile_Change)
  {
      SimpleGattProfile_callback( notifyApp );
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_callback
 *
 * @brief   This function will be called from the BLE App Util module
 *          context.
 *          Calling the application callback
 *
 * @param   pData - data
 *
 * @return  None
 */
void SimpleGattProfile_callback( uint8 paramID )
{
  char *pData = ICall_malloc(sizeof(char));

  if(pData == NULL)
  {
    return;
  }

  pData[0] = paramID;

  BLEAppUtil_invokeFunction(SimpleGattProfile_invokeFromFWContext, pData);
}

/*********************************************************************
 * @fn      SimpleGattProfile_invokeFromFWContext
 *
 * @brief   This function will be called from the BLE App Util module
 *          context.
 *          Calling the application callback
 *
 * @param   pData - data
 *
 * @return  None
 */
void SimpleGattProfile_invokeFromFWContext( char *pData )
{
  simpleGattProfile_appCBs->pfnSimpleGattProfile_Change(pData[0]);
}

/*********************************************************************
*********************************************************************/
