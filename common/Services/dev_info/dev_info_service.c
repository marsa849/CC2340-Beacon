/******************************************************************************

 @file  dev_info_service.c

 @brief This file contains the Device Information service.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2012 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "common/Services/dev_info/dev_info_service.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Device information service
GATT_BT_UUID(devInfoServUUID, DEVINFO_SERV_UUID);

// System ID
GATT_BT_UUID(devInfoSystemIdUUID, SYSTEM_ID_UUID);

// Model Number String
GATT_BT_UUID(devInfoModelNumberUUID, MODEL_NUMBER_UUID);

// Serial Number String
GATT_BT_UUID(devInfoSerialNumberUUID, SERIAL_NUMBER_UUID);

// Firmware Revision String
GATT_BT_UUID(devInfoFirmwareRevUUID, FIRMWARE_REV_UUID);

// Hardware Revision String
GATT_BT_UUID(devInfoHardwareRevUUID, HARDWARE_REV_UUID);

// Current Time String
GATT_BT_UUID(devInfoSoftwareRevUUID, CURRENT_TIME_UUID);

// Manufacturer Name String
GATT_BT_UUID(devInfoMfrNameUUID, MANUFACTURER_NAME_UUID);

// IEEE 11073-20601 Regulatory Certification Data List
GATT_BT_UUID(devInfo11073CertUUID, IEEE_11073_CERT_DATA_UUID);

// PnP ID
GATT_BT_UUID(devInfoPnpIdUUID, PNP_ID_UUID);

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * Profile Attributes - variables
 */

// Device Information Service attribute
static CONST gattAttrType_t devInfoService = GATT_ATT_BT_UUID_TYPE(devInfoServUUID);

// System ID characteristic
static uint8 devInfoSystemIdProps = GATT_PROP_READ;
static uint8 devInfoSystemId[DEVINFO_SYSTEM_ID_LEN] = {0, 0, 0, 0, 0, 0};

// Model Number String characteristic
static uint8 devInfoModelNumberProps = GATT_PROP_READ;
uint8 devInfoModelNumber[11] = "2019-01-01";

// Serial Number String characteristic
static uint8 devInfoSerialNumberProps = GATT_PROP_READ;
uint8 devInfoSerialNumber[13] = "a02c2f151b15";

// Firmware Revision String characteristic
static uint8 devInfoFirmwareRevProps = GATT_PROP_READ;
uint8 devInfoFirmwareRev[15] = "FMVERSION_1000";

// Hardware Revision String characteristic
static uint8 devInfoHardwareRevProps = GATT_PROP_READ;
uint8 devInfoHardwareRev[15] = "HWVERSION_1000";

// Software Revision String characteristic
static uint8 devInfoSoftwareRevProps = GATT_PROP_READ;
uint8 devInfoSoftwareRev[11] = "2019-01-01";

// Manufacturer Name String characteristic
static uint8 devInfoMfrNameProps = GATT_PROP_READ;
static uint8 devInfoMfrName[DEVINFO_STR_ATTR_LEN+1] = "Manufacturer Name";

// IEEE 11073-20601 Regulatory Certification Data List characteristic
static uint8 devInfo11073CertProps = GATT_PROP_READ;
static uint8 defaultDevInfo11073Cert[] =
{
  DEVINFO_11073_BODY_EXP,     // authoritative body type
  0x00,                       // authoritative body structure type
                              // authoritative body data follows below:
  'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l'
};

// The length of this characteristic is not fixed
static uint8 *devInfo11073Cert = defaultDevInfo11073Cert;
static uint8 devInfo11073CertLen = sizeof(defaultDevInfo11073Cert);

// PnP ID characteristic
static uint8 devInfoPnpIdProps = GATT_PROP_READ;
static uint8 devInfoPnpId[DEVINFO_PNP_ID_LEN] =
{
  1,                                      // Vendor ID source (1=Bluetooth SIG)
  LO_UINT16(0x000D), HI_UINT16(0x000D),   // Vendor ID (Texas Instruments)
  LO_UINT16(0x0000), HI_UINT16(0x0000),   // Product ID (vendor-specific)
  LO_UINT16(0x0110), HI_UINT16(0x0110)    // Product version (JJ.M.N)
};

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t devInfoAttrTbl[] =
{
  /*---------------type----------------*/  /*--permissions--*/  /*-----------pValue-----------*/

  // Device Information Service
  GATT_BT_ATT( primaryServiceUUID,          GATT_PERMIT_READ,       (uint8 *)&devInfoService ),

  // System ID Declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &devInfoSystemIdProps ),
  // System ID Value
  GATT_BT_ATT( devInfoSystemIdUUID,         GATT_PERMIT_READ,       (uint8 *) devInfoSystemId ),

  // Model Number String Declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &devInfoModelNumberProps ),
  // Model Number Value
  GATT_BT_ATT( devInfoModelNumberUUID,      GATT_PERMIT_READ,       (uint8 *) devInfoModelNumber ),

  // Serial Number String Declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &devInfoSerialNumberProps ),
  // Serial Number Value
  GATT_BT_ATT( devInfoSerialNumberUUID,     GATT_PERMIT_READ,       (uint8 *) devInfoSerialNumber ),

  // Firmware Revision String Declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &devInfoFirmwareRevProps ),
  // Firmware Revision Value
  GATT_BT_ATT( devInfoFirmwareRevUUID,      GATT_PERMIT_READ,       (uint8 *) devInfoFirmwareRev ),

  // Hardware Revision String Declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &devInfoHardwareRevProps ),
  // Hardware Revision Value
  GATT_BT_ATT( devInfoHardwareRevUUID,      GATT_PERMIT_READ,       (uint8 *) devInfoHardwareRev ),

  // Software Revision String Declaration
  GATT_BT_ATT( characterUUID,               GATT_PERMIT_READ,       &devInfoSoftwareRevProps ),
  // Software Revision Value
  GATT_BT_ATT( devInfoSoftwareRevUUID,      GATT_PERMIT_READ,       (uint8 *) devInfoSoftwareRev ),

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t DevInfo_readAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Device Info Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t devInfoCBs =
{
  DevInfo_readAttrCB, // Read callback function pointer
  NULL,               // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      DevInfo_AddService
 *
 * @brief   Initializes the Device Information service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t DevInfo_addService( void )
{
  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( devInfoAttrTbl,
                                      GATT_NUM_ATTRS( devInfoAttrTbl ),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &devInfoCBs );
}

/*********************************************************************
 * @fn      DevInfo_setParameter
 *
 * @brief   Set a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_setParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
     case DEVINFO_SYSTEM_ID:
      // verify length
      if (len == sizeof(devInfoSystemId))
      {
        memcpy(devInfoSystemId, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_MODEL_NUMBER:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoModelNumber, 0, 11);
        memcpy(devInfoModelNumber, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case DEVINFO_SERIAL_NUMBER:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoSerialNumber, 0, 13);
        memcpy(devInfoSerialNumber, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_FIRMWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoFirmwareRev, 0, 15);
        memcpy(devInfoFirmwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_HARDWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoHardwareRev, 0, 15);
        memcpy(devInfoHardwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_SOFTWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoSoftwareRev, 0, 11);
        memcpy(devInfoSoftwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      DevInfo_getParameter
 *
 * @brief   Get a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_getParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case DEVINFO_SYSTEM_ID:
      memcpy(value, devInfoSystemId, sizeof(devInfoSystemId));
      break;

    case DEVINFO_MODEL_NUMBER:
      memcpy(value, devInfoModelNumber, DEVINFO_STR_ATTR_LEN);
      break;
    case DEVINFO_SERIAL_NUMBER:
      memcpy(value, devInfoSerialNumber, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_FIRMWARE_REV:
      memcpy(value, devInfoFirmwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_HARDWARE_REV:
      memcpy(value, devInfoHardwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_SOFTWARE_REV:
      memcpy(value, devInfoSoftwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_MANUFACTURER_NAME:
      memcpy(value, devInfoMfrName, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_11073_CERT_DATA:
      memcpy(value, devInfo11073Cert, devInfo11073CertLen);
      break;

    case DEVINFO_PNP_ID:
      memcpy(value, devInfoPnpId, sizeof(devInfoPnpId));
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          DevInfo_ReadAttrCB
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
static bStatus_t DevInfo_readAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // If the value offset of the Read Blob Request is greater than the
  // length of the attribute value, an Error Response shall be sent with
  // the error code Invalid Offset.
  switch (uuid)
  {
    case SYSTEM_ID_UUID:
      // verify offset
      if (offset > sizeof(devInfoSystemId))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length
        *pLen = MIN(maxLen, (sizeof(devInfoSystemId) - offset));

        // copy data
        memcpy(pValue, &devInfoSystemId[offset], *pLen);
      }
      break;

    case MODEL_NUMBER_UUID:
        *pLen = 11;
        memcpy(pValue, devInfoModelNumber, 11);
        break;
    case SERIAL_NUMBER_UUID:
        *pLen = 13;
        memcpy(pValue, devInfoSerialNumber, 13);
        break;
    case FIRMWARE_REV_UUID:
        *pLen = 15;
        memcpy(pValue, devInfoFirmwareRev, 15);
        break;
    case HARDWARE_REV_UUID:
        *pLen = 15;
        memcpy(pValue, devInfoHardwareRev, 15);
        break;
    case CURRENT_TIME_UUID:
        *pLen = 11;
        memcpy(pValue, devInfoSoftwareRev, 11);
        break;


    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/
