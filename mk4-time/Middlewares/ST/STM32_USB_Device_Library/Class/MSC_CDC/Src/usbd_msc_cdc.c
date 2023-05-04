/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

#include "usbd_cdc.h"
#include "usbd_msc.h"



// Interface numbers
#define MSC_INTERFACE_IDX 0x0           // Index of MSC interface
#define CDC_INTERFACE_IDX 0x1           // Index of CDC interface

// endpoints numbers
#define MSC_EP_IDX                      0x01
#define CDC_CMD_EP_IDX                  0x02
#define CDC_EP_IDX                      0x03

#define IN_EP_DIR                       0x80 // Adds a direction bit

#define MSC_OUT_EP                      MSC_EP_IDX                     /* EP1 for BULK OUT */
#define MSC_IN_EP                       (MSC_EP_IDX | IN_EP_DIR)       /* EP1 for BULK IN */

#undef CDC_CMD_EP
#undef CDC_OUT_EP
#undef CDC_IN_EP

#define CDC_CMD_EP                      (CDC_CMD_EP_IDX | IN_EP_DIR)   /* EP2 for CDC commands */
#define CDC_OUT_EP                      CDC_EP_IDX                     /* EP3 for data OUT */
#define CDC_IN_EP                       (CDC_EP_IDX | IN_EP_DIR)       /* EP3 for data IN */


#define USB_MSC_CDC_CONFIG_DESC_SIZ       98
/* USB MSC+CDC device Configuration Descriptor */
static const uint8_t USBD_MSC_CDC_CfgDesc[USB_MSC_CDC_CONFIG_DESC_SIZ] =
{
  0x09,         /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_MSC_CDC_CONFIG_DESC_SIZ, /* wTotalLength: Bytes returned */
  0x00,
  0x03,         /*bNumInterfaces: 3 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supports Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 bytes */

  /********************  Mass Storage interface ********************/
  0x09,   /* bLength: Interface Descriptor size */
  0x04,   /* bDescriptorType: */
  MSC_INTERFACE_IDX,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints*/
  0x08,   /* bInterfaceClass: MSC Class */
  0x06,   /* bInterfaceSubClass : SCSI transparent command set*/
  0x50,   /* nInterfaceProtocol */
  USBD_IDX_INTERFACE_STR, /* iInterface: */
  /* 09 bytes */
  /********************  Mass Storage Endpoints ********************/
  0x07,   /*Endpoint descriptor length = 7*/
  0x05,   /*Endpoint descriptor type */
  MSC_IN_EP,   /*Endpoint address (IN, address 1) */
  0x02,   /*Bulk endpoint type */
  LOBYTE(MSC_MAX_FS_PACKET),
  HIBYTE(MSC_MAX_FS_PACKET),
  0x00,   /*Polling interval in milliseconds */
  /* 07 bytes */
  0x07,   /*Endpoint descriptor length = 7 */
  0x05,   /*Endpoint descriptor type */
  MSC_OUT_EP,   /*Endpoint address (OUT, address 1) */
  0x02,   /*Bulk endpoint type */
  LOBYTE(MSC_MAX_FS_PACKET),
  HIBYTE(MSC_MAX_FS_PACKET),
  0x00,     /*Polling interval in milliseconds*/
  /* 07 bytes */

  /******** IAD should be positioned just before the CDC interfaces ******
                          IAD to associate the two CDC interfaces */
  0x08, /* bLength */
  0x0B, /* bDescriptorType */
  CDC_INTERFACE_IDX, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x00, /* iFunction (Index of string descriptor describing this function) */
  /* 08 bytes */

  /********************  CDC interfaces ********************/
    /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_INTERFACE_IDX,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x01,   /* iInterface: */
  /* 09 bytes */

  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  /* 05 bytes */

  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  CDC_INTERFACE_IDX + 1,   /* bDataInterface: 2 */
  /* 05 bytes */

  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  /* 04 bytes */

  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  CDC_INTERFACE_IDX,   /* bMasterInterface: Communication class interface */
  CDC_INTERFACE_IDX + 1,   /* bSlaveInterface0: Data Class Interface */
  /* 05 bytes */

  /*Endpoint 2 Descriptor*/
  0x07,                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,        /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                    /* bEndpointAddress */
  0x03,                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                          /* bInterval: */
  /* 07 bytes */

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,       /* bDescriptorType: */
  CDC_INTERFACE_IDX + 1,         /* bInterfaceNumber: Number of Interface */
  0x00,                          /* bAlternateSetting: Alternate setting */
  0x02,                          /* bNumEndpoints: Two endpoints used */
  0x0A,                          /* bInterfaceClass: CDC */
  0x00,                          /* bInterfaceSubClass: */
  0x00,                          /* bInterfaceProtocol: */
  0x00,                          /* iInterface: */
  /* 09 bytes */

  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,        /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                    /* bEndpointAddress */
  0x02,                          /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                          /* bInterval: ignore for Bulk transfer */
  /* 07 bytes */

  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,        /* bDescriptorType: Endpoint */
  CDC_IN_EP,                     /* bEndpointAddress */
  0x02,                          /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                          /* bInterval */
  /* 07 bytes */

};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN  uint8_t USBD_MSC_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  MSC_MAX_FS_PACKET,
  0x01,
  0x00,
};

/**
* @brief  USBD_MSC_CDC_Init
*         Initialize the MSC+CDC interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t USBD_MSC_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* MSC initialization */
  uint8_t ret = USBD_MSC_Init(pdev, cfgidx);
  if(ret != USBD_OK)
          return ret;
  /* CDC initialization */
  ret = USBD_CDC_Init(pdev, cfgidx);
  if(ret != USBD_OK)
    return ret;
  return USBD_OK;
}
/**
* @brief  USBD_MSC_CDC_Init
*         DeInitialize the MSC+CDC layer
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t USBD_MSC_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* MSC De-initialization */
  USBD_MSC_DeInit(pdev, cfgidx);
  /* CDC De-initialization */
  USBD_CDC_DeInit(pdev, cfgidx);
  return USBD_OK;
}

/**
 * @brief  USBD_MSC_CDC_Setup
 *         Handle the MSC+CDC specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_MSC_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  // Route requests to MSC interface or its endpoints to MSC class implementation
  if(((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE && req->wIndex == MSC_INTERFACE_IDX) ||
    ((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_ENDPOINT && ((req->wIndex & 0x7F) == MSC_EP_IDX)))
  {
    return USBD_MSC_Setup(pdev, req);
  }
  return USBD_CDC_Setup(pdev, req);
}

/**
 * @brief  USBD_MSC_CDC_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MSC_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if(epnum == MSC_EP_IDX)
    return USBD_MSC_DataIn(pdev, epnum);
  return USBD_CDC_DataIn(pdev, epnum);
}
/**
 * @brief  USBD_MSC_CDC_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MSC_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if(epnum == MSC_EP_IDX)
    return USBD_MSC_DataOut(pdev, epnum);
  return USBD_CDC_DataOut(pdev, epnum);
}

/**
  * @brief  USBD_MSC_CDC_EP0_RxReady
  *       handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_MSC_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  return USBD_CDC_EP0_RxReady(pdev);
}

/**
  * @brief  USBD_MSC_CDC_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static const uint8_t  *USBD_MSC_CDC_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_MSC_CDC_CfgDesc);
  return USBD_MSC_CDC_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t *USBD_MSC_CDC_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = sizeof(USBD_MSC_CDC_DeviceQualifierDesc);

  return USBD_MSC_CDC_DeviceQualifierDesc;
}

USBD_ClassTypeDef USBD_MSC_CDC_ClassDriver =
{
      USBD_MSC_CDC_Init,
      USBD_MSC_CDC_DeInit,
      USBD_MSC_CDC_Setup,
      NULL, //USBD_MSC_CDC_EP0_TxReady,
      USBD_MSC_CDC_EP0_RxReady,
      USBD_MSC_CDC_DataIn,
      USBD_MSC_CDC_DataOut,
      NULL, //USBD_MSC_CDC_SOF,
      NULL, //USBD_MSC_CDC_IsoINIncomplete,
      NULL, //USBD_MSC_CDC_IsoOutIncomplete,
      USBD_MSC_CDC_GetCfgDesc,
      USBD_MSC_CDC_GetCfgDesc,
      USBD_MSC_CDC_GetCfgDesc,
      USBD_MSC_CDC_GetDeviceQualifierDesc,
};
