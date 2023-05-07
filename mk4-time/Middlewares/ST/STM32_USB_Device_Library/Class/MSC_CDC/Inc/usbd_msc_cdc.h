/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_MSC_CDC_H
#define __USB_MSC_CDC_H

#ifdef __cplusplus
extern "C" {
#endif


#include  "usbd_ioreq.h"

extern USBD_ClassTypeDef  USBD_MSC_CDC_ClassDriver;


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

#define CDC_CMD_EP                      (CDC_CMD_EP_IDX | IN_EP_DIR)   /* EP2 for CDC commands */
#define CDC_OUT_EP                      CDC_EP_IDX                     /* EP3 for data OUT */
#define CDC_IN_EP                       (CDC_EP_IDX | IN_EP_DIR)       /* EP3 for data IN */


#ifdef __cplusplus
}
#endif

#endif  /* __USB_MSC_CDC_H */
