#ifndef __BOOT_H
#define __BOOT_H

#define DATE_CMD_REPORT_CRC         0x9E
#define DATE_CMD_START_BOOTLOADER   0x9F

#define BOOT_CMD_GO                 0x21

uint8_t doDateUpdate(void);


#endif /* __BOOT_H */
