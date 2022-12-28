/**
  ******************************************************************************
  * @file           : usbd_composite_if.h
  * @version        : V1.0
  * @brief          : usbd_composite_if.c的头文件
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 Sunshine Circuit.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_COMPOSITE_IF_H__
#define __USBD_COMPOSITE_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbd_composite.h"
#include "stdbool.h"

/* 定义CDC上接收和传输缓冲区的大小 */
#define APP_RX_DATA_SIZE	0x800
#define APP_TX_DATA_SIZE	0x800
#define Recive_Finish		true
#define Recive_UnFinish		false
#define New_Package			true
#define Old_Package			false

extern bool Recive_State;	/**< 接收状态 */
extern bool Tag;			/**< 下一个包状态 */
extern uint16_t Length;		/**< 包长 */
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* 操作接口句柄 */
extern USBD_CDC_ItfTypeDef USBD_CDC_Interface_fops_FS;
extern USBD_StorageTypeDef USBD_MSC_Interface_fops_FS;

/* 外部函数 */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
uint8_t usb_printf(const char *format, ...);
uint8_t usb_scanf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_COMPOSITE_IF_H__ */

