/**
  ******************************************************************************
  * @file           : usbd_composite_if.c
  * @version        : V1.0
  * @brief          : 组合USB设备应用层
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_composite.h"
#include "usbd_composite_if.h"
#include "stdarg.h"
#include "stm32h7xx.h"
#include "sdmmc.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
#define STORAGE_LUN_NBR			1			/**< 盘符数量 */
#define STORAGE_BLK_NBR			0x10000		/**< 扇区数量 */
#define STORAGE_BLK_SIZ			0x200		/**< 扇区大小 */
/* Macro ---------------------------------------------------------------------*/
/* CDC操作接口静态函数 */
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* MSC操作接口静态函数 */
static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* Variables -----------------------------------------------------------------*/
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */
	/* LUN 0 */
	0x00,
	0x80,
	0x02,
	0x02,
	(STANDARD_INQUIRY_DATA_LEN - 5),
	0x00,
	0x00,
	0x00,
	'Y', 'G', 'D', 'L', ' ', ' ', ' ', ' ',	/* 制造商 : 8 bytes  */
	'M', 'a', 's', 's', ' ', 'S', 't', 'o',	/* 产品   : 16 Bytes */
	'r', 'a', 'g', 'e', ' ', ' ', ' ', ' ',
	'1', '.', '0' ,'0'						/* 版本   : 4 Bytes  */
};

/* CDC操作函数接口 */
USBD_CDC_ItfTypeDef USBD_CDC_Interface_fops_FS =
{
	CDC_Init_FS,
	CDC_DeInit_FS,
	CDC_Control_FS,
	CDC_Receive_FS,
	CDC_TransmitCplt_FS,
};

/* MSC操作函数接口 */
USBD_StorageTypeDef USBD_MSC_Interface_fops_FS =
{
	STORAGE_Init_FS,
	STORAGE_GetCapacity_FS,
	STORAGE_IsReady_FS,
	STORAGE_IsWriteProtected_FS,
	STORAGE_Read_FS,
	STORAGE_Write_FS,
	STORAGE_GetMaxLun_FS,
	(int8_t *)STORAGE_Inquirydata_FS,
};

/* CDC特有类 */
USBD_CDC_LineCodingTypeDef linecoding =
{
	115200, /* baud rate*/
	0x00,   /* stop bits-1*/
	0x00,   /* parity - none*/
	0x08    /* nb. of bits 8*/
};

/* 为接收和传输创建缓冲区，这取决于用户重新定义和/或删除这些定义 */
/* 通过USB接收的数据被存储在这个缓冲区中 */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* 通过USB CDC发送的数据存储在这个缓冲区中 */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];


extern USBD_HandleTypeDef hUsbDeviceFS;

/**
  * @brief  通过FS USB IP初始化CDC介质低层
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
	/* 设置应用程序缓冲区 */
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
	return (USBD_OK);
}

/**
  * @brief  取消初始化CDC介质低层
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
	return (USBD_OK);
}

/**
  * @brief  管理CDC类请求
  * @param  cmd: 命令代码
  * @param  pbuf: 缓冲区包含命令数据(请求参数)
  * @param  length: 要发送的数据长度(以字节为单位)
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	switch(cmd)
	{
		case CDC_SEND_ENCAPSULATED_COMMAND:

			break;

		case CDC_GET_ENCAPSULATED_RESPONSE:

			break;

		case CDC_SET_COMM_FEATURE:

			break;

		case CDC_GET_COMM_FEATURE:

			break;

		case CDC_CLEAR_COMM_FEATURE:

			break;

		/*******************************************************************************/
		/* 行编码结构                                                                   */
		/*-----------------------------------------------------------------------------*/
		/* Offset | Field       | Size | Value  | Description                          */
		/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
		/* 4      | bCharFormat |   1  | Number | Stop bits                            */
		/*                                        0 - 1 Stop bit                       */
		/*                                        1 - 1.5 Stop bits                    */
		/*                                        2 - 2 Stop bits                      */
		/* 5      | bParityType |  1   | Number | Parity                               */
		/*                                        0 - None                             */
		/*                                        1 - Odd                              */
		/*                                        2 - Even                             */
		/*                                        3 - Mark                             */
		/*                                        4 - Space                            */
		/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
		/*******************************************************************************/
		case CDC_SET_LINE_CODING:
			linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
			linecoding.format     = pbuf[4];
			linecoding.paritytype = pbuf[5];
			linecoding.datatype   = pbuf[6];
			break;

		case CDC_GET_LINE_CODING:
			pbuf[0] = (uint8_t)(linecoding.bitrate);
			pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
			pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
			pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
			pbuf[4] = linecoding.format;
			pbuf[5] = linecoding.paritytype;
			pbuf[6] = linecoding.datatype;
			break;

		case CDC_SET_CONTROL_LINE_STATE:

			break;

		case CDC_SEND_BREAK:

			break;

		default:
			break;
	}

	return (USBD_OK);
}

/**
  * @brief  通过USB OUT端点接收的数据通过CDC接口发送。
  *
  *         @note
  *         该函数将在USB端点接收到的任何OUT数据包上发出一个NAK数据包，
  *         直到退出该函数。如果您在CDC接口上完成传输之前退出此功能(即：
  *         使用DMA控制器)它将导致接收更多的数据，而之前的数据仍然没有发送。
  *
  * @param  Buf: 要接收的数据的缓冲区
  * @param  Len: 接收的数据长度(以字节为单位)
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	CDC_Transmit_FS(UserRxBufferFS, *Len);
	return (USBD_OK);
}

/**
  * @brief  通过USB IN端点发送的数据通过CDC接口发送。
  *         
  *         @note
  *
  * @param  Buf: 要发送的数据的缓冲区
  * @param  Len: 发送的数据长度(以字节为单位)
  * @return 操作状态
  * @retval USBD_OK如果所有的操作都是OK的，否则USBD_FAIL或USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
	uint8_t result = USBD_OK;
	uint32_t TimeStart = HAL_GetTick();
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

	while(hcdc->TxState)
	{
		if(HAL_GetTick() - TimeStart > 10)
			return USBD_BUSY;
		else
			break;
	}
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	while(hcdc->TxState)
	{
		if(HAL_GetTick() - TimeStart > 10)
			return USBD_BUSY;
		else
			break;
	}

	return result;
}

/**
  * @brief  数据传输回调
  *         
  *         @note
  *         此功能是IN传输完成回调，用于通知用户提交的数据通过USB成功发送。
  *
  * @param  Buf: 要发送的数据的缓冲区
  * @param  Len: 发送的数据长度(以字节为单位)
  * @param  epnum: 发送的端点号
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
	uint8_t result = USBD_OK;

	UNUSED(Buf);
	UNUSED(Len);

	return result;
}

/**
  * @brief  usb打印函数
  * @param  format: 字符串指针，可带不定长度变量
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
uint8_t usb_printf(const char *format, ...)
{
	va_list args;
	uint8_t result = USBD_OK;
	uint32_t length;

	va_start(args, format);
	length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
	va_end(args);
	result = CDC_Transmit_FS(UserTxBufferFS, length);

	return (uint8_t)result;
}

/* ------------------------------------- MSC -------------------------------------------- */

/**
  * @brief  通过USB FS IP初始化存储单元(介质)
  * @param  lun: 逻辑单元号
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
	UNUSED(lun);

	return (USBD_OK);
}

/**
  * @brief  返回中等容量
  * @param  lun: 逻辑单元号
  * @param  block_num: 总块数
  * @param  block_size: 块大小
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
	UNUSED(lun);
	HAL_SD_CardInfoTypeDef info;

	if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
	{
		HAL_SD_GetCardInfo(&hsd1, &info);
		*block_num  = info.LogBlockNbr;
		*block_size = info.LogBlockSize;
		return (USBD_OK);
	}

	return (USBD_FAIL);
}

/**
  * @brief  检查介质是否准备好
  * @param  lun: 逻辑单元号
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
	UNUSED(lun);

	return (USBD_OK);
}

/**
  * @brief  检查介质是否有写保护
  * @param  lun: 逻辑单元号
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
	UNUSED(lun);

	return (USBD_OK);
}

/**
  * @brief  从介质中读取数据
  * @param  lun: 逻辑单元号
  * @param  buf: 数据缓存
  * @param  blk_addr: 逻辑块地址
  * @param  blk_len: 块数量
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
	UNUSED(lun);
	UNUSED(buf);
	UNUSED(blk_addr);
	UNUSED(blk_len);
	int8_t ret = USBD_FAIL;

	if(HAL_SD_ReadBlocks(&hsd1, buf, blk_addr, blk_len, HAL_MAX_DELAY) == HAL_OK)
	{
		ret = USBD_OK;
		
		while(HAL_SD_GetState(&hsd1) == HAL_SD_STATE_BUSY);
		while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);    
	}

	return ret;
}

/**
  * @brief  将数据写入介质
  * @param  lun: 逻辑单元号
  * @param  buf: 数据缓存
  * @param  blk_addr: 逻辑块地址
  * @param  blk_len: 块数量
  * @return 操作状态
  * @retval USBD_OK，如果所有操作都是OK，否则USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
	UNUSED(lun);
	UNUSED(buf);
	UNUSED(blk_addr);
	UNUSED(blk_len);
	int8_t ret = USBD_FAIL;

	if(HAL_SD_WriteBlocks(&hsd1, buf, blk_addr, blk_len, HAL_MAX_DELAY) == HAL_OK)
	{
		ret = USBD_OK;
		
		while(HAL_SD_GetState(&hsd1) == HAL_SD_STATE_BUSY);
		while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);    
	}

	return (USBD_OK);
}

/**
  * @brief  返回最大支持的盘符数量。
  * @param  None
  * @retval 盘符数量.
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
	return (STORAGE_LUN_NBR - 1);
}

