/**
  ******************************************************************************
  * @file    usbd_composite.h
  * @author  Sunshine Circuit
  * @brief   usbd_composite.c的头文件
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
#ifndef __USBD_COMPOSITE_H
#define __USBD_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"
#include "usbd_ioreq.h"

#define COM_CDC_IN_EP									0x81U		/**< 端点1，输入 */
#define COM_CDC_OUT_EP									0x01U		/**< 端点1，输出 */
#define COM_CDC_CMD_EP									0x82U		/**< 端点2，中断控制端点*/
#define COM_MSC_IN_EP									0x83U		/**< 端点3，输入 */
#define COM_MSC_OUT_EP									0x03U		/**< 端点3，输出 */

/* 控制传输：高速模式的最大包长固定为64个字节；全速模式可在8、16、32、64字节中选择；低速模式的最大包长固定为8个字节。
   批量传输：高速模式固定为512个字节；全速模式最大包长可在8、16、32、64字节中选择；低速模式不支持批量传输。
   同步传输：高速模式的最大包长上限为1024个字节；全速模式的最大包长上限为1023个字节；低速模式不支持同步传输。
   中断传输：告诉模式的最大包长上限为1024个字节；全速模式最大包长上限为64个字节；低速模式最大最大包长上限为8个字节。 */
#define COM_CDC_DATA_MAX_PACK_SIZE						0x40U		/**< 传输数据包大小 */
#define COM_CDC_CMD_PACK_SIZE							0x08U		/**< 控制包大小 */
#define COM_MSC_DATA_MAX_PACK_SIZE						0x40U		/**< MSC端点数据包大小 */

#define COM_CDC_FS_BINTERVAL							0x10U		/**< 控制端点查询时间 */
#define COM_CDC_HS_BINTERVAL							0x10U		/**< 控制端点查询时间 */

#define USB_COM_COMFIG_DESC_SIZ							106U

/* ----------------------------------------------------------------------------------------------------- */
#define CDC_REQ_MAX_DATA_SIZE							0x07U
#define CDC_SEND_ENCAPSULATED_COMMAND					0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE					0x01U
#define CDC_SET_COMM_FEATURE							0x02U
#define CDC_GET_COMM_FEATURE							0x03U
#define CDC_CLEAR_COMM_FEATURE							0x04U
#define CDC_SET_LINE_CODING								0x20U
#define CDC_GET_LINE_CODING								0x21U
#define CDC_SET_CONTROL_LINE_STATE						0x22U
#define CDC_SEND_BREAK									0x23U

/* ----------------------------------------------------------------------------------------------------- */
#define BOT_GET_MAX_LUN									0xFE
#define BOT_RESET										0xFF 

/* ----------------------------------------------------------------------------------------------------- */
#define USBD_BOT_IDLE									0U			/* Idle state */
#define USBD_BOT_DATA_OUT								1U			/* Data Out state */
#define USBD_BOT_DATA_IN								2U			/* Data In state */
#define USBD_BOT_LAST_DATA_IN							3U			/* Last Data In Last */
#define USBD_BOT_SEND_DATA								4U			/* Send Immediate data */
#define USBD_BOT_NO_DATA								5U			/* No data Stage */

#define USBD_BOT_CBW_SIGNATURE							0x43425355U
#define USBD_BOT_CSW_SIGNATURE							0x53425355U
#define USBD_BOT_CBW_LENGTH								31U
#define USBD_BOT_CSW_LENGTH								13U
#define USBD_BOT_MAX_DATA								256U

/* CSW Status Definitions */
#define USBD_CSW_CMD_PASSED								0x00U
#define USBD_CSW_CMD_FAILED								0x01U
#define USBD_CSW_PHASE_ERROR							0x02U

/* BOT Status */
#define USBD_BOT_STATUS_NORMAL							0U
#define USBD_BOT_STATUS_RECOVERY						1U
#define USBD_BOT_STATUS_ERROR							2U


#define USBD_DIR_IN										0U
#define USBD_DIR_OUT									1U
#define USBD_BOTH_DIR									2U

/* ----------------------------------------------------------------------------------------------------- */
#define MODE_SENSE6_LEN									0x17U
#define MODE_SENSE10_LEN								0x1BU
#define LENGTH_INQUIRY_PAGE00							0x06U
#define LENGTH_INQUIRY_PAGE80							0x08U
#define LENGTH_FORMAT_CAPACITIES						0x14U

/* ----------------------------------------------------------------------------------------------------- */
#define SENSE_LIST_DEEPTH								4U

/* SCSI命令 */
#define SCSI_FORMAT_UNIT								0x04U
#define SCSI_INQUIRY									0x12U
#define SCSI_MODE_SELECT6								0x15U
#define SCSI_MODE_SELECT10								0x55U
#define SCSI_MODE_SENSE6								0x1AU
#define SCSI_MODE_SENSE10								0x5AU
#define SCSI_ALLOW_MEDIUM_REMOVAL						0x1EU
#define SCSI_READ6										0x08U
#define SCSI_READ10										0x28U
#define SCSI_READ12										0xA8U
#define SCSI_READ16										0x88U

#define SCSI_READ_CAPACITY10							0x25U
#define SCSI_READ_CAPACITY16							0x9EU

#define SCSI_REQUEST_SENSE								0x03U
#define SCSI_START_STOP_UNIT							0x1BU
#define SCSI_TEST_UNIT_READY							0x00U
#define SCSI_WRITE6										0x0AU
#define SCSI_WRITE10									0x2AU
#define SCSI_WRITE12									0xAAU
#define SCSI_WRITE16									0x8AU

#define SCSI_VERIFY10									0x2FU
#define SCSI_VERIFY12									0xAFU
#define SCSI_VERIFY16									0x8FU

#define SCSI_SEND_DIAGNOSTIC							0x1DU
#define SCSI_READ_FORMAT_CAPACITIES						0x23U

#define NO_SENSE										0U
#define RECOVERED_ERROR									1U
#define NOT_READY										2U
#define MEDIUM_ERROR									3U
#define HARDWARE_ERROR									4U
#define ILLEGAL_REQUEST									5U
#define UNIT_ATTENTION									6U
#define DATA_PROTECT									7U
#define BLANK_CHECK										8U
#define VENDOR_SPECIFIC									9U
#define COPY_ABORTED									10U
#define ABORTED_COMMAND									11U
#define VOLUME_OVERFLOW									13U
#define MISCOMPARE										14U

#define INVALID_CDB										0x20U
#define INVALID_FIELED_IN_COMMAND						0x24U
#define PARAMETER_LIST_LENGTH_ERROR						0x1AU
#define INVALID_FIELD_IN_PARAMETER_LIST					0x26U
#define ADDRESS_OUT_OF_RANGE							0x21U
#define MEDIUM_NOT_PRESENT								0x3AU
#define MEDIUM_HAVE_CHANGED								0x28U
#define WRITE_PROTECTED									0x27U
#define UNRECOVERED_READ_ERROR							0x11U
#define WRITE_FAULT										0x03U

#define READ_FORMAT_CAPACITY_DATA_LEN					0x0CU
#define READ_CAPACITY10_DATA_LEN						0x08U
#define REQUEST_SENSE_DATA_LEN							0x12U
#define STANDARD_INQUIRY_DATA_LEN						0x24U
#define BLKVFY											0x04U

#define SCSI_MEDIUM_UNLOCKED							0x00U
#define SCSI_MEDIUM_LOCKED								0x01U
#define SCSI_MEDIUM_EJECTED								0x02U

/* ----------------------------------------------------------------------------------------------------- */

typedef struct
{
	uint32_t bitrate;
	uint8_t  format;
	uint8_t  paritytype;
	uint8_t  datatype;
}USBD_CDC_LineCodingTypeDef;

typedef struct _USBD_CDC_Itf
{
	int8_t (* Init)(void);
	int8_t (* DeInit)(void);
	int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
	int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
	int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
}USBD_CDC_ItfTypeDef;


typedef struct
{
	uint32_t data[COM_CDC_DATA_MAX_PACK_SIZE / 4U];      /* 强制32位对齐 */
	uint8_t  CmdOpCode;
	uint8_t  CmdLength;
	uint8_t  *RxBuffer;
	uint8_t  *TxBuffer;
	uint32_t RxLength;
	uint32_t TxLength;

	__IO uint32_t TxState;
	__IO uint32_t RxState;
}USBD_CDC_HandleTypeDef;

/* ----------------------------------------------------------------------------------------------------- */
typedef struct _SENSE_ITEM
{
	uint8_t Skey;
	union
	{
		struct _ASCs
		{
			uint8_t ASC;
			uint8_t ASCQ;
		} b;
		uint8_t ASC;
		uint8_t *pData;
	} w;
}USBD_SCSI_SenseTypeDef;

/* ----------------------------------------------------------------------------------------------------- */

typedef struct
{
	uint32_t dSignature;
	uint32_t dTag;
	uint32_t dDataLength;
	uint8_t  bmFlags;
	uint8_t  bLUN;
	uint8_t  bCBLength;
	uint8_t  CB[16];
	uint8_t  ReservedForAlign;
}USBD_MSC_BOT_CBWTypeDef;


typedef struct
{
	uint32_t dSignature;
	uint32_t dTag;
	uint32_t dDataResidue;
	uint8_t  bStatus;
	uint8_t  ReservedForAlign[3];
}USBD_MSC_BOT_CSWTypeDef;

/* ----------------------------------------------------------------------------------------------------- */

typedef struct _USBD_STORAGE
{
	int8_t (* Init)(uint8_t lun);
	int8_t (* GetCapacity)(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
	int8_t (* IsReady)(uint8_t lun);
	int8_t (* IsWriteProtected)(uint8_t lun);
	int8_t (* Read)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
	int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
	int8_t (* GetMaxLun)(void);
	int8_t *pInquiry;
}USBD_StorageTypeDef;

typedef struct
{
	uint32_t					max_lun;
	uint32_t					interface;
	uint8_t						bot_state;
	uint8_t						bot_status;
	uint32_t					bot_data_length;
	uint8_t						bot_data[MSC_MEDIA_PACKET];
	USBD_MSC_BOT_CBWTypeDef		cbw;
	USBD_MSC_BOT_CSWTypeDef		csw;

	USBD_SCSI_SenseTypeDef		scsi_sense [SENSE_LIST_DEEPTH];
	uint8_t						scsi_sense_head;
	uint8_t						scsi_sense_tail;
	uint8_t						scsi_medium_state;

	uint16_t					scsi_blk_size;
	uint32_t					scsi_blk_nbr;

	uint32_t					scsi_blk_addr;
	uint32_t					scsi_blk_len;
}USBD_MSC_BOT_HandleTypeDef;

/* ----------------------------------------------------------------------------------------------------- */

uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_CDC_ItfTypeDef *fops);
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint32_t length);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev);

/* ----------------------------------------------------------------------------------------------------- */

uint8_t  USBD_MSC_RegisterInterface(USBD_HandleTypeDef   *pdev, USBD_StorageTypeDef *fops);

/* ----------------------------------------------------------------------------------------------------- */
void MSC_BOT_Init(USBD_HandleTypeDef  *pdev);
void MSC_BOT_Reset(USBD_HandleTypeDef  *pdev);
void MSC_BOT_DeInit(USBD_HandleTypeDef  *pdev);
void MSC_BOT_DataIn(USBD_HandleTypeDef  *pdev, uint8_t epnum);
void MSC_BOT_DataOut(USBD_HandleTypeDef  *pdev, uint8_t epnum);
void MSC_BOT_SendCSW(USBD_HandleTypeDef  *pdev, uint8_t CSW_Status);
void  MSC_BOT_CplClrFeature(USBD_HandleTypeDef  *pdev, uint8_t epnum);

/* ----------------------------------------------------------------------------------------------------- */
int8_t SCSI_ProcessCmd(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *cmd);
void SCSI_SenseCode(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t sKey, uint8_t ASC);

extern USBD_ClassTypeDef USBD_COMPOSITE;

#endif
