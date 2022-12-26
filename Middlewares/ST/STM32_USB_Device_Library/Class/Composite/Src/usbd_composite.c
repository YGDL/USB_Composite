/**
  ******************************************************************************
  * @file    usbd_composite.c
  * @author  Sunshine Circuit
  * @brief   该文件提供高层固件功能，以管理USB Composite Class的以下功能:
  *           - 高层和低层的初始化和配置
  *           - 作为COMPOSITE设备的枚举(以及每个实现的内存接口的枚举)
  *           - OUT/IN数据传输
  *           - 命令IN传输(类请求管理)
  *           - 错误管理
  *
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
  *  @verbatim
  *
  *          ===================================================================
  *                                CDC类驱动描述
  *          ===================================================================
  *           此驱动程序管理“通信设备的通用串行总线类定义修订版1.2 2007年11月16日”和
  *           “PSTN 设备的通用串行总线通信类子类规范修订版 1.2 2007 年 2 月 9 日”的
  *           子协议规范 此驱动程序实现规范的以下方面：
  *             - 设备描述符管理
  *             - 配置描述符管理
  *             - 枚举为具有2个数据端点(IN和OUT)和1个命令端点(IN)的CDC设备
  *             - 请求管理(如规范中的6.2节所述)
  *             - 控制模型兼容
  *             - 联合函数集合(使用1个IN端点进行控制)
  *             - 数据接口类
  *
  *           这些方面可以针对特定的用户应用程序进行丰富或修改。
  *
  *            此驱动程序不实现规范的以下方面(但可以通过对此驱动程序进行一些修改来管理这些功能):
  *             - 任何与通信类相关的类特定方面都应该由用户应用程序管理。
  *             - 不管理除PSTN之外的所有通信类
  *          ===================================================================
  *                                MSC类驱动描述
  *          ===================================================================
  *           此模块按照“通用串行总线大容量存储类(MSC)仅批量传输(BOT)版本1.0 1999年9月31日”
  *           管理MSC类 V1.0。
  *           这个驱动程序实现了规范的以下方面:
  *             - 仅批量传输协议
  *             - 子类:SCSI透明命令集(参考:SCSI主命令-3 (SPC-3))
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

 /* Includes ------------------------------------------------------------------*/
#include "stdint.h"

#include "stm32h7xx.h"
#include "usbd_composite.h"
#include "usbd_composite_if.h"

/* ---------------------------------- Composite Funtion Declare ---------------------------------- */

static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef *pDev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t *USBD_COMPOSITE_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_COMPOSITE_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_COMPOSITE_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_COMPOSITE_GetDeviceQualifierDescriptor(uint16_t *length);

/* ------------------------------------- BOT Funtion Declare ------------------------------------- */

static void MSC_BOT_SendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint32_t len);
static void MSC_BOT_CBW_Decode(USBD_HandleTypeDef *pdev);
static void MSC_BOT_Abort(USBD_HandleTypeDef *pdev);

/* ------------------------------------- SCSI Funtion Declare ------------------------------------ */

static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Inquiry(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadCapacity16(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_RequestSense(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_AllowPreventRemovable(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense6(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Write10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Write12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Read10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Read12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Verify10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_CheckAddressRange(USBD_HandleTypeDef *pdev, uint8_t lun, uint32_t blk_offset, uint32_t blk_nbr);
static int8_t SCSI_ProcessRead(USBD_HandleTypeDef *pdev, uint8_t lun);
static int8_t SCSI_ProcessWrite(USBD_HandleTypeDef *pdev, uint8_t lun);
static int8_t SCSI_UpdateBotData(USBD_MSC_BOT_HandleTypeDef *hmsc, uint8_t *pBuff, uint16_t length);

/* ------------------------------------ Composite Descriptor ------------------------------------- */

/* COMPOSITE类实例化 */
USBD_ClassTypeDef USBD_COMPOSITE = 
{
	USBD_COMPOSITE_Init,
	USBD_COMPOSITE_DeInit,
	USBD_COMPOSITE_Setup,
	NULL,		/**< 端点0不做发送使用，但端点0是一个双向的IO */
	USBD_COMPOSITE_EP0_RxReady,	/**< 端点0做接收使用 */
	USBD_COMPOSITE_DataIn,
	USBD_COMPOSITE_DataOut,
	NULL,		/**< SOF 中断不做处理 */
	NULL,		/**< IsoINIncomplete 同步传输发送未完成中断不做处理 */
	NULL,		/**< IsoOUTIncomplete 同步传输接收未完成中断也不做处理 */
	USBD_COMPOSITE_GetHSCfgDesc,		/**< 获取高速USB配置描述符 */
	USBD_COMPOSITE_GetFSCfgDesc,		/**< 获取全速USB配置描述符 */
	USBD_COMPOSITE_GetOtherSpeedCfgDesc,/**< 获取其他速度USB配置描述符 */
	USBD_COMPOSITE_GetDeviceQualifierDescriptor,
};

/* 设备限定描述符 */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
	USB_LEN_DEV_QUALIFIER_DESC,					/**< bLength: 设备限定描述符大小 */
	USB_DESC_TYPE_DEVICE_QUALIFIER,				/**< bDescriptorType: 设备限定描述符 */
	0x00,										/**< bcdUSB: USB规范版本号，BCD码 */
	0x02,										/**< bcdUSB: USB规范版本号，BCD码 */
	0x00,										/**< bDeviceClass: 类 */
	0x00,										/**< bDeviceSubClass: 子类 */
	0x00,										/**< bDeviceProtocol: 协议 */
	0x40,										/**< bMaxPacketSize0: 端点0支持的最大包长 */
	0x01,										/**< bNumConfigurations: 所支持的配置数 */
	0x00,										/**< bReserved: 保留 */
};

/* USB COMPOSITE设备配置描述符 */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_CfgDesc[USB_COM_COMFIG_DESC_SIZ] __ALIGN_END =
{
	/* 配置描述符 */
	0x09,										/* bLength: 配置描述符大小 */
	USB_DESC_TYPE_CONFIGURATION,				/* bDescriptorType: 配置描述符 */
	USB_COM_COMFIG_DESC_SIZ,					/* wTotalLength: 长度 */
	0x00,
	0x03,										/* bNumInterfaces: 3个接口，*/
	0x01,										/* bConfigurationValue: 配置值 */
	0x00,										/* iConfiguration: 描述配置的字符串描述符的索引 */
#if (USBD_SELF_POWERED == 1U)
	0xC0,										/* bmAttributes: 总线根据用户配置上电 */
#else
	0x80,										/* bmAttributes: 总线根据用户配置上电 */
#endif /* USBD_SELF_POWERED */
	USBD_MAX_POWER,								/* MaxPower (mA) */

	/*-------------- Communication Device Class (Virtual Port Com) --------------*/

	/* 组合描述符 */
	0x08,										/* bLength: 组合描述符*/
	USB_DESC_TYPE_IAD,							/* bDescriptorTyppe: 组合描述符*/
	0x00,										/* bFirstInterface: 首个接口编号 */
	0x02,										/* bInterfaceCount: 接口数量 */
	0x02,										/* bFunctionClass: CDC类 */
	0x02,										/* bFunctionSubClass: 抽象控制模型 */
	0x01,										/* bFunctionProtocol: AT常用命令 */
	0x09,										/* iFunction: */

	/* 接口描述符 */
	0x09,										/* bLength: 接口描述符大小 */
	USB_DESC_TYPE_INTERFACE,					/* bDescriptorType: 接口描述符 */
	0x00,										/* bInterfaceNumber: 接口编号 */
	0x00,										/* bAlternateSetting: 备用通道  */
	0x01,										/* bNumEndpoints: 使用的一个端点？？？？？端点0？？？ */
	0x02,										/* bInterfaceClass: CDC类 */
	0x02,										/* bInterfaceSubClass: 抽象控制模型 */
	0x01,										/* bInterfaceProtocol: AT常用命令 */
	0x05,										/* iInterface */

	/* 功能描述符 */
	0x05,										/* bLength: 端点描述符大小 */
	0x24,										/* bDescriptorType: CS_INTERFACE描述符 */
	0x00,										/* bDescriptorSubtype: 功能描述符 */
	0x10,										/* bcdCDC: 规格发布号，即USB通信设备协议的版本号 */
	0x01,

	/* 调用管理功能描述符 */
	0x05,										/* bFunctionLength */
	0x24,										/* bDescriptorType: CS_INTERFACE */
	0x01,										/* bDescriptorSubtype: 调用管理功能描述符 */
	0x00,										/* bmCapabilities: D0+D1 描述设备的能力，只有最低两位D0和D1有意义，其余
													位为保留值0。D0为0，表示设备自己不处理调用管理，为1则表示自己处理。 */
	0x01,										/* bDataInterface 表示选择用来做调用管理的数据类接口编号，由于不使用
													数据类接口做调用管理，因而该字段设置为0 */

	/* 抽象控制管理功能描述符 */
	0x04,										/* bFunctionLength */
	0x24,										/* bDescriptorType: CS_INTERFACE */
	0x02,										/* bDescriptorSubtype: 摘要控制管理描述符 */
	0x02,										/* bmCapabilities 其中D7-4位为保留位，设置为0，支持Set_Line_Coding、
													Set_Control_Line_State、Get_Line_Coding请求和Serial_State通知
													D0表示是否支持以下请求：Set_Comm_Feature、Clear_Comm_Feature、
													Get_Comm_Feature,为1表示支持；D1位表示是否支持Set_Line_Coding、
													Set_Control_Line_State、Get_Line_Coding请求和Serial_State通知，
													为1表示支持；D2为表示是否支持Send_Break，为1表示支持；D3表示是否
													支持Network_Connection通知，为1表示支持 */

	/* 联合函数描述符 */
	0x05,										/* bFunctionLength */
	0x24,										/* bDescriptorType: CS_INTERFACE */
	0x06,										/* bDescriptorSubtype: Union func desc */
	0x00,										/* bMasterInterface: Communication class interface */
	0x01,										/* bSlaveInterface0: 数据类型接口编号 */

	/* 端点2描述符 */
	0x07,										/* bLength: 端点描述符大小 */
	USB_DESC_TYPE_ENDPOINT,						/* bDescriptorType: 端点描述符 */
	COM_CDC_CMD_EP,								/* bEndpointAddress: 端点描述符地址 */
	0x03,										/* bmAttributes: 中断控制传输 */
	LOBYTE(COM_CDC_CMD_PACK_SIZE),				/* wMaxPacketSize: 端点支持的最大包长 */
	HIBYTE(COM_CDC_CMD_PACK_SIZE),
	COM_CDC_FS_BINTERVAL,						/* bInterval: 端点查询时间 */

	/* 数据类接口描述符 */
	0x09,										/* bLength: 接口描述符大小 */
	USB_DESC_TYPE_INTERFACE,					/* bDescriptorType: 接口描述符 */
	0x01,										/* bInterfaceNumber: 接口编号 */
	0x00,										/* bAlternateSetting: 交替设置 */
	0x02,										/* bNumEndpoints: 使用两个端点 */
	0x0A,										/* bInterfaceClass: CDC */
	0x00,										/* bInterfaceSubClass */
	0x00,										/* bInterfaceProtocol */
	0x06,										/* iInterface */

	/* 端点输入描述符 */
	0x07,										/* bLength: 端点描述符大小 */
	USB_DESC_TYPE_ENDPOINT,						/* bDescriptorType: 端点描述符 */
	COM_CDC_IN_EP,								/* bEndpointAddress: 端点1 */
	0x02,										/* bmAttributes: Bulk 批量传输 */
	LOBYTE(COM_CDC_DATA_MAX_PACK_SIZE),			/* wMaxPacketSize: 端点包大小 */
	HIBYTE(COM_CDC_DATA_MAX_PACK_SIZE),
	0x00,										/* bInterval: 端点查询时间 */

	/* 端点输出描述符 */
	0x07,										/* bLength: 端点描述符大小 */
	USB_DESC_TYPE_ENDPOINT,						/* bDescriptorType: 端点描述符 */
	COM_CDC_OUT_EP,								/* bEndpointAddress: 端点1 */
	0x02,										/* bmAttributes: Bulk 批量传输 */
	LOBYTE(COM_CDC_DATA_MAX_PACK_SIZE),			/* wMaxPacketSize: 端点包大小 */
	HIBYTE(COM_CDC_DATA_MAX_PACK_SIZE),
	0x00,										/* bInterval: 端点查询时间 */

	/*--------------------------- Mass Storage Class ----------------------------*/

	/* 关联接口描述符 */
	0x08,										/* bLength: 组合描述符尺寸 */
	USB_DESC_TYPE_IAD,							/* bDescriptorTyppe: 组合描述符*/
	0x02,										/* bFirstInterface: 首个接口编号 */
	0x01,										/* bInterfaceCount: 接口数量 */
	0x08,										/* bFunctionClass: CDC类 */
	0x06,										/* bFunctionSubClass: 抽象控制模型 */
	0x50,										/* bFunctionProtocol: AT常用命令 */
	0x0A,										/* iFunction: */

	/* 接口描述符 */
	0x09,										/* bLength: 接口描述符尺寸 */
	USB_DESC_TYPE_INTERFACE,					/* bDescriptorType: 接口描述符 */
	0x02,										/* bInterfaceNumber: 接口数量 */
	0x00,										/* bAlternateSetting: 交替设置 */
	0x02,										/* bNumEndpoints: 端点数量 */
	0x08,										/* bInterfaceClass: MSC Class */
	0x06,										/* bInterfaceSubClass: SCSI透明传输 */
	0x50,										/* nInterfaceProtocol */
	0x07,										/* iInterface: */

	/* 端点描述符 */
	0x07,										/* bLength: 端点描述符大小 */
	USB_DESC_TYPE_ENDPOINT,						/* bDescriptorType: 端点描述符 */
	COM_MSC_IN_EP,								/* bEndpointAddress: 端点3 */
	0x02,										/* bmAttributes: Bulk 批量传输 */
	LOBYTE(COM_MSC_DATA_MAX_PACK_SIZE),			/* wMaxPacketSize: 端点包大小 */
	HIBYTE(COM_MSC_DATA_MAX_PACK_SIZE),
	0x00,										/* bInterval: 轮询间隔(毫秒) */

	0x07,										/* bLength: 端点描述符大小 */
	USB_DESC_TYPE_ENDPOINT,						/* bDescriptorType: 端点描述符 */
	COM_MSC_OUT_EP,								/* bEndpointAddress: 端点3 */
	0x02,										/* bmAttributes: Bulk 批量传输 */
	LOBYTE(COM_MSC_DATA_MAX_PACK_SIZE),			/* wMaxPacketSize: 端点包大小 */
	HIBYTE(COM_MSC_DATA_MAX_PACK_SIZE),
	0x00,										/* bInterval: 轮询间隔(毫秒) */
};

/* ----------------------------------- Composite Class Funtion ----------------------------------- */

/**
  * @brief  USBD_CDC_MALLOC 申请CDC静态内存
  * @retval 内存地址
  */
static USBD_CDC_HandleTypeDef * USBD_CDC_MALLOC(void)
{
	static USBD_CDC_HandleTypeDef USBD_CDC_Handle;
	return &USBD_CDC_Handle;
}

/**
  * @brief  USBD_MSC_MALLOC 申请MSC静态内存
  * @retval 内存地址
  */
static USBD_MSC_BOT_HandleTypeDef * USBD_MSC_MALLOC(void)
{
	static USBD_MSC_BOT_HandleTypeDef USBD_MSC_Handle;
	return &USBD_MSC_Handle;
}

/**
  * @brief  USBD_CDC_FREE 释放CDC内存
  * @retval 释放结果，若释放完成，则为NULL，否则为原地址
  */
static void * USBD_CDC_FREE(void * p)
{
	return NULL;
}

/**
  * @brief  USBD_MSC_FREE 释放MSC内存
  * @retval 释放结果，若释放完成，则为NULL，否则为原地址
  */
static void * USBD_MSC_FREE(void * p)
{
	return NULL;
}

/**
  * @brief  USBD_COMPOSITE_Init 初始化CDC与MSC接口
  * @note   当设备收到设置配置请求时，会调用此回调；在此函数中类接口使用的端点打开。
  * @param  pdev: 设备的实例
  * @param  cfgidx: 配置指标
  * @retval 状态
  */
 static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
 {
	UNUSED(cfgidx);

	/* 获取类数据句柄 */
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	USBD_MSC_BOT_HandleTypeDef *hmsc = USBD_MSC_MALLOC();

	/* ------------------------------------- CDC Init ------------------------------------- */
	/* 判断申请是否成功，用于动态申请 */
	if(hcdc == NULL)
	{
		pdev->pClassDataCmsit[pdev->classId] = NULL;
		return (uint8_t)USBD_EMEM;
	}
	(void)USBD_memset(hcdc, 0, sizeof(USBD_CDC_HandleTypeDef));
	/* 转换操作句柄与数据句柄 */
	pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
	pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
	pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];
	/* Open EP IN */
	(void)USBD_LL_OpenEP(pdev, COM_CDC_IN_EP, USBD_EP_TYPE_BULK, COM_CDC_DATA_MAX_PACK_SIZE);
	pdev->ep_in[COM_CDC_IN_EP & 0x0FU].is_used = 1U;
	/* Open EP OUT */
	(void)USBD_LL_OpenEP(pdev, COM_CDC_OUT_EP, USBD_EP_TYPE_BULK, COM_CDC_DATA_MAX_PACK_SIZE);
	pdev->ep_out[COM_CDC_OUT_EP & 0x0FU].is_used = 1U;
	/* 为CDC CMD端点设置bInterval */
	pdev->ep_in[COM_CDC_CMD_EP & 0xFU].bInterval = COM_CDC_FS_BINTERVAL;
	/* Open Command IN EP */
	(void)USBD_LL_OpenEP(pdev, COM_CDC_CMD_EP, USBD_EP_TYPE_INTR, COM_CDC_CMD_PACK_SIZE);
	pdev->ep_in[COM_CDC_CMD_EP & 0x0FU].is_used = 1U;

	hcdc->RxBuffer = NULL;

	/* 初始化物理接口 */
	((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();

	/* 初始化Xfer状态 */
	hcdc->TxState = 0U;
	hcdc->RxState = 0U;

	if(hcdc->RxBuffer == NULL)
	{
		return (uint8_t)USBD_EMEM;
	}
	/* 准备Out端点以接收下一个数据包 */
    (void)USBD_LL_PrepareReceive(pdev, COM_CDC_OUT_EP, hcdc->RxBuffer, COM_CDC_DATA_MAX_PACK_SIZE);

	/* ------------------------------------- MSC Init ------------------------------------- */
	/* 判断申请是否成功，用于动态申请 */
	if(hmsc == NULL)
	{
		pdev->pClassDataCmsit[pdev->classId] = NULL;
		return (uint8_t)USBD_EMEM;
	}
	/* 转换操作句柄与数据句柄 */
	pdev->pUserData[pdev->classId] = &USBD_MSC_Interface_fops_FS;
	pdev->pClassDataCmsit[pdev->classId] = (void *)hmsc;
	pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];
	/* Open EP IN*/
	(void)USBD_LL_OpenEP(pdev, COM_MSC_IN_EP, USBD_EP_TYPE_BULK, COM_MSC_DATA_MAX_PACK_SIZE);
	pdev->ep_in[COM_MSC_IN_EP & 0x0FU].is_used = 1U;
	/* Open EP OUT */
	(void)USBD_LL_OpenEP(pdev, COM_MSC_OUT_EP, USBD_EP_TYPE_BULK, COM_MSC_DATA_MAX_PACK_SIZE);
	pdev->ep_out[COM_MSC_OUT_EP & 0x0FU].is_used = 1U;

	/* 初始化BOT层 */
	MSC_BOT_Init(pdev);

	return (uint8_t)USBD_OK;
 }

 /**
  * @brief  USBD_COMPOSITE_DeInit 去初始化COMPOSITE层
  * @note   当收到清除配置请求时，会调用此回调；此函数会关闭类接口使用的端点。
  * @param  pdev: 设备实例
  * @param  cfgidx: 配置指标
  * @retval 状态
  */
static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	UNUSED(cfgidx);

	/* 获取类数据句柄 */
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	USBD_MSC_BOT_HandleTypeDef *hmsc = USBD_MSC_MALLOC();

	/* ------------------------------------ CDC DeInit ------------------------------------ */
	/* 转换操作句柄与数据句柄 */
	pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
	pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
	
	/* Malloc检查 */
	if (pdev->pClassDataCmsit[pdev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	/* Close EP IN */
	(void)USBD_LL_CloseEP(pdev, COM_CDC_IN_EP);
	pdev->ep_in[COM_CDC_IN_EP & 0xFU].is_used = 0U;

	/* Close EP OUT */
	(void)USBD_LL_CloseEP(pdev, COM_CDC_OUT_EP);
	pdev->ep_out[COM_CDC_OUT_EP & 0xFU].is_used = 0U;

	/* Close Command IN EP */
	(void)USBD_LL_CloseEP(pdev, COM_CDC_CMD_EP);
	pdev->ep_in[COM_CDC_CMD_EP & 0xFU].is_used = 0U;
	pdev->ep_in[COM_CDC_CMD_EP & 0xFU].bInterval = 0U;

	/* 去初始化物理接口组件 */
	if (pdev->pClassDataCmsit[pdev->classId] != NULL)
	{
		((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
		(void)USBD_CDC_FREE(pdev->pClassDataCmsit[pdev->classId]);
		pdev->pClassDataCmsit[pdev->classId] = NULL;
		pdev->pClassData = NULL;
	}

	/* ------------------------------------ MSC DeInit ------------------------------------ */
	/* 转换操作句柄与数据句柄 */
	pdev->pUserData[pdev->classId] = &USBD_MSC_Interface_fops_FS;
	pdev->pClassDataCmsit[pdev->classId] = (void *)hmsc;

	/* Malloc检查 */
	if (pdev->pClassDataCmsit[pdev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	/* Close EP IN */
	(void)USBD_LL_CloseEP(pdev, COM_MSC_IN_EP);
	pdev->ep_out[COM_MSC_IN_EP & 0xFU].is_used = 0U;

	/* Close EP OUT */
	(void)USBD_LL_CloseEP(pdev, COM_MSC_OUT_EP);
	pdev->ep_in[COM_MSC_OUT_EP & 0xFU].is_used = 0U;

	/* 释放MSC类资源 */
	if (pdev->pClassDataCmsit[pdev->classId] != NULL)
	{
		/* 去初始化BOT层 */
		MSC_BOT_DeInit(pdev);
		(void)USBD_MSC_FREE(pdev->pClassDataCmsit[pdev->classId]);
		pdev->pClassDataCmsit[pdev->classId]  = NULL;
		pdev->pClassData = NULL;
	}
	
	return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_Setup 处理COMPOSITE特定的请求
  * @note   调用此回调可处理特定类设置请求。
  * @param  pdev: 设备实例
  * @param  req: USB请求
  * @retval 状态
  */
static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	/* 获取类数据句柄 */
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	USBD_MSC_BOT_HandleTypeDef *hmsc = USBD_MSC_MALLOC();
	
	uint16_t len;
	uint8_t ifalt = 0U;
	uint16_t status_info = 0U;
	USBD_StatusTypeDef ret = USBD_OK;
	/* wIndex是接口编号 */
	switch(req->wIndex)
	{
		/* 第一个IAD组 */
		case 0:
		case 1:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
			/* Malloc检查 */
			if (hmsc == NULL)
				return (uint8_t)USBD_FAIL;

			switch (req->bmRequest & USB_REQ_TYPE_MASK)
			{
				case USB_REQ_TYPE_CLASS:
					if (req->wLength != 0U)
					{
						if ((req->bmRequest & 0x80U) != 0U)
						{
							((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest, (uint8_t *)hcdc->data, req->wLength);

							len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
							(void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, len);
						}
						else
						{
							hcdc->CmdOpCode = req->bRequest;
							hcdc->CmdLength = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);

							(void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, hcdc->CmdLength);
						}
					}
					else
					{
						((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
																						(uint8_t *)req, 0U);
					}
					break;

				case USB_REQ_TYPE_STANDARD:
					switch (req->bRequest)
					{
						case USB_REQ_GET_STATUS:
							if (pdev->dev_state == USBD_STATE_CONFIGURED)
								(void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
							else
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						case USB_REQ_GET_INTERFACE:
							if (pdev->dev_state == USBD_STATE_CONFIGURED)
							{
								(void)USBD_CtlSendData(pdev, &ifalt, 1U);
							}
							else
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						case USB_REQ_SET_INTERFACE:
							if (pdev->dev_state != USBD_STATE_CONFIGURED)
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						case USB_REQ_CLEAR_FEATURE:
							break;

						default:
							USBD_CtlError(pdev, req);
							ret = USBD_FAIL;
							break;
					}
					break;

				default:
					USBD_CtlError(pdev, req);
					ret = USBD_FAIL;
					break;
			}

			return (uint8_t)ret;
		/* 第二个IAD组 */
		case 2:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_MSC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hmsc;
			/* Malloc检查 */
			if (hmsc == NULL)
				return (uint8_t)USBD_FAIL;

			switch (req->bmRequest & USB_REQ_TYPE_MASK)
			{
				/* Class request */
				case USB_REQ_TYPE_CLASS:
					switch (req->bRequest)
					{
						case BOT_GET_MAX_LUN:
							if ((req->wValue  == 0U) && (req->wLength == 1U) && ((req->bmRequest & 0x80U) == 0x80U))
							{
								hmsc->max_lun = (uint32_t)((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->GetMaxLun();
								(void)USBD_CtlSendData(pdev, (uint8_t *)&hmsc->max_lun, 1U);
							}
							else
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						case BOT_RESET :
							if ((req->wValue  == 0U) && (req->wLength == 0U) && ((req->bmRequest & 0x80U) != 0x80U))
								MSC_BOT_Reset(pdev);
							else
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						default:
							USBD_CtlError(pdev, req);
							ret = USBD_FAIL;
							break;
					}
					break;
				/* Interface & Endpoint request */
				case USB_REQ_TYPE_STANDARD:
					switch (req->bRequest)
					{
						case USB_REQ_GET_STATUS:
							if (pdev->dev_state == USBD_STATE_CONFIGURED)
								(void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
							else
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						case USB_REQ_GET_INTERFACE:
							if (pdev->dev_state == USBD_STATE_CONFIGURED)
								(void)USBD_CtlSendData(pdev, (uint8_t *)&hmsc->interface, 1U);
							else
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						case USB_REQ_SET_INTERFACE:
							if (pdev->dev_state == USBD_STATE_CONFIGURED)
								hmsc->interface = (uint8_t)(req->wValue);
							else
							{
								USBD_CtlError(pdev, req);
								ret = USBD_FAIL;
							}
							break;

						case USB_REQ_CLEAR_FEATURE:
							if (pdev->dev_state == USBD_STATE_CONFIGURED)
							{
								if (req->wValue == USB_FEATURE_EP_HALT)
								{
									/* Flush the FIFO */
									(void)USBD_LL_FlushEP(pdev, (uint8_t)req->wIndex);

									/* Handle BOT error */
									MSC_BOT_CplClrFeature(pdev, (uint8_t)req->wIndex);
								}
							}
							break;

						default:
							USBD_CtlError(pdev, req);
							ret = USBD_FAIL;
							break;
					}
					break;

				default:
					USBD_CtlError(pdev, req);
					ret = USBD_FAIL;
					break;
			}

			return (uint8_t)ret;
		default:
			return USBD_FAIL;
	}
}

/**
  * @brief  USBD_COMPOSITE_DataIn 非控制输入端点发送的数据
  * @param  pdev: 设备实例
  * @param  epnum: 端点号
  * @retval 状态
  */
static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	/* 获取类数据句柄 */
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	USBD_MSC_BOT_HandleTypeDef *hmsc = USBD_MSC_MALLOC();
	
	switch(epnum | 0x80)
	{
		case COM_CDC_IN_EP:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
			
			PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
			/* Malloc检查 */
			if (pdev->pClassDataCmsit[pdev->classId] == NULL)
				return (uint8_t)USBD_FAIL;

			if ((pdev->ep_in[epnum & 0xFU].total_length > 0U) && ((pdev->ep_in[epnum & 0xFU].total_length % hpcd->IN_ep[epnum & 0xFU].maxpacket) == 0U))
			{
				/* 更新数据包总长度 */
				pdev->ep_in[epnum & 0xFU].total_length = 0U;

				/* 发送ZLP */
				(void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
			}
			else
			{
				hcdc->TxState = 0U;

				if (((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt != NULL)
					((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt(hcdc->TxBuffer, &hcdc->TxLength, epnum);
			}

			break;

		case COM_MSC_IN_EP:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_MSC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hmsc;
			
			/* Malloc检查 */
			if (pdev->pClassDataCmsit[pdev->classId] == NULL)
				return (uint8_t)USBD_FAIL;

			MSC_BOT_DataIn(pdev, epnum);
			break;
	}

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_DataOut 非控制输出端点接收的数据
  * @param  pdev: 设备实例
  * @param  epnum: 端点号
  * @retval 状态
  */
static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	/* 获取类数据句柄 */
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	USBD_MSC_BOT_HandleTypeDef *hmsc = USBD_MSC_MALLOC();
	
	switch(epnum)
	{
		case COM_CDC_OUT_EP:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;

			/* Malloc检查 */
			if (pdev->pClassDataCmsit[pdev->classId] == NULL)
				return (uint8_t)USBD_FAIL;

			/* 获取接收的数据长度 */
			hcdc->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

			/* USB数据将被立即处理，这允许下一个USB流量被裸直到应用程序Xfer结束 */
			((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hcdc->RxBuffer, &hcdc->RxLength);

			break;

		case COM_MSC_OUT_EP:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_MSC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hmsc;

			/* Malloc检查 */
			if (pdev->pClassDataCmsit[pdev->classId] == NULL)
				return (uint8_t)USBD_FAIL;
			
			MSC_BOT_DataOut(pdev, epnum);
			break;
	}

	return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_EP0_RxReady 处理EP0 Rx读事件
  * @param  pdev: 设备的实例
  * @retval 状态
  */
static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
	/* 获取类数据句柄 */
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	USBD_MSC_BOT_HandleTypeDef *hmsc = USBD_MSC_MALLOC();

	switch(pdev->request.wIndex)
	{
		case 0:
		case 1:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
			/* Malloc检查 */
			if(pdev->pClassDataCmsit[pdev->classId] == NULL)
				return (uint8_t)USBD_FAIL;

			if((pdev->pUserData[pdev->classId] != NULL) && (hcdc->CmdOpCode != 0xFFU))
			{
				((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(hcdc->CmdOpCode, (uint8_t *)hcdc->data, (uint16_t)hcdc->CmdLength);
				hcdc->CmdOpCode = 0xFFU;
			}
			break;

		case 2:
			/* 转换操作句柄与数据句柄 */
			pdev->pUserData[pdev->classId] = &USBD_MSC_Interface_fops_FS;
			pdev->pClassDataCmsit[pdev->classId] = (void *)hmsc;
			/* Malloc检查 */
			if (pdev->pClassDataCmsit[pdev->classId] == NULL)
				return (uint8_t)USBD_FAIL;
			break;
	}

	return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_GetFSCfgDesc 返回配置描述符
  * @param  length : 指针数据长度
  * @retval 描述符缓冲区的指针
  */
static uint8_t *USBD_COMPOSITE_GetFSCfgDesc(uint16_t *length)
{
	*length = (uint16_t)sizeof(USBD_COMPOSITE_CfgDesc);
	return USBD_COMPOSITE_CfgDesc;
}

/**
  * @brief  USBD_COMPOSITE_GetHSCfgDesc 返回配置描述符
  * @param  length : 指针数据长度
  * @retval 描述符缓冲区的指针
  */
static uint8_t *USBD_COMPOSITE_GetHSCfgDesc(uint16_t *length)
{
	*length = (uint16_t)sizeof(USBD_COMPOSITE_CfgDesc);
	return USBD_COMPOSITE_CfgDesc;
}

/**
  * @brief  USBD_COMPOSITE_GetOtherSpeedCfgDesc 返回配置描述符
  * @param  length : 指针数据长度
  * @retval 描述符缓冲区的指针
  */
static uint8_t *USBD_COMPOSITE_GetOtherSpeedCfgDesc(uint16_t *length)
{
	*length = (uint16_t)sizeof(USBD_COMPOSITE_CfgDesc);
	return USBD_COMPOSITE_CfgDesc;
}

/**
  * @brief  USBD_COMPOSITE_GetDeviceQualifierDescriptor 返回设备限定符
  * @param  length : 指针数据长度
  * @retval 描述符缓冲区的指针
  */
uint8_t *USBD_COMPOSITE_GetDeviceQualifierDescriptor(uint16_t *length)
{
	*length = (uint16_t)sizeof(USBD_COMPOSITE_DeviceQualifierDesc);
	return USBD_COMPOSITE_DeviceQualifierDesc;
}

/**
  * @brief  USBD_CDC_RegisterInterface CDC类接口注册操作
  * @param  pdev: 设备实例
  * @param  fops: CD接口回调
  * @retval 状态
  */
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_CDC_ItfTypeDef *fops)
{
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	pdev->pClassDataCmsit[pdev->classId] = hcdc;

	if(fops == NULL)
		return (uint8_t)USBD_FAIL;

	pdev->pUserData[pdev->classId] = fops;

	return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MSC_RegisterInterface MSC类接口注册操作
  * @param  pdev: 设备实例
  * @param  fops: MS接口回调
  * @retval 状态
  */
uint8_t USBD_MSC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_StorageTypeDef *fops)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = USBD_MSC_MALLOC();
	pdev->pClassDataCmsit[pdev->classId] = (void *)hmsc;

	if(fops == NULL)
		return (uint8_t)USBD_FAIL;

	pdev->pUserData[pdev->classId] = fops;

	return (uint8_t)USBD_OK;
}

/* -------------------------------------- CDC Class Funtion -------------------------------------- */

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: 设备实例
  * @param  pbuff: Tx缓冲
  * @param  length: Tx缓冲长度
  * @retval 状态
  */
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint32_t length)
{
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	/* Malloc检查 */
	if (hcdc == NULL)
		return (uint8_t)USBD_FAIL;

	hcdc->TxBuffer = pbuff;
	hcdc->TxLength = length;

	return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: 设备实例
  * @param  pbuff: Rx缓冲
  * @retval 状态
  */
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	/* Malloc检查 */
	if (hcdc == NULL)
		return (uint8_t)USBD_FAIL;

	hcdc->RxBuffer = pbuff;

	return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_TransmitPacket 输入端点发送数据包
  * @param  pdev: 设备实例
  * @retval 状态
  */
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
	USBD_StatusTypeDef ret = USBD_BUSY;
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	
	pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
	pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
	
	if (pdev->pClassDataCmsit[pdev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	if (hcdc->TxState == 0U)
	{
		/* Tx传输正在进行中 */
		hcdc->TxState = 1U;

		/* 更新数据包总长度 */
		pdev->ep_in[COM_CDC_IN_EP & 0xFU].total_length = hcdc->TxLength;

		/* 发送下一个数据包 */
		(void)USBD_LL_Transmit(pdev, COM_CDC_IN_EP, hcdc->TxBuffer, hcdc->TxLength);

		ret = USBD_OK;
	}

	return (uint8_t)ret;
}

/**
  * @brief  USBD_CDC_ReceivePacket 输出端点接收数据包
  * @param  pdev: 设备实例
  * @retval 状态
  */
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
	USBD_CDC_HandleTypeDef *hcdc = USBD_CDC_MALLOC();
	
	pdev->pUserData[pdev->classId] = &USBD_CDC_Interface_fops_FS;
	pdev->pClassData = (void *)hcdc;
	/* Malloc检查 */
	if(pdev->pClassDataCmsit[pdev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	if (pdev->dev_speed == USBD_SPEED_HIGH)
		/* 准备Out端点以接收下一个数据包 */
		(void)USBD_LL_PrepareReceive(pdev, COM_CDC_OUT_EP, hcdc->RxBuffer, COM_CDC_DATA_MAX_PACK_SIZE);
	else
		/* 准备Out端点以接收下一个数据包 */
		(void)USBD_LL_PrepareReceive(pdev, COM_CDC_OUT_EP, hcdc->RxBuffer, COM_CDC_DATA_MAX_PACK_SIZE);

	return (uint8_t)USBD_OK;
}

/* -------------------------------------- MSC Class Funtion -------------------------------------- */

/* --------------------------------------- MSC BOT Funtion -------------------------------------- */

/**
  * @brief  MSC_BOT_Init 初始化BOT流程 
  * @param  pdev: device instance
  * @retval None
  */
void MSC_BOT_Init(USBD_HandleTypeDef *pdev)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	hmsc->bot_state = USBD_BOT_IDLE;
	hmsc->bot_status = USBD_BOT_STATUS_NORMAL;

	hmsc->scsi_sense_tail = 0U;
	hmsc->scsi_sense_head = 0U;
	hmsc->scsi_medium_state = SCSI_MEDIUM_UNLOCKED;

	((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->Init(0U);

	(void)USBD_LL_FlushEP(pdev, COM_MSC_OUT_EP);
	(void)USBD_LL_FlushEP(pdev, COM_MSC_IN_EP);

	/* 准备EP接收第一个BOT命令 */
	(void)USBD_LL_PrepareReceive(pdev, COM_MSC_OUT_EP, (uint8_t *)&hmsc->cbw, USBD_BOT_CBW_LENGTH);
}

/**
  * @brief  MSC_BOT_Reset 重置BOT机器
  * @param  pdev: device instance
  * @retval  None
  */
void MSC_BOT_Reset(USBD_HandleTypeDef *pdev)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	hmsc->bot_state  = USBD_BOT_IDLE;
	hmsc->bot_status = USBD_BOT_STATUS_RECOVERY;

	(void)USBD_LL_ClearStallEP(pdev, COM_MSC_IN_EP);
	(void)USBD_LL_ClearStallEP(pdev, COM_MSC_OUT_EP);

	/* 准备EP接收第一个BOT命令 */
	(void)USBD_LL_PrepareReceive(pdev, COM_MSC_OUT_EP, (uint8_t *)&hmsc->cbw, USBD_BOT_CBW_LENGTH);
}

/**
  * @brief  MSC_BOT_DeInit 去初始化BOT机器
  * @param  pdev: device instance
  * @retval None
  */
void MSC_BOT_DeInit(USBD_HandleTypeDef  *pdev)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc != NULL)
		hmsc->bot_state = USBD_BOT_IDLE;
}

/**
  * @brief  MSC_BOT_DataIn 处理BOT输入数据阶段
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval None
  */
void MSC_BOT_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	UNUSED(epnum);

	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	switch (hmsc->bot_state)
	{
		case USBD_BOT_DATA_IN:
			if (SCSI_ProcessCmd(pdev, hmsc->cbw.bLUN, &hmsc->cbw.CB[0]) < 0)
				MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_FAILED);
			break;

		case USBD_BOT_SEND_DATA:
		case USBD_BOT_LAST_DATA_IN:
			MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_PASSED);
			break;

		default:
			break;
	}
}
/**
  * @brief  MSC_BOT_DataOut 处理BOT输出数据阶段
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval None
  */
void MSC_BOT_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	UNUSED(epnum);

	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	switch (hmsc->bot_state)
	{
		case USBD_BOT_IDLE:
			MSC_BOT_CBW_Decode(pdev);
			break;

		case USBD_BOT_DATA_OUT:
			if (SCSI_ProcessCmd(pdev, hmsc->cbw.bLUN, &hmsc->cbw.CB[0]) < 0)
				MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_FAILED);
			break;

		default:
			break;
	}
}

/**
  * @brief  MSC_BOT_CBW_Decode 解码CBW命令并相应地设置BOT状态机
  * @param  pdev: device instance
  * @retval None
  */
static void  MSC_BOT_CBW_Decode(USBD_HandleTypeDef *pdev)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	hmsc->csw.dTag = hmsc->cbw.dTag;
	hmsc->csw.dDataResidue = hmsc->cbw.dDataLength;

	if ((USBD_LL_GetRxDataSize(pdev, COM_MSC_OUT_EP) != USBD_BOT_CBW_LENGTH) || (hmsc->cbw.dSignature != USBD_BOT_CBW_SIGNATURE) ||
		(hmsc->cbw.bLUN > 1U) || (hmsc->cbw.bCBLength < 1U) || (hmsc->cbw.bCBLength > 16U))
	{
		SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);

		hmsc->bot_status = USBD_BOT_STATUS_ERROR;
		MSC_BOT_Abort(pdev);
	}
	else
	{
		if (SCSI_ProcessCmd(pdev, hmsc->cbw.bLUN, &hmsc->cbw.CB[0]) < 0)
		{
			if (hmsc->bot_state == USBD_BOT_NO_DATA)
				MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_FAILED);
			else
				MSC_BOT_Abort(pdev);
		}
		/* 突发Xfer内部处理 */
		else
		{
			if ((hmsc->bot_state != USBD_BOT_DATA_IN) && (hmsc->bot_state != USBD_BOT_DATA_OUT) &&
				(hmsc->bot_state != USBD_BOT_LAST_DATA_IN))
			{
				if (hmsc->bot_data_length > 0U)
					MSC_BOT_SendData(pdev, hmsc->bot_data, hmsc->bot_data_length);
				else
				{
					if (hmsc->bot_data_length == 0U)
						MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_PASSED);
					else
						MSC_BOT_Abort(pdev);
				}
			}
			else
				return;
		}
	}
}

/**
  * @brief  MSC_BOT_SendData 发送请求的数据
  * @param  pdev: device instance
  * @param  buf: pointer to data buffer
  * @param  len: Data Length
  * @retval None
  */
static void  MSC_BOT_SendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint32_t len)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	uint32_t length;

	if (hmsc == NULL)
	{
		return;
	}

	length = MIN(hmsc->cbw.dDataLength, len);

	hmsc->csw.dDataResidue -= len;
	hmsc->csw.bStatus = USBD_CSW_CMD_PASSED;
	hmsc->bot_state = USBD_BOT_SEND_DATA;

	(void)USBD_LL_Transmit(pdev, COM_MSC_IN_EP, pbuf, length);
}

/**
  * @brief  MSC_BOT_SendCSW 发送命令状态包装器
  * @param  pdev: device instance
  * @param  status : CSW status
  * @retval None
  */
void  MSC_BOT_SendCSW(USBD_HandleTypeDef *pdev, uint8_t CSW_Status)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	hmsc->csw.dSignature = USBD_BOT_CSW_SIGNATURE;
	hmsc->csw.bStatus = CSW_Status;
	hmsc->bot_state = USBD_BOT_IDLE;

	(void)USBD_LL_Transmit(pdev, COM_MSC_IN_EP, (uint8_t *)&hmsc->csw, USBD_BOT_CSW_LENGTH);

	/* 准备EP接收下一个命令 */
	(void)USBD_LL_PrepareReceive(pdev, COM_MSC_OUT_EP, (uint8_t *)&hmsc->cbw, USBD_BOT_CBW_LENGTH);
}

/**
  * @brief  MSC_BOT_Abort 终止当前传输
  * @param  pdev: device instance
  * @retval status
  */
static void  MSC_BOT_Abort(USBD_HandleTypeDef *pdev)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	if ((hmsc->cbw.bmFlags == 0U) && (hmsc->cbw.dDataLength != 0U) && (hmsc->bot_status == USBD_BOT_STATUS_NORMAL))
		(void)USBD_LL_StallEP(pdev, COM_MSC_OUT_EP);

	(void)USBD_LL_StallEP(pdev, COM_MSC_IN_EP);

	if (hmsc->bot_status == USBD_BOT_STATUS_ERROR)
	{
		(void)USBD_LL_StallEP(pdev, COM_MSC_IN_EP);
		(void)USBD_LL_StallEP(pdev, COM_MSC_OUT_EP);
	}
}

/**
  * @brief  MSC_BOT_CplClrFeature 完成清晰的特性请求
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval None
  */

void  MSC_BOT_CplClrFeature(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;
	/* 不良CBW签名 */
	if (hmsc->bot_status == USBD_BOT_STATUS_ERROR) 
	{
		(void)USBD_LL_StallEP(pdev, COM_MSC_IN_EP);
		(void)USBD_LL_StallEP(pdev, COM_MSC_OUT_EP);
	}
	else
	{
		if (((epnum & 0x80U) == 0x80U) && (hmsc->bot_status != USBD_BOT_STATUS_RECOVERY))
			MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_FAILED);
		else
			return;
	}
}

/* --------------------------------------- MSC BOT Funtion -------------------------------------- */

/* USB Mass storage Page 0 Inquiry Data */
uint8_t MSC_Page00_Inquiry_Data[LENGTH_INQUIRY_PAGE00] =
{
  0x00,
  0x00,
  0x00,
  (LENGTH_INQUIRY_PAGE00 - 4U),
  0x00,
  0x80
};

/* USB Mass storage VPD Page 0x80 Inquiry Data for Unit Serial Number */
uint8_t MSC_Page80_Inquiry_Data[LENGTH_INQUIRY_PAGE80] =
{
  0x00,
  0x80,
  0x00,
  LENGTH_INQUIRY_PAGE80,
  0x20,     /* Put Product Serial number */
  0x20,
  0x20,
  0x20
};

/* USB Mass storage sense 6 Data */
uint8_t MSC_Mode_Sense6_data[MODE_SENSE6_LEN] =
{
  0x22,
  0x00,
  0x00,
  0x00,
  0x08,
  0x12,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};


/* USB Mass storage sense 10  Data */
uint8_t MSC_Mode_Sense10_data[MODE_SENSE10_LEN] =
{
  0x00,
  0x26,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x08,
  0x12,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};

/* -------------------------------------- MSC SCSI Funtion -------------------------------------- */
/**
  * @brief  SCSI_ProcessCmd 进程SCSI命令
  * @param  pdev: device instance
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
int8_t SCSI_ProcessCmd(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *cmd)
{
	int8_t ret;
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	switch (cmd[0])
	{
		case SCSI_TEST_UNIT_READY:
			ret = SCSI_TestUnitReady(pdev, lun, cmd);
			break;

		case SCSI_REQUEST_SENSE:
			ret = SCSI_RequestSense(pdev, lun, cmd);
			break;

		case SCSI_INQUIRY:
			ret = SCSI_Inquiry(pdev, lun, cmd);
			break;

		case SCSI_START_STOP_UNIT:
			ret = SCSI_StartStopUnit(pdev, lun, cmd);
			break;

		case SCSI_ALLOW_MEDIUM_REMOVAL:
			ret = SCSI_AllowPreventRemovable(pdev, lun, cmd);
			break;

		case SCSI_MODE_SENSE6:
			ret = SCSI_ModeSense6(pdev, lun, cmd);
			break;

		case SCSI_MODE_SENSE10:
			ret = SCSI_ModeSense10(pdev, lun, cmd);
			break;

		case SCSI_READ_FORMAT_CAPACITIES:
			ret = SCSI_ReadFormatCapacity(pdev, lun, cmd);
			break;

		case SCSI_READ_CAPACITY10:
			ret = SCSI_ReadCapacity10(pdev, lun, cmd);
			break;

		case SCSI_READ_CAPACITY16:
			ret = SCSI_ReadCapacity16(pdev, lun, cmd);
			break;

		case SCSI_READ10:
			ret = SCSI_Read10(pdev, lun, cmd);
			break;

		case SCSI_READ12:
			ret = SCSI_Read12(pdev, lun, cmd);
			break;

		case SCSI_WRITE10:
			ret = SCSI_Write10(pdev, lun, cmd);
			break;

		case SCSI_WRITE12:
			ret = SCSI_Write12(pdev, lun, cmd);
			break;

		case SCSI_VERIFY10:
			ret = SCSI_Verify10(pdev, lun, cmd);
			break;

		default:
			SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_CDB);
			hmsc->bot_status = USBD_BOT_STATUS_ERROR;
			ret = -1;
			break;
	}

	return ret;
}


/**
  * @brief  SCSI_TestUnitReady 进程SCSI测试单元准备命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(params);
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	/* case 9 : Hi > D0 */
	if (hmsc->cbw.dDataLength != 0U)
	{
		SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);

		return -1;
	}

	if (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED)
	{
		SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
		hmsc->bot_state = USBD_BOT_NO_DATA;
		return -1;
	}

	if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->IsReady(lun) != 0)
	{
		SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
		hmsc->bot_state = USBD_BOT_NO_DATA;

		return -1;
	}
	hmsc->bot_data_length = 0U;

	return 0;
}


/**
  * @brief  SCSI_Inquiry 流程查询命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_Inquiry(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	uint8_t *pPage;
	uint16_t len;
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if (hmsc->cbw.dDataLength == 0U)
	{
		SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
		return -1;
	}

	if ((params[1] & 0x01U) != 0U) /* Evpd is set */
	{
		if (params[2] == 0U) /* Request for Supported Vital Product Data Pages*/
			(void)SCSI_UpdateBotData(hmsc, MSC_Page00_Inquiry_Data, LENGTH_INQUIRY_PAGE00);
		else
		{
			if (params[2] == 0x80U) /* Request for VPD page 0x80 Unit Serial Number */
				(void)SCSI_UpdateBotData(hmsc, MSC_Page80_Inquiry_Data, LENGTH_INQUIRY_PAGE80);
			else /* Request Not supported */
			{
				SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);

				return -1;
			}
		}
	}
	else
	{
		pPage = (uint8_t *) & ((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->pInquiry[lun * STANDARD_INQUIRY_DATA_LEN];
		len = (uint16_t)pPage[4] + 5U;

		if (params[4] <= len)
			len = params[4];

		(void)SCSI_UpdateBotData(hmsc, pPage, len);
	}

	return 0;
}


/**
  * @brief  SCSI_ReadCapacity10 进程读容量10命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(params);
	int8_t ret;
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	ret = ((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->GetCapacity(lun, &hmsc->scsi_blk_nbr, &hmsc->scsi_blk_size);

	if ((ret != 0) || (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED))
	{
		SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
		return -1;
	}

	hmsc->bot_data[0] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 24);
	hmsc->bot_data[1] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 16);
	hmsc->bot_data[2] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 8);
	hmsc->bot_data[3] = (uint8_t)(hmsc->scsi_blk_nbr - 1U);

	hmsc->bot_data[4] = (uint8_t)(hmsc->scsi_blk_size >> 24);
	hmsc->bot_data[5] = (uint8_t)(hmsc->scsi_blk_size >> 16);
	hmsc->bot_data[6] = (uint8_t)(hmsc->scsi_blk_size >> 8);
	hmsc->bot_data[7] = (uint8_t)(hmsc->scsi_blk_size);

	hmsc->bot_data_length = 8U;

	return 0;
}


/**
  * @brief  SCSI_ReadCapacity16 进程读容量16命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_ReadCapacity16(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(params);
	uint8_t idx;
	int8_t ret;
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	ret = ((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->GetCapacity(lun, &hmsc->scsi_blk_nbr, &hmsc->scsi_blk_size);

	if ((ret != 0) || (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED))
	{
		SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
		return -1;
	}

	hmsc->bot_data_length = ((uint32_t)params[10] << 24) | ((uint32_t)params[11] << 16) |
							((uint32_t)params[12] <<  8) | (uint32_t)params[13];

	for (idx = 0U; idx < hmsc->bot_data_length; idx++)
		hmsc->bot_data[idx] = 0U;

	hmsc->bot_data[4] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 24);
	hmsc->bot_data[5] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 16);
	hmsc->bot_data[6] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 8);
	hmsc->bot_data[7] = (uint8_t)(hmsc->scsi_blk_nbr - 1U);

	hmsc->bot_data[8] = (uint8_t)(hmsc->scsi_blk_size >> 24);
	hmsc->bot_data[9] = (uint8_t)(hmsc->scsi_blk_size >> 16);
	hmsc->bot_data[10] = (uint8_t)(hmsc->scsi_blk_size >> 8);
	hmsc->bot_data[11] = (uint8_t)(hmsc->scsi_blk_size);

	hmsc->bot_data_length = ((uint32_t)params[10] << 24) | ((uint32_t)params[11] << 16) |
							((uint32_t)params[12] <<  8) | (uint32_t)params[13];

	return 0;
}


/**
  * @brief  SCSI_ReadFormatCapacity 进程读取格式容量命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(params);
	uint16_t blk_size;
	uint32_t blk_nbr;
	uint16_t i;
	int8_t ret;
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	ret = ((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->GetCapacity(lun, &blk_nbr, &blk_size);

	if ((ret != 0) || (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED))
	{
		SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
		return -1;
	}

	for (i = 0U; i < 12U ; i++)
		hmsc->bot_data[i] = 0U;


	hmsc->bot_data[3] = 0x08U;
	hmsc->bot_data[4] = (uint8_t)((blk_nbr - 1U) >> 24);
	hmsc->bot_data[5] = (uint8_t)((blk_nbr - 1U) >> 16);
	hmsc->bot_data[6] = (uint8_t)((blk_nbr - 1U) >> 8);
	hmsc->bot_data[7] = (uint8_t)(blk_nbr - 1U);

	hmsc->bot_data[8] = 0x02U;
	hmsc->bot_data[9] = (uint8_t)(blk_size >> 16);
	hmsc->bot_data[10] = (uint8_t)(blk_size >> 8);
	hmsc->bot_data[11] = (uint8_t)(blk_size);

	hmsc->bot_data_length = 12U;

	return 0;
}


/**
  * @brief  SCSI_ModeSense6 进程模式Sense6命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_ModeSense6(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(lun);
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	uint16_t len = MODE_SENSE6_LEN;

	if (hmsc == NULL)
		return -1;

	if (params[4] <= len)
		len = params[4];

	(void)SCSI_UpdateBotData(hmsc, MSC_Mode_Sense6_data, len);

	return 0;
}


/**
  * @brief  SCSI_ModeSense10 进程模式Sense10命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_ModeSense10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(lun);
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	uint16_t len = MODE_SENSE10_LEN;

	if (hmsc == NULL)
		return -1;

	if (params[8] <= len)
		len = params[8];

	(void)SCSI_UpdateBotData(hmsc, MSC_Mode_Sense10_data, len);

	return 0;
}


/**
  * @brief  SCSI_RequestSense 进程请求检测命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_RequestSense(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(lun);
	uint8_t i;
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if (hmsc->cbw.dDataLength == 0U)
	{
		SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
		return -1;
	}

	for (i = 0U; i < REQUEST_SENSE_DATA_LEN; i++)
		hmsc->bot_data[i] = 0U;

	hmsc->bot_data[0] = 0x70U;
	hmsc->bot_data[7] = REQUEST_SENSE_DATA_LEN - 6U;

	if ((hmsc->scsi_sense_head != hmsc->scsi_sense_tail))
	{
		hmsc->bot_data[2] = (uint8_t)hmsc->scsi_sense[hmsc->scsi_sense_head].Skey;
		hmsc->bot_data[12] = (uint8_t)hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASC;
		hmsc->bot_data[13] = (uint8_t)hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASCQ;
		hmsc->scsi_sense_head++;

		if (hmsc->scsi_sense_head == SENSE_LIST_DEEPTH)
			hmsc->scsi_sense_head = 0U;
	}

	hmsc->bot_data_length = REQUEST_SENSE_DATA_LEN;

	if (params[4] <= REQUEST_SENSE_DATA_LEN)
		hmsc->bot_data_length = params[4];

	return 0;
}

/**
  * @brief  SCSI_SenseCode 加载错误列表中的最后一个错误代码
  * @param  lun: Logical unit number
  * @param  sKey: Sense Key
  * @param  ASC: Additional Sense Code
  * @retval none
  */
void SCSI_SenseCode(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t sKey, uint8_t ASC)
{
	UNUSED(lun);
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return;

	hmsc->scsi_sense[hmsc->scsi_sense_tail].Skey = sKey;
	hmsc->scsi_sense[hmsc->scsi_sense_tail].w.b.ASC = ASC;
	hmsc->scsi_sense[hmsc->scsi_sense_tail].w.b.ASCQ = 0U;
	hmsc->scsi_sense_tail++;

	if (hmsc->scsi_sense_tail == SENSE_LIST_DEEPTH)
		hmsc->scsi_sense_tail = 0U;
}


/**
  * @brief  SCSI_StartStopUnit 进程启动停止单元命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(lun);
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if ((hmsc->scsi_medium_state == SCSI_MEDIUM_LOCKED) && ((params[4] & 0x3U) == 2U))
	{
		SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);

		return -1;
	}

	if ((params[4] & 0x3U) == 0x1U) /* START=1 */
		hmsc->scsi_medium_state = SCSI_MEDIUM_UNLOCKED;
	else
	{
		if ((params[4] & 0x3U) == 0x2U) /* START=0 and LOEJ Load Eject=1 */
			hmsc->scsi_medium_state = SCSI_MEDIUM_EJECTED;
		else
		{
			if ((params[4] & 0x3U) == 0x3U) /* START=1 and LOEJ Load Eject=1 */
				hmsc->scsi_medium_state = SCSI_MEDIUM_UNLOCKED;
			else
				;/* .. */
		}
	}
	hmsc->bot_data_length = 0U;

	return 0;
}


/**
  * @brief  SCSI_AllowPreventRemovable 进程允许阻止可移动介质命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_AllowPreventRemovable(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	UNUSED(lun);
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if (params[4] == 0U)
		hmsc->scsi_medium_state = SCSI_MEDIUM_UNLOCKED;
	else
		hmsc->scsi_medium_state = SCSI_MEDIUM_LOCKED;

	hmsc->bot_data_length = 0U;

	return 0;
}


/**
  * @brief  SCSI_Read10 进程Read10命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_Read10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
	{
		/* case 10 : Ho <> Di */
		if ((hmsc->cbw.bmFlags & 0x80U) != 0x80U)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		if (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);

			return -1;
		}

		if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->IsReady(lun) != 0)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
			return -1;
		}

		hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) | ((uint32_t)params[3] << 16) |
							  ((uint32_t)params[4] <<  8) | (uint32_t)params[5];

		hmsc->scsi_blk_len = ((uint32_t)params[7] <<  8) | (uint32_t)params[8];

		if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr, hmsc->scsi_blk_len) < 0)
			return -1; /* error */

		/* cases 4,5 : Hi <> Dn */
		if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_len * hmsc->scsi_blk_size))
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		hmsc->bot_state = USBD_BOT_DATA_IN;
	}
	hmsc->bot_data_length = MSC_MEDIA_PACKET;

	return SCSI_ProcessRead(pdev, lun);
}


/**
  * @brief  SCSI_Read12 进程Read12命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_Read12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
	{
		/* case 10 : Ho <> Di */
		if ((hmsc->cbw.bmFlags & 0x80U) != 0x80U)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		if (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
			return -1;
		}

		if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->IsReady(lun) != 0)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
			return -1;
		}

		hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) | ((uint32_t)params[3] << 16) |
							  ((uint32_t)params[4] <<  8) | (uint32_t)params[5];

		hmsc->scsi_blk_len = ((uint32_t)params[6] << 24) | ((uint32_t)params[7] << 16) |
							 ((uint32_t)params[8] << 8) | (uint32_t)params[9];

		if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr, hmsc->scsi_blk_len) < 0)
			return -1; /* error */

		/* cases 4,5 : Hi <> Dn */
		if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_len * hmsc->scsi_blk_size))
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		hmsc->bot_state = USBD_BOT_DATA_IN;
	}
	hmsc->bot_data_length = MSC_MEDIA_PACKET;

	return SCSI_ProcessRead(pdev, lun);
}

/**
  * @brief  SCSI_Write10 进程Write10命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_Write10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	uint32_t len;

	if (hmsc == NULL)
		return -1;

	if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
	{
		if (hmsc->cbw.dDataLength == 0U)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		/* case 8 : Hi <> Do */
		if ((hmsc->cbw.bmFlags & 0x80U) == 0x80U)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		/* Check whether Media is ready */
		if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->IsReady(lun) != 0)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
			return -1;
		}

		/* Check If media is write-protected */
		if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->IsWriteProtected(lun) != 0)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, WRITE_PROTECTED);
			return -1;
		}

		hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) | ((uint32_t)params[3] << 16) |
							  ((uint32_t)params[4] << 8) | (uint32_t)params[5];

		hmsc->scsi_blk_len = ((uint32_t)params[7] << 8) | (uint32_t)params[8];

		/* check if LBA address is in the right range */
		if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr, hmsc->scsi_blk_len) < 0)
		return -1; /* error */

		len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

		/* cases 3,11,13 : Hn,Ho <> D0 */
		if (hmsc->cbw.dDataLength != len)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		len = MIN(len, MSC_MEDIA_PACKET);

		/* Prepare EP to receive first data packet */
		hmsc->bot_state = USBD_BOT_DATA_OUT;
		(void)USBD_LL_PrepareReceive(pdev, COM_MSC_OUT_EP, hmsc->bot_data, len);
	}
	else /* Write Process ongoing */
		return SCSI_ProcessWrite(pdev, lun);

	return 0;
}


/**
  * @brief  SCSI_Write12 进程Write12命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_Write12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	uint32_t len;

	if (hmsc == NULL)
		return -1;

	if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
	{
		if (hmsc->cbw.dDataLength == 0U)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		/* case 8 : Hi <> Do */
		if ((hmsc->cbw.bmFlags & 0x80U) == 0x80U)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		/* Check whether Media is ready */
		if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->IsReady(lun) != 0)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
			hmsc->bot_state = USBD_BOT_NO_DATA;
			return -1;
		}

		/* Check If media is write-protected */
		if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->IsWriteProtected(lun) != 0)
		{
			SCSI_SenseCode(pdev, lun, NOT_READY, WRITE_PROTECTED);
			hmsc->bot_state = USBD_BOT_NO_DATA;
			return -1;
		}

		hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) | ((uint32_t)params[3] << 16) |
							  ((uint32_t)params[4] << 8) | (uint32_t)params[5];

		hmsc->scsi_blk_len = ((uint32_t)params[6] << 24) | ((uint32_t)params[7] << 16) |
							 ((uint32_t)params[8] << 8) | (uint32_t)params[9];

		/* check if LBA address is in the right range */
		if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr, hmsc->scsi_blk_len) < 0)
			return -1; /* error */

		len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

		/* cases 3,11,13 : Hn,Ho <> D0 */
		if (hmsc->cbw.dDataLength != len)
		{
			SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
			return -1;
		}

		len = MIN(len, MSC_MEDIA_PACKET);

		/* Prepare EP to receive first data packet */
		hmsc->bot_state = USBD_BOT_DATA_OUT;
		(void)USBD_LL_PrepareReceive(pdev, COM_MSC_OUT_EP, hmsc->bot_data, len);
	}
	else /* Write Process ongoing */
		return SCSI_ProcessWrite(pdev, lun);

	return 0;
}


/**
  * @brief  SCSI_Verify10 进程Verify10命令
  * @param  lun: Logical unit number
  * @param  params: Command parameters
  * @retval status
  */
static int8_t SCSI_Verify10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if ((params[1] & 0x02U) == 0x02U)
	{
		SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
		return -1; /* Error, Verify Mode Not supported*/
	}

	if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr, hmsc->scsi_blk_len) < 0)
		return -1; /* error */

	hmsc->bot_data_length = 0U;

	return 0;
}

/**
  * @brief  SCSI_CheckAddressRange 检查地址范围
  * @param  lun: Logical unit number
  * @param  blk_offset: first block address
  * @param  blk_nbr: number of block to be processed
  * @retval status
  */
static int8_t SCSI_CheckAddressRange(USBD_HandleTypeDef *pdev, uint8_t lun, uint32_t blk_offset, uint32_t blk_nbr)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hmsc == NULL)
		return -1;

	if ((blk_offset + blk_nbr) > hmsc->scsi_blk_nbr)
	{
		SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, ADDRESS_OUT_OF_RANGE);
		return -1;
	}

	return 0;
}

/**
  * @brief  SCSI_ProcessRead 读进程
  * @param  lun: Logical unit number
  * @retval status
  */
static int8_t SCSI_ProcessRead(USBD_HandleTypeDef *pdev, uint8_t lun)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	uint32_t len;

	if (hmsc == NULL)
		return -1;

	len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

	len = MIN(len, MSC_MEDIA_PACKET);

	if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->Read(lun, hmsc->bot_data, hmsc->scsi_blk_addr, (len / hmsc->scsi_blk_size)) < 0)
	{
		SCSI_SenseCode(pdev, lun, HARDWARE_ERROR, UNRECOVERED_READ_ERROR);
		return -1;
	}

	(void)USBD_LL_Transmit(pdev, COM_MSC_IN_EP, hmsc->bot_data, len);

	hmsc->scsi_blk_addr += (len / hmsc->scsi_blk_size);
	hmsc->scsi_blk_len -= (len / hmsc->scsi_blk_size);

	/* case 6 : Hi = Di */
	hmsc->csw.dDataResidue -= len;

	if (hmsc->scsi_blk_len == 0U)
		hmsc->bot_state = USBD_BOT_LAST_DATA_IN;

	return 0;
}

/**
  * @brief  SCSI_ProcessWrite 写过程
  * @param  lun: Logical unit number
  * @retval status
  */
static int8_t SCSI_ProcessWrite(USBD_HandleTypeDef *pdev, uint8_t lun)
{
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	uint32_t len;

	if (hmsc == NULL)
		return -1;

	len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

	len = MIN(len, MSC_MEDIA_PACKET);

	if (((USBD_StorageTypeDef *)pdev->pUserData[pdev->classId])->Write(lun, hmsc->bot_data, hmsc->scsi_blk_addr, (len / hmsc->scsi_blk_size)) < 0)
	{
		SCSI_SenseCode(pdev, lun, HARDWARE_ERROR, WRITE_FAULT);
		return -1;
	}

	hmsc->scsi_blk_addr += (len / hmsc->scsi_blk_size);
	hmsc->scsi_blk_len -= (len / hmsc->scsi_blk_size);

	/* case 12 : Ho = Do */
	hmsc->csw.dDataResidue -= len;

	if (hmsc->scsi_blk_len == 0U)
		MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_PASSED);
	else
	{
		len = MIN((hmsc->scsi_blk_len * hmsc->scsi_blk_size), MSC_MEDIA_PACKET);

		/* Prepare EP to Receive next packet */
		(void)USBD_LL_PrepareReceive(pdev, COM_MSC_OUT_EP, hmsc->bot_data, len);
	}

	return 0;
}


/**
  * @brief  SCSI_UpdateBotData 将请求的数据填充到传输缓冲区
  * @param  hmsc handler
  * @param  pBuff: Data buffer
  * @param  length: Data length
  * @retval status
  */
static int8_t SCSI_UpdateBotData(USBD_MSC_BOT_HandleTypeDef *hmsc, uint8_t *pBuff, uint16_t length)
{
	uint16_t len = length;

	if (hmsc == NULL)
		return -1;

	hmsc->bot_data_length = len;

	while (len != 0U)
	{
		len--;
		hmsc->bot_data[len] = pBuff[len];
	}

	return 0;
}



