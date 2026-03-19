#include "bsp_can.h"
#include "can.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define CAN1_ID_H      0x0000 //32位基础ID设置（高16位）
#define CAN1_ID_L      0x0000 //32位基础ID设置（低16位）
#define CAN1_MASK_H    0x0000 //32位屏蔽MASK设置（高16位）
#define CAN1_MASK_L    0x0000 //32位屏蔽MASK设置（低16位）

static CAN_HandleTypeDef* UserCan1 = &hcan1; //使用的CAN句柄名称
static CAN_HandleTypeDef* UserCan2 = &hcan2; //使用的CAN句柄名称

static SemaphoreHandle_t can1_mutex = NULL;
static SemaphoreHandle_t can2_mutex = NULL;

//CAN总线滤波器初始化
static void CAN1_Filter_Init(void)
{
	static uint8_t init = 0;
    CAN_FilterTypeDef  sFilterConfig;
    HAL_StatusTypeDef  HAL_Status;
	
	//创建CAN1互斥量
	can1_mutex = xSemaphoreCreateMutex();
	
	if( 1==init ) return;
	
    sFilterConfig.FilterBank = 0;						//使用过滤器0
    sFilterConfig.FilterMode =   CAN_FILTERMODE_IDMASK; //设为IDLIST列表模式/IDMASK屏蔽模式
    sFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT; //过滤器位宽度
    sFilterConfig.FilterIdHigh = CAN1_ID_H;				//32位基础ID设置（高16位）
    sFilterConfig.FilterIdLow  = CAN1_ID_L;				//32位基础ID设置（低16位）
    sFilterConfig.FilterMaskIdHigh =  CAN1_MASK_H;		//32位屏蔽MASK设置（高16位）
    sFilterConfig.FilterMaskIdLow  =  CAN1_MASK_L;		//32位屏蔽MASK设置（低16位）
    sFilterConfig.FilterFIFOAssignment =  CAN_RX_FIFO0; //接收到的报文放入FIFO0中
    sFilterConfig.FilterActivation =  ENABLE;			//ENABLE激活过滤器，DISABLE禁止过滤器
    sFilterConfig.SlaveStartFilterBank  = 14;			//过滤器组设置，双CAN通讯时CAN1必须从14开始
	
    HAL_Status=HAL_CAN_ConfigFilter(UserCan1,&sFilterConfig);//将以上结构体参数设置到CAN寄存器中
	
	//CAN滤波器初始化失败需要的提示
    if(HAL_Status!=HAL_OK) {}
	
	//开启CAN总线功能
    HAL_Status=HAL_CAN_Start(UserCan1);
	
	//CAN总线开启失败需要的提示
    if(HAL_Status!=HAL_OK){}

	//开启CAN总线中断
    HAL_Status=HAL_CAN_ActivateNotification(UserCan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    
	//CAN总线开启失败需要的提示
	if(HAL_Status!=HAL_OK){}
	
	init = 1;
}

//CAN总线滤波器初始化
static void CAN2_Filter_Init(void)
{
	static uint8_t init = 0;
    CAN_FilterTypeDef  sFilterConfig;
    HAL_StatusTypeDef  HAL_Status;
	
	can2_mutex = xSemaphoreCreateMutex();
	
	if( 1==init ) return;
	
    sFilterConfig.FilterBank = 14;						//使用过滤器0
    sFilterConfig.FilterMode =   CAN_FILTERMODE_IDMASK; //设为IDLIST列表模式/IDMASK屏蔽模式
    sFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT; //过滤器位宽度
    sFilterConfig.FilterIdHigh = CAN1_ID_H;				//32位基础ID设置（高16位）
    sFilterConfig.FilterIdLow  = CAN1_ID_L;				//32位基础ID设置（低16位）
    sFilterConfig.FilterMaskIdHigh =  CAN1_MASK_H;		//32位屏蔽MASK设置（高16位）
    sFilterConfig.FilterMaskIdLow  =  CAN1_MASK_L;		//32位屏蔽MASK设置（低16位）
    sFilterConfig.FilterFIFOAssignment =  CAN_RX_FIFO1; //接收到的报文放入FIFO1中
    sFilterConfig.FilterActivation =  ENABLE;			//ENABLE激活过滤器，DISABLE禁止过滤器
	
    HAL_Status=HAL_CAN_ConfigFilter(UserCan2,&sFilterConfig);//将以上结构体参数设置到CAN寄存器中
	
	//CAN滤波器初始化失败需要的提示
    if(HAL_Status!=HAL_OK) {}
	
	//开启CAN总线功能
    HAL_Status=HAL_CAN_Start(UserCan2);
	
	//CAN总线开启失败需要的提示
    if(HAL_Status!=HAL_OK){}

	//开启CAN总线中断
    HAL_Status=HAL_CAN_ActivateNotification(UserCan2,CAN_IT_RX_FIFO1_MSG_PENDING);
    
	//CAN总线开启失败需要的提示
	if(HAL_Status!=HAL_OK){}
	
	init = 1;
}



////旧版C50C适配
//static void CAN1_Filter_Init_V1_00(void)
//{
//	static uint8_t init = 0;
//    CAN_FilterTypeDef  sFilterConfig;
//    HAL_StatusTypeDef  HAL_Status;
//	
//	if( 1==init ) return;
//	
//	//旧版C50C,无usb接口,需要对usb接口进行反向初始化
//    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
//    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
//	
//	//对cubemx配置版本的can进行反初始化
//    __HAL_RCC_CAN1_CLK_DISABLE();
//    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);
//    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
//	
//	//初始化CAN io
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	__HAL_RCC_CAN1_CLK_ENABLE();
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
//	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
//	
//	//滤波器初始化
//    sFilterConfig.FilterBank = 0;						//使用过滤器0
//    sFilterConfig.FilterMode =   CAN_FILTERMODE_IDMASK; //设为IDLIST列表模式/IDMASK屏蔽模式
//    sFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT; //过滤器位宽度
//    sFilterConfig.FilterIdHigh = CAN1_ID_H;				//32位基础ID设置（高16位）
//    sFilterConfig.FilterIdLow  = CAN1_ID_L;				//32位基础ID设置（低16位）
//    sFilterConfig.FilterMaskIdHigh =  CAN1_MASK_H;		//32位屏蔽MASK设置（高16位）
//    sFilterConfig.FilterMaskIdLow  =  CAN1_MASK_L;		//32位屏蔽MASK设置（低16位）
//    sFilterConfig.FilterFIFOAssignment =  CAN_RX_FIFO1; //接收到的报文放入FIFO1中
//    sFilterConfig.FilterActivation =  ENABLE;			//ENABLE激活过滤器，DISABLE禁止过滤器
//    sFilterConfig.SlaveStartFilterBank  = 0;			//过滤器组设置，双CAN通讯时CAN1必须从14开始
//	
//    HAL_Status=HAL_CAN_ConfigFilter(UserCan,&sFilterConfig);//将以上结构体参数设置到CAN寄存器中
//	
//	//CAN滤波器初始化失败需要的提示
//    if(HAL_Status!=HAL_OK) {}
//	
//	//开启CAN总线功能
//    HAL_Status=HAL_CAN_Start(UserCan);
//	
//	//CAN总线开启失败需要的提示
//    if(HAL_Status!=HAL_OK){}

//	//开启CAN总线中断
//    HAL_Status=HAL_CAN_ActivateNotification(UserCan,CAN_IT_RX_FIFO1_MSG_PENDING);
//    
//	//CAN总线开启失败需要的提示
//	if(HAL_Status!=HAL_OK){}
//	
//	init = 1;
//}

static uint8_t can1_send_StdNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //CAN传输状态的枚举
	uint8_t  FreeTxNum=0;         //空闲邮箱的数量
	uint32_t TxMailBox = 0;       //接收发送数据的邮箱,硬件确定并赋值
	CAN_TxHeaderTypeDef TxMsg;    //发送数据的结构体
	
	//数据有效性判断
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can1_mutex, portMAX_DELAY) == pdTRUE)
	{
		//等待有空闲邮箱
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan1);
		}
		
		TxMsg.StdId = id;      //帧id
		TxMsg.IDE = CAN_ID_STD;//标准帧
		
		//CAN_RTR_DATA    数据帧
		//CAN_RTR_REMOTE  摇控帧
		TxMsg.RTR = CAN_RTR_DATA;
		
		//数据长度,取值1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan1,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can1_mutex);
	}
		
	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

static uint8_t can1_send_ExtNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //CAN传输状态的枚举
	uint8_t  FreeTxNum=0;         //空闲邮箱的数量
	uint32_t TxMailBox = 0;       //接收发送数据的邮箱,硬件确定并赋值
	CAN_TxHeaderTypeDef TxMsg;    //发送数据的结构体
	
	//数据有效性判断
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can1_mutex, portMAX_DELAY) == pdTRUE)
	{
		//等待有空闲邮箱
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan1);
		}
		
		TxMsg.ExtId = id;      //帧id
		TxMsg.IDE = CAN_ID_EXT;//扩展帧
		
		//CAN_RTR_DATA    数据帧
		//CAN_RTR_REMOTE  摇控帧
		TxMsg.RTR = CAN_RTR_DATA;
		
		//数据长度,取值1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan1,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can1_mutex);
	}

	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

static uint8_t can2_send_StdNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //CAN传输状态的枚举
	uint8_t  FreeTxNum=0;         //空闲邮箱的数量
	uint32_t TxMailBox = 0;       //接收发送数据的邮箱,硬件确定并赋值
	CAN_TxHeaderTypeDef TxMsg;    //发送数据的结构体
	
	//数据有效性判断
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can2_mutex, portMAX_DELAY) == pdTRUE)
	{
		//等待有空闲邮箱
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan2);
		}
		
		TxMsg.StdId = id;      //帧id
		TxMsg.IDE = CAN_ID_STD;//标准帧
		
		//CAN_RTR_DATA    数据帧
		//CAN_RTR_REMOTE  摇控帧
		TxMsg.RTR = CAN_RTR_DATA;
		
		//数据长度,取值1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan2,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can2_mutex);
	}
		
	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

static uint8_t can2_send_ExtNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //CAN传输状态的枚举
	uint8_t  FreeTxNum=0;         //空闲邮箱的数量
	uint32_t TxMailBox = 0;       //接收发送数据的邮箱,硬件确定并赋值
	CAN_TxHeaderTypeDef TxMsg;    //发送数据的结构体
	
	//数据有效性判断
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can2_mutex, portMAX_DELAY) == pdTRUE)
	{
		//等待有空闲邮箱
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan2);
		}
		
		TxMsg.ExtId = id;      //帧id
		TxMsg.IDE = CAN_ID_EXT;//扩展帧
		
		//CAN_RTR_DATA    数据帧
		//CAN_RTR_REMOTE  摇控帧
		TxMsg.RTR = CAN_RTR_DATA;
		
		//数据长度,取值1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan2,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can2_mutex);
	}

	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

CANInterface_t UserCAN1Dev = {
	.init = CAN1_Filter_Init,
	.sendStd = can1_send_StdNum,
	.sendExt = can1_send_ExtNum,
};

CANInterface_t UserCAN2Dev = {
	.init = CAN2_Filter_Init,
	.sendStd = can2_send_StdNum,
	.sendExt = can2_send_ExtNum,
};
