/*
 * usbh_conf.c
 *
 *  Created on: 30/12/2015
 *      Author: Mori
 */

#include "stm32f4xx_hal.h"
#include "usbh_core.h"
#include "stm32f4xx_hal_hcd.h"


#define HOST_POWERSW_FS_CLK_ENABLE()          __GPIOC_CLK_ENABLE()
#define HOST_POWERSW_FS_PORT                  GPIOC
#define HOST_POWERSW_FS_VBUS                  GPIO_PIN_0

#define HOST_POWERSW_HS_CLK_ENABLE()          __GPIOB_CLK_ENABLE()
#define HOST_POWERSW_HS_PORT                  GPIOB
#define HOST_POWERSW_HS_VBUS                  GPIO_PIN_0

HCD_HandleTypeDef _hHCD[2];

extern USBH_HandleTypeDef hUSBHost[5];

/*******************************************************************************
                       HCD BSP Routines
*******************************************************************************/
/**
  * @brief  Initializes the HCD MSP.
  * @param  hHCD: HCD handle
  * @retval None
  */
void HAL_HCD_MspInit(HCD_HandleTypeDef *hHCD)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

USBH_UsrLog("HAL_HCD_MspInit");

	if(hHCD->Instance == USB_OTG_HS)
	{
		//PB0  = OTG_HS_VBUS
		//PB14 = OTG_HS_DM
		//PB15 = OTG_HS_DP

		__GPIOB_CLK_ENABLE();
//		HOST_POWERSW_HS_CLK_ENABLE();

		/* Configure DM DP Pins */
		GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;

		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    	GPIO_InitStruct.Pull = GPIO_NOPULL;
    	GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    	/* Configure Power Switch Vbus Pin */
//    	GPIO_InitStruct.Pin = HOST_POWERSW_HS_VBUS;
//    	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    	GPIO_InitStruct.Pull = GPIO_NOPULL;
//    	HAL_GPIO_Init(HOST_POWERSW_HS_PORT, &GPIO_InitStruct);

    	/* Enable USB HS Clocks */
    	__USB_OTG_HS_CLK_ENABLE();

    	/* Set USBFS Interrupt to the lowest priority */
    	/*  A lower priority value indicates a higher priority */
    	HAL_NVIC_SetPriority(OTG_HS_IRQn, 1, 0);

    	/* Enable USBFS Interrupt */
    	HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
	}
	else if(hHCD->Instance == USB_OTG_FS)
	{
		//PA9  = OTG_FS_VBUS
		//PA11 = OTG_FS_DM
		//PA12 = OTG_FS_DP

		__GPIOA_CLK_ENABLE();

#ifdef STM32F4DISCOVERY
		HOST_POWERSW_FS_CLK_ENABLE();
#endif
		/* Configure DM DP Pins */
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    	/* Configure Power Switch Vbus Pin */
#ifdef STM32F4DISCOVERY
		GPIO_InitStruct.Pin = HOST_POWERSW_FS_VBUS;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(HOST_POWERSW_FS_PORT, &GPIO_InitStruct);
#endif
		/* Enable USB FS Clocks */
		__USB_OTG_FS_CLK_ENABLE();

		/* Set USBFS Interrupt to the lowest priority */
		/*  A lower priority value indicates a higher priority */
		HAL_NVIC_SetPriority(OTG_FS_IRQn, 3, 0);

		/* Enable USBFS Interrupt */
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
	}
}

/**
  * @brief  DeInitializes the HCD MSP.
  * @param  hHCD: HCD handle
  * @retval None
  */
void HAL_HCD_MspDeInit(HCD_HandleTypeDef *hHCD)
{
	if(hHCD->Instance == USB_OTG_HS)
	{
		/* Disable USB HS Clocks */
		__USB_OTG_HS_CLK_DISABLE();
	}
	else
	{
		/* Disable USB FS Clocks */
		__USB_OTG_FS_CLK_DISABLE();
	}
}

/*******************************************************************************
                       LL Driver Callbacks (HCD -> USB Host Library)
*******************************************************************************/

// MORI
extern  USBH_HandleTypeDef hUSBHost[5];

 /**
  * @brief  SOF callback.
  * @param  hHCD: HCD handle
  * @retval None
  */
void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hHCD)
{
	// No endpoint operation will be done here
	int i;
	for(i = 0; i < 5; ++i)
	{
		if(hUSBHost[i].valid)
			USBH_LL_IncTimer(&hUSBHost[i]);
	}

//	USBH_LL_IncTimer(hHCD->pData);
}

/**
  * @brief  Connect callback.
  * @param  hHCD: HCD handle
  * @retval None
  */
void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hHCD)
{
	USBH_LL_Connect(hHCD->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hHCD: HCD handle
  * @retval None
  */
void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hHCD)
{
	USBH_LL_Disconnect(hHCD->pData);
}


/**
  * @brief  Notify URB state change callback.
  * @param  hpcd: HCD handle
  * @retval None
  */
void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hHCD, uint8_t chnum, HCD_URBStateTypeDef urb_state)
{
}

/*******************************************************************************
                       LL Driver Interface (USB Host Library --> HCD)
*******************************************************************************/
/**
  * @brief  USBH_LL_Init
  *         Initialize the Low Level portion of the Host driver.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_LL_Init (USBH_HandleTypeDef *phost)
{
	/* Change Systick priority */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	HCD_HandleTypeDef *phHCD = &_hHCD[phost->id];

	/*Set LL Driver parameters */
	phHCD->Instance = (phost->id == ID_USB_HOST_FS) ? USB_OTG_FS : USB_OTG_HS;
	phHCD->Init.Host_channels = 11;
	phHCD->Init.dma_enable = DISABLE;
	phHCD->Init.low_power_enable = DISABLE;
	phHCD->Init.phy_itface = HCD_PHY_EMBEDDED;
	phHCD->Init.Sof_enable = DISABLE;
	phHCD->Init.speed = HCD_SPEED_FULL;
	phHCD->Init.vbus_sensing_enable = ENABLE;//DISABLE;

	/* Link The driver to the stack */
	phHCD->pData = phost;
	phost->pData = phHCD;

	/*Initialize LL Driver */
	HAL_HCD_Init(phHCD);

	USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(phHCD));

	return USBH_OK;
}

/**
  * @brief  USBH_LL_DeInit
  *         De-Initialize the Low Level portion of the Host driver.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_LL_DeInit (USBH_HandleTypeDef *phost)
{
	HAL_HCD_DeInit(phost->pData);
	return USBH_OK;
}

/**
  * @brief  USBH_LL_Start
  *         Start the Low Level portion of the Host driver.
  * @param  phost: Host handle
  * @retval USBH Status
 */
USBH_StatusTypeDef  USBH_LL_Start(USBH_HandleTypeDef *phost)
{
	HAL_HCD_Start(phost->pData);
	return USBH_OK;
}

/**
  * @brief  USBH_LL_Stop
  *         Stop the Low Level portion of the Host driver.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_LL_Stop (USBH_HandleTypeDef *phost)
{
	HAL_HCD_Stop(phost->pData);
	return USBH_OK;
}

USBH_StatusTypeDef  USBH_LL_StopHC(USBH_HandleTypeDef *phost, uint8_t chnum)
{
	HAL_HCD_StopHC(phost->pData, chnum);
	return USBH_OK;
}


/**
  * @brief  USBH_LL_GetSpeed
  *         Return the USB Host Speed from the Low Level Driver.
  * @param  phost: Host handle
  * @retval USBH Speeds
  */
USBH_SpeedTypeDef USBH_LL_GetSpeed(USBH_HandleTypeDef *phost)
{
	USBH_SpeedTypeDef speed = USBH_SPEED_FULL;

	switch (HAL_HCD_GetCurrentSpeed(phost->pData))
	{
		case 0 :	speed = USBH_SPEED_HIGH;	break;
		case 1 :	speed = USBH_SPEED_FULL;	break;
		case 2 :	speed = USBH_SPEED_LOW;		break;
		default:	speed = USBH_SPEED_FULL;	break;
	}

USBH_DbgLog("USBH_LL_GetSpeed %d (LOW=2,FULL=1)", speed);

	return  speed;
}

/**
  * @brief  USBH_LL_ResetPort
  *         Reset the Host Port of the Low Level Driver.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_LL_ResetPort(USBH_HandleTypeDef *phost)
{
	HAL_HCD_ResetPort(phost->pData);
	return USBH_OK;
}

/**
  * @brief  USBH_LL_GetLastXferSize
  *         Return the last transfered packet size.
  * @param  phost: Host handle
  * @param  pipe: Pipe index
  * @retval Packet Size
  */
uint32_t USBH_LL_GetLastXferSize(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	return HAL_HCD_HC_GetXferCount(phost->pData, pipe);
}

/**
  * @brief  USBH_LL_OpenPipe
  *         Open a pipe of the Low Level Driver.
  * @param  phost: Host handle
  * @param  pipe_num: Pipe index
  * @param  epnum: Endpoint Number
  * @param  dev_address: Device USB address
  * @param  speed: Device Speed
  * @param  ep_type: Endpoint Type
  * @param  mps: Endpoint Max Packet Size
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_LL_OpenPipe(USBH_HandleTypeDef *phost,
                                      uint8_t pipe_num,
                                      uint8_t epnum,
                                      uint8_t dev_address,
                                      uint8_t speed,
                                      uint8_t ep_type,
                                      uint16_t mps)
{
	HAL_HCD_HC_Init(phost->pData, pipe_num, epnum, dev_address, speed, ep_type, mps);
	return USBH_OK;
}

/**
  * @brief  USBH_LL_ClosePipe
  *         Close a pipe of the Low Level Driver.
  * @param  phost: Host handle
  * @param  pipe_num: Pipe index
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_LL_ClosePipe(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	HAL_HCD_HC_Halt(phost->pData, pipe);
	return USBH_OK;
}


/**
  * @brief  USBH_LL_SubmitURB
  *         Submit a new URB to the low level driver.
  * @param  phost: Host handle
  * @param  pipe: Pipe index
  *         This parameter can be a value from 1 to 15
  * @param  direction : Channel number
  *          This parameter can be one of the these values:
  *           0 : Output
  *           1 : Input
  * @param  ep_type : Endpoint Type
  *          This parameter can be one of the these values:
  *            @arg EP_TYPE_CTRL: Control type
  *            @arg EP_TYPE_ISOC: Isochrounous type
  *            @arg EP_TYPE_BULK: Bulk type
  *            @arg EP_TYPE_INTR: Interrupt type
  * @param  token : Endpoint Type
  *          This parameter can be one of the these values:
  *            @arg 0: PID_SETUP
  *            @arg 1: PID_DATA
  * @param  pbuff : pointer to URB data
  * @param  length : Length of URB data
  * @param  do_ping : activate do ping protocol (for high speed only)
  *          This parameter can be one of the these values:
  *           0 : do ping inactive
  *           1 : do ping active
  * @retval Status
  */
USBH_StatusTypeDef   USBH_LL_SubmitURB  (USBH_HandleTypeDef *phost,
                                            uint8_t pipe,
                                            uint8_t direction ,
                                            uint8_t ep_type,
                                            uint8_t token,
                                            uint8_t* pbuff,
                                            uint16_t length,
                                            uint8_t do_ping)
{
//if(phost->transfering)
//{
//	USBH_UsrLog("<<<<<<<< OPS !!! pipe %d, dir: %d, ep_type: %d !!!! >>>>>>>>>", pipe, direction, ep_type);
//	return USBH_BUSY;
//}

//	phost->transfering = 1;

	HAL_HCD_HC_SubmitRequest (phost->pData, pipe, direction, ep_type, token, pbuff, length, do_ping);
	return USBH_OK;
}

/**
  * @brief  USBH_LL_GetURBState
  *         Get a URB state from the low level driver.
  * @param  phost: Host handle
  * @param  pipe: Pipe index
  *         This parameter can be a value from 1 to 15
  * @retval URB state
  *          This parameter can be one of the these values:
  *            @arg URB_IDLE
  *            @arg URB_DONE
  *            @arg URB_NOTREADY
  *            @arg URB_NYET
  *            @arg URB_ERROR
  *            @arg URB_STALL
  */
USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	return (USBH_URBStateTypeDef)HAL_HCD_HC_GetURBState (phost->pData, pipe);
}

/**
  * @brief  USBH_LL_DriverVBUS
  *         Drive VBUS.
  * @param  phost: Host handle
  * @param  state : VBUS state
  *          This parameter can be one of the these values:
  *           0 : VBUS Active
  *           1 : VBUS Inactive
  * @retval Status
  */
USBH_StatusTypeDef USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost, uint8_t state)
{
	/*
	On-chip 5 V VBUS generation is not supported. For this reason, a charge pump
	or, if 5 V are available on the application board, a basic power switch, must
	be added externally to drive the 5 V VBUS line. The external charge pump can
	be driven by any GPIO output. When the application decides to power on VBUS
	using the chosen GPIO, it must also set the port power bit in the host port
	control and status register (PPWR bit in OTG_FS_HPRT).

	Bit 12 PPWR: Port power
	The application uses this field to control power to this port, and the core
	clears this bit on an overcurrent condition.
	*/

	if(_hHCD[phost->id].Instance == USB_OTG_FS)
	{
		if (0 == state)
		{
			/* DISABLE is needed on output of the Power Switch */
			HAL_GPIO_WritePin(HOST_POWERSW_FS_PORT, HOST_POWERSW_FS_VBUS, GPIO_PIN_SET);
		}
		else
		{
			/*ENABLE the Power Switch by driving the Enable LOW */
			HAL_GPIO_WritePin(HOST_POWERSW_FS_PORT, HOST_POWERSW_FS_VBUS, GPIO_PIN_RESET);
		}
	}
	else if(_hHCD[phost->id].Instance == USB_OTG_HS)
	{
		if (0 == state)
		{
			/* DISABLE is needed on output of the Power Switch */
			HAL_GPIO_WritePin(HOST_POWERSW_HS_PORT, HOST_POWERSW_HS_VBUS, GPIO_PIN_RESET);//GPIO_PIN_SET);
		}
		else
		{
			/*ENABLE the Power Switch by driving the Enable LOW */
			HAL_GPIO_WritePin(HOST_POWERSW_HS_PORT, HOST_POWERSW_HS_VBUS, GPIO_PIN_SET);//GPIO_PIN_RESET);
		}
	}

	HAL_Delay(200);
	return USBH_OK;
}

/**
  * @brief  USBH_LL_SetToggle
  *         Set toggle for a pipe.
  * @param  phost: Host handle
  * @param  pipe: Pipe index
  * @param  pipe_num: Pipe index
  * @param  toggle: toggle (0/1)
  * @retval Status
  */
USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef *phost, uint8_t pipe, uint8_t toggle)
{
	if(_hHCD[phost->id].hc[pipe].ep_is_in)
	{
		_hHCD[phost->id].hc[pipe].toggle_in = toggle;
	}
	else
	{
		_hHCD[phost->id].hc[pipe].toggle_out = toggle;
	}

	return USBH_OK;
}

/**
  * @brief  USBH_LL_GetToggle
  *         Return the current toggle of a pipe.
  * @param  phost: Host handle
  * @param  pipe: Pipe index
  * @retval toggle (0/1)
  */
uint8_t USBH_LL_GetToggle(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	uint8_t toggle = 0;

	if(_hHCD[phost->id].hc[pipe].ep_is_in)
	{
		toggle = _hHCD[phost->id].hc[pipe].toggle_in;
	}
	else
	{
		toggle = _hHCD[phost->id].hc[pipe].toggle_out;
	}

	return toggle;
}

/**
  * @brief  USBH_Delay
  *         Delay routine for the USB Host Library
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBH_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

/**
  * @brief  USBH_LL_SetupEP0
  *         Setup endpoint with selected device info
  * @param  phost: Host handle
  * @retval Status
  */
HAL_StatusTypeDef USBH_LL_SetupEP0(USBH_HandleTypeDef *phost)
{
	HCD_HandleTypeDef *phHCD =  &_hHCD[phost->id];

__HAL_LOCK(phHCD);

	phHCD->hc[phost->Control.pipe_out].dev_addr   = phost->device.address;
	phHCD->hc[phost->Control.pipe_out].max_packet = phost->Control.pipe_size;
	phHCD->hc[phost->Control.pipe_out].speed      = phost->device.speed;

//phHCD->hc[phost->Control.pipe_out].ch_num     = phost->Control.pipe_out;
//phHCD->hc[phost->Control.pipe_out].toggle_out = phost->Control.toggle_out;
//phHCD->hc[phost->Control.pipe_out].data_pid = phost->Control.data_pid_out;

	phHCD->hc[phost->Control.pipe_in].dev_addr    = phost->device.address;
	phHCD->hc[phost->Control.pipe_in].max_packet  = phost->Control.pipe_size;
	phHCD->hc[phost->Control.pipe_in].speed       = phost->device.speed;

//phHCD->hc[phost->Control.pipe_in].ch_num      = phost->Control.pipe_in;
//phHCD->hc[phost->Control.pipe_in].toggle_in   = phost->Control.toggle_in;
//phHCD->hc[phost->Control.pipe_in].data_pid   = phost->Control.data_pid_in;

	phHCD->pData = phost;
	phost->pData = phHCD;

__HAL_UNLOCK(phHCD);

	return HAL_OK;
}
