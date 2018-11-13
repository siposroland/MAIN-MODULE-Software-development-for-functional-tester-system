/**
  ******************************************************************************
  * @file    usbh_hid_digital_io.c
  * @author  Roland Sipos, PCB Design LTD.
  * @version V3.2.2
  * @date    11-November-2018
  * @brief   This file is the application layer for USB Host HID Mouse Handling.                  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

#include "usbh_hid_digital_io.h"
#include "usbh_hid_parser.h"

static HID_DIGITAL_IO_Info_TypeDef   digital_io_info;
uint32_t	digital_io_report_data[1];

static USBH_StatusTypeDef USBH_HID_Digital_IO_Decode(USBH_HandleTypeDef *phost);

/* Structures defining how to access items in a HID digital io report */
/* Port number (max 8). */
static const HID_Report_ItemTypedef portnum={
  (uint8_t *)digital_io_report_data+0, /*data*/
  8,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFFFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFFFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin values (max 16 pin). */
static const HID_Report_ItemTypedef pinval={
  (uint8_t *)digital_io_report_data+1, /*data*/
  8,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFFFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFFFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef direction={
  (uint8_t *)digital_io_report_data+2, /*data*/
  1,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};



/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_Digital_IO_Init(USBH_HandleTypeDef *phost)
{
  uint8_t i = 0, j = 0;
  uint8_t idx = phost->device.current_interface;
  HID_HandleTypeDef *HID_Handle = phost->USBH_ClassTypeDef_pData[idx];

  // Initialize default values
  digital_io_info.port_enabled_size = 0;
  digital_io_info.is_inited = 1;
  for(i = 0; i < DIGITAL_MAX_PORT_NUM; i++)
  {
	  digital_io_info.ports[i].pin_enabled_size = 0;
	  digital_io_info.ports[i].direction = DIGITAL_PIN_INPUT;
	  for (j = 0; j < DIGITAL_MAX_PIN_NUM; j++){
		  digital_io_info.ports[i].pins[j] = DIGITAL_PIN_LOW;
	  }
  }
  
  // Initialize default report buffer
  digital_io_report_data[0] = 0;
  

  if(HID_Handle->length[0] > sizeof(digital_io_report_data))
  {
    HID_Handle->length[0] = sizeof(digital_io_report_data);
  }
  HID_Handle->pData = (uint8_t *)digital_io_report_data;
  fifo_init(&HID_Handle->fifo, phost->device.Data, HID_QUEUE_SIZE * sizeof(digital_io_report_data));

  return USBH_OK;
}

/**
  * @brief  USBH_HID_Get_Digital_IO_Info
  *         The function return digital io information.
  * @param  phost: Host handle
  * @retval digital io information
  */
HID_DIGITAL_IO_Info_TypeDef *USBH_HID_Get_Digital_IO_Info(USBH_HandleTypeDef *phost)
{
	uint8_t temp = USBH_HID_GetDeviceType(phost);
	if(temp == HID_DIGITAL_IO)
	{
		if(USBH_HID_Digital_IO_Decode(phost)== USBH_OK)
			return &digital_io_info;
	}

	return NULL;
}

/**
  * @brief  USBH_HID_MouseDecode 
  *         The function decode mouse data.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_Digital_IO_Decode(USBH_HandleTypeDef *phost)
{
//  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	uint8_t idx = phost->device.current_interface;
	HID_HandleTypeDef *HID_Handle = phost->USBH_ClassTypeDef_pData[idx];

  if(HID_Handle->length[0] == 0)
  {
    return USBH_FAIL;
  }
  //Fill report
  if((fifo_read(&HID_Handle->fifo, &digital_io_report_data, HID_Handle->length[0]) ==  HID_Handle->length[0]) /*&& (digital_io_info.is_inited)*/)
  {
	uint8_t actual_port = 0;
	uint8_t i = 0;
    // Decode report

	// Read actual port number
	actual_port = (int16_t )HID_ReadItem((HID_Report_ItemTypedef *) &portnum, 0);

	// Check size and continue decode
	//if(digital_io_info.port_enabled_size > actual_port)
	//{
		// Read IO direction
		digital_io_info.ports[actual_port].direction = (uint8_t)HID_ReadItem((HID_Report_ItemTypedef *) &direction, 0);

		// Read pins
		for (i = 0; i < digital_io_info.ports[actual_port].pin_enabled_size; i++)
		{
			uint16_t temp = (int16_t )HID_ReadItem((HID_Report_ItemTypedef *) &pinval, 0);
			digital_io_info.ports[actual_port].pins[i] = 1 & (temp << i);
		}
	//}
	//else
	//{
		//return   USBH_FAIL;
	//}
    return USBH_OK;  
  }
  return   USBH_FAIL;
}


/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */


/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
