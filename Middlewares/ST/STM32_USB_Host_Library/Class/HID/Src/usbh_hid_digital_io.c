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
static const HID_Report_ItemTypedef pin0={
  (uint8_t *)digital_io_report_data+1, /*data*/
  4,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Structures defining how to access items in a HID digital io report */
/* Port number (max 8). */
static const HID_Report_ItemTypedef pin1={
  (uint8_t *)digital_io_report_data+1, /*data*/
  4,     	/*size*/
  4,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin values (max 16 pin). */
static const HID_Report_ItemTypedef pin2={
  (uint8_t *)digital_io_report_data+2, /*data*/
  4,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin values (max 16 pin). */
static const HID_Report_ItemTypedef pin3={
  (uint8_t *)digital_io_report_data+2, /*data*/
  4,     	/*size*/
  4,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin values (max 16 pin). */
static const HID_Report_ItemTypedef pin4={
  (uint8_t *)digital_io_report_data+3, /*data*/
  4,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin values (max 16 pin). */
static const HID_Report_ItemTypedef pin5={
  (uint8_t *)digital_io_report_data+3, /*data*/
  4,     	/*size*/
  4,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin values (max 16 pin). */
static const HID_Report_ItemTypedef data7={
  (uint8_t *)digital_io_report_data+7, /*data*/
  8,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

static const HID_Report_ItemTypedef data8={
  (uint8_t *)digital_io_report_data+8, /*data*/
  8,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

static const HID_Report_ItemTypedef data9={
  (uint8_t *)digital_io_report_data+9, /*data*/
  8,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

static const HID_Report_ItemTypedef data10={
  (uint8_t *)digital_io_report_data+10, /*data*/
  8,     	/*size*/
  0,     	/*shift*/
  0,     	/*count (only for array items)*/
  0,     	/*signed?*/
  0,     	/*min value read can return*/
  0xFF,   /*max value read can return*/
  0,     	/*min value device can report*/
  0xFF,   /*max value device can report*/
  1      	/*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef btn1={
  (uint8_t *)digital_io_report_data+0, /*data*/
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

/* Pin direction. */
static const HID_Report_ItemTypedef btn2={
  (uint8_t *)digital_io_report_data+0, /*data*/
  1,     /*size*/
  1,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef btn3={
  (uint8_t *)digital_io_report_data+0, /*data*/
  1,     /*size*/
  2,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef btn4={
  (uint8_t *)digital_io_report_data+0, /*data*/
  1,     /*size*/
  3,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef btn5={
  (uint8_t *)digital_io_report_data+0, /*data*/
  1,     /*size*/
  4,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef btn6={
  (uint8_t *)digital_io_report_data+0, /*data*/
  1,     /*size*/
  5,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef btn7={
  (uint8_t *)digital_io_report_data+0, /*data*/
  1,     /*size*/
  6,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

/* Pin direction. */
static const HID_Report_ItemTypedef btn8={
  (uint8_t *)digital_io_report_data+0, /*data*/
  1,     /*size*/
  7,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min value device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

static const HID_Report_ItemTypedef* btn[8] = {&btn1, &btn2, &btn3, &btn4, &btn5, &btn6, &btn7, &btn8};
static const HID_Report_ItemTypedef* pin[6] = {&pin0, &pin1, &pin2, &pin3, &pin4, &pin5};



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
  digital_io_info.port_enabled_size = 6;
  digital_io_info.is_inited = 1;
  for(i = 0; i < DIGITAL_MAX_PORT_NUM; i++)
  {
	  digital_io_info.ports[i].pin_enabled_size = 4;
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
	uint8_t port_idx = 0, pin_idx = 0;
    // Decode report

	// Read actual port number


	// Check size and continue decode
		// Read IO direction

	for(port_idx = 0; port_idx < digital_io_info.port_enabled_size; port_idx++ )
	{
		digital_io_info.ports[port_idx].direction = (uint8_t)HID_ReadItem((HID_Report_ItemTypedef *) btn[port_idx], 0);
		volatile uint8_t temp = (uint8_t)HID_ReadItem((HID_Report_ItemTypedef *) pin[port_idx], 0);
		// Read pins
		for (pin_idx = 0; pin_idx < digital_io_info.ports[port_idx].pin_enabled_size; pin_idx++)
		{

			volatile uint8_t temp2 = (temp >> pin_idx);
			volatile uint8_t temp3 = 1 & temp2;
			digital_io_info.ports[port_idx].pins[pin_idx] = temp3;
		}
	}

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
