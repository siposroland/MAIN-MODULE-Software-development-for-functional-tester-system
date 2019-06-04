/**
  ******************************************************************************
  * @file    usbd_digital_io.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                HID Class  Description
  *          =================================================================== 
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse protocol
  *             - Usage Page : Generic Desktop
  *             - Usage : Digital IO
  *             - Collection : Application 
  *      
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
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

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid_analog_io.h"

/* Global variables */

/* Functions */

#include "usbh_hid_parser.h"

static HID_ANALOG_IO_Info_TypeDef   analog_io_info;
uint32_t	analog_io_report_data[1];

static USBH_StatusTypeDef USBH_HID_Analog_IO_Decode(USBH_HandleTypeDef *phost);

/* Structures defining how to access items in a HID digital io report */
static const HID_Report_ItemTypedef adc0low={
  (uint8_t *)analog_io_report_data+0, /*data*/
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

/* Structures defining how to access items in a HID digital io report */
static const HID_Report_ItemTypedef adc0high={
  (uint8_t *)analog_io_report_data+1, /*data*/
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

/* Structures defining how to access items in a HID digital io report */
static const HID_Report_ItemTypedef adc1low={
  (uint8_t *)analog_io_report_data+2, /*data*/
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

/* Structures defining how to access items in a HID digital io report */
static const HID_Report_ItemTypedef adc1high={
  (uint8_t *)analog_io_report_data+3, /*data*/
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


/* Structures defining how to access items in a HID digital io report */
static const HID_Report_ItemTypedef frame0={
  (uint8_t *)analog_io_report_data+4, /*data*/
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

/* Structures defining how to access items in a HID digital io report */
static const HID_Report_ItemTypedef dac0low={
  (uint8_t *)analog_io_report_data+5, /*data*/
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

/* Structures defining how to access items in a HID digital io report */
static const HID_Report_ItemTypedef dac0high={
  (uint8_t *)analog_io_report_data+6, /*data*/
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

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_Analog_IO_Init(USBH_HandleTypeDef *phost)
{
  uint8_t in_idx = 0, out_idx = 0;
  uint8_t idx = phost->device.current_interface;
  HID_HandleTypeDef *HID_Handle = phost->USBH_ClassTypeDef_pData[idx];

  // Step over all INPUT
  for(in_idx = 0; in_idx < ANALOG_MAX_IN_NUM; in_idx++)
  {
	  // Add default INPUT values
	  analog_io_info.in[in_idx] = 0;
  }
  // Step over all OUTPUT
  for(out_idx = 0; out_idx < ANALOG_MAX_OUT_NUM; out_idx++)
  {
	  // Add default OUTPUT values
	  analog_io_info.out[out_idx] = 0;
	  analog_io_info.type[out_idx] = OUT_CONST;
	  analog_io_info.value[out_idx] = 0;
  }

  // Initialize default report buffer
  analog_io_report_data[0] = 0;


  if(HID_Handle->length[0] > sizeof(analog_io_report_data))
  {
    HID_Handle->length[0] = sizeof(analog_io_report_data);
  }
  HID_Handle->pData = (uint8_t *)analog_io_report_data;
  fifo_init(&HID_Handle->fifo, phost->device.Data, HID_QUEUE_SIZE * sizeof(analog_io_report_data));

  return USBH_OK;

}

/**
  * @brief  USBH_HID_Get_Digital_IO_Info
  *         The function return digital io information.
  * @param  phost: Host handle
  * @retval digital io information
  */
HID_ANALOG_IO_Info_TypeDef *USBH_HID_Get_Analog_IO_Info(USBH_HandleTypeDef *phost)
{
	uint8_t temp = USBH_HID_GetDeviceType(phost);
	if(temp == HID_ANALOG_IO)
	{
		if(USBH_HID_Analog_IO_Decode(phost)== USBH_OK)
			return &analog_io_info;
	}

	return NULL;
}

/**
  * @brief  USBH_HID_MouseDecode
  *         The function decode mouse data.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_Analog_IO_Decode(USBH_HandleTypeDef *phost)
{
//  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	//USBH_SelectInterface(phost, 1);
	uint8_t idx = phost->device.current_interface ;
	HID_HandleTypeDef *HID_Handle = phost->USBH_ClassTypeDef_pData[idx];

  if(HID_Handle->length[0] == 0)
  {
    return USBH_FAIL;
  }

  volatile uint16_t length = HID_Handle->length[0];
  volatile uint16_t temp = fifo_read(&HID_Handle->fifo, &analog_io_report_data, length);

  //Fill report
  if(temp ==  length) /*&& (digital_io_info.is_inited)*/
  {
	uint8_t port_idx = 0, pin_idx = 0;
	uint16_t enable = 01;
    // Decode report

	// Read actual port number
	analog_io_info.in[0] = 0;
	analog_io_info.in[0] = (uint16_t)HID_ReadItem((HID_Report_ItemTypedef*) &adc0low, 0) /*& 0b0000000011111111*/;
	//analog_io_info.in[0] |= ((uint16_t)HID_ReadItem((HID_Report_ItemTypedef*) &adc0high, 0) << 8) & 0b111111100000000;
	// Check size and continue decode
	analog_io_info.in[1] = 0;
	analog_io_info.in[1] = (uint16_t)HID_ReadItem((HID_Report_ItemTypedef *) &adc1low, 0) /*& 0b0000000011111111*/;
	//analog_io_info.in[1] |= ((uint16_t)HID_ReadItem((HID_Report_ItemTypedef *) &adc1high, 0) << 8) & 0b111111100000000;

	enable = (uint16_t)HID_ReadItem((HID_Report_ItemTypedef *) &frame0, 0);
	if(enable == 0){
		analog_io_info.out[0] = 0;
		analog_io_info.out[0] = (uint16_t)HID_ReadItem((HID_Report_ItemTypedef *) &dac0low, 0) & 0b0000000011111111;
		//analog_io_info.out[0] |= ((uint16_t)HID_ReadItem((HID_Report_ItemTypedef *) &dac0high, 0) << 8) & 0b111111100000000;
	}
		// Read IO direction

	/*for(port_idx = 0; port_idx < digital_io_info.port_enabled_size; port_idx++ )
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
	}*/

	//else
	//{
		//return   USBH_FAIL;
	//}
    return USBH_OK;
  }
  return   USBH_FAIL;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
