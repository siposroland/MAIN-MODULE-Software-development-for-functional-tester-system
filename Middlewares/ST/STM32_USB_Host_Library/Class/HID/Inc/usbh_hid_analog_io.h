/**
  ******************************************************************************
  * @file    usbd_digital_io.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for the usbd_hid_core.c file.
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
/* Includes -------------------------------------*/
//#include "usbh_customhid.h"
#include "usbh_hid.h"

/* Defines -------------------------------------*/
#ifndef __USBD_ANALOG_IO_H
#define __USBD_ANALOG_IO_H

#define ANALOG_MAX_OUT_NUM		(0x01U)
#define ANALOG_MAX_IN_NUM 		(0x02U)

#define ANALOG_IO_MAX_TRIG_NUM (0x02U)


#ifdef __cplusplus
 extern "C" {
#endif


 typedef enum {
	 OUT_CONST,
	 OUT_SINUS,
	 OUT_TRIANGLE
 } HID_ANALOG_IO_Out_Type;


 typedef struct _HID_ANALOG_IO_Info
 {
   uint16_t     			in[ANALOG_MAX_IN_NUM];
   uint16_t					out[ANALOG_MAX_OUT_NUM];
   uint16_t					value[ANALOG_MAX_OUT_NUM];
   HID_ANALOG_IO_Out_Type 	type[ANALOG_MAX_OUT_NUM];
 }  HID_ANALOG_IO_Info_TypeDef;


 USBH_StatusTypeDef USBH_HID_Analog_IO_Init(USBH_HandleTypeDef *phost);
 HID_ANALOG_IO_Info_TypeDef *USBH_HID_Get_Analog_IO_Info(USBH_HandleTypeDef *phost);
 //USBH_StatusTypeDef USBH_HID_Mouse_DecodeData(USBH_HandleTypeDef *phost, HID_Report_ItemTypedef *parser_data, uint8_t *pData, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif  /* __USBD_DIGITAL_IO_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
