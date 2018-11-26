/*
 * cmd_interpreter.h
 *
 * \brief  This file contains the CMD Interpreter functions
 *
 * Author: Sipos Roland -  PCB Design Ltd.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 PCB Design Ltd</center></h2>
 */ 


#ifndef CMD_INTERPRETER_H_
#define CMD_INTERPRETER_H_

/* == include files ========================================================== */
#include "usbh_core.h"
/* == #defines ================================================================ */

#define DIGITAL_BIT_ENABLE		(0u)
#define DIGITAL_BIT_DIRECTION	(1u)
#define DIGITAL_BIT_PULLDOWN	(2u)
#define DIGITAL_BIT_PULLUP		(3u)
#define DIGITAL_BIT_PIN_0		(4u)
#define DIGITAL_BIT_PIN_1		(5u)
#define DIGITAL_BIT_PIN_2		(6u)
#define DIGITAL_BIT_PIN_3		(7u)

#define CMD_OK					(0u)
#define CMD_ERROR				(1u)

/* == types ==================================================================== */
typedef enum{
	NOPULL = 0,
	PULLDOWN = 1,
	PULLUP = 2
} PORT_PULL_Type;
/* == global constants ========================================================== */

/* == global variables ========================================================== */
extern uint8_t change_buffer[7];
extern uint8_t cmd_change_flag;
extern uint8_t cmd_trigger_flag;
/* == public function prototypes ============================================== */
/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t Command_Interpreter_Digital(uint8_t* cmd);

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_store_msg(uint8_t* cmd, uint8_t idx);

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_null_msg(uint8_t* cmd, uint8_t idx);

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_change_msg(uint8_t* cmd, uint8_t idx);

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_trigger_msg(uint8_t* cmd, uint8_t idx);


#endif /* CMD_INTERPRETER_H_ */
