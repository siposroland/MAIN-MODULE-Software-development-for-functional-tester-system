/*
 * cmd_interpreter.c
 *
 * \brief  This file contains the RingBuffer handler methods
 *
 * Author: Sipos Roland -  PCB Design Ltd.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 PCB Design Ltd</center></h2>
 */ 

/* == include files ========================================================== */
#include "cmd_interpreter.h"
/* == #defines ================================================================ */

/* == types ==================================================================== */

/* == constants ================================================================= */

/* == global constants ========================================================== */

/* == global variables ========================================================== */
uint8_t change_buffer[7] = {0};
uint8_t trig_event_buffer[6] = {0};
uint8_t cmd_change_flag = 0;
uint8_t cmd_trigger_flag = 0;
uint8_t cmd_trigger_event_flag = 0;
/* == file-scope (static) variables ============================================= */

/* == file-scope (static) function prototypes =================================== */

/* == file-scope (static) functions defines =================================== */

/* == Public functions defines =============================================== */
/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t Command_Interpreter_Digital(uint8_t* cmd)
{
	uint8_t idx = 1;
	if (cmd[idx+1] == 'S' && cmd[idx+2] == 'T' && cmd[idx+3] == 'O' && cmd[idx+4] == 'R')
	{
		return process_store_msg(cmd, idx+4);
	}
	else if (cmd[idx+1] == 'N' && cmd[idx+2] == 'U' && cmd[idx+3] == 'L' && cmd[idx+4] == 'L')
	{
		return process_null_msg(cmd, idx+4);
	}
	else if (cmd[idx+1] == 'T' && cmd[idx+2] == 'R' && cmd[idx+3] == 'I' && cmd[idx+4] == 'G')
	{
		return process_trigger_msg(cmd, idx+4);
	}
	else if (cmd[idx+1] == 'C' && cmd[idx+2] == 'H' && cmd[idx+3] == 'N' && cmd[idx+4] == 'G')
	{
		return process_change_msg(cmd, idx+4);
	}
	else if (cmd[idx+1] == 'T' && cmd[idx+2] == 'R' && cmd[idx+3] == 'E' && cmd[idx+4] == 'V')
	{
		return process_trigger_event_msg(cmd, idx+4);
	}
	else
	{
		return CMD_ERROR;
	}
}

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_store_msg(uint8_t* cmd, uint8_t idx)
{
	uint8_t port_num = 0;

	//  ********** STEP 1 **********
	// read port number and add enable flag (ASCII conversion)
	port_num = cmd[++idx] - 48 + 1;
	change_buffer[port_num] |= (1 << DIGITAL_BIT_ENABLE);

	//  ********** STEP 2 **********
	// add io direction
	if (cmd[++idx] == 'I') {
		// set input as 0
		change_buffer[port_num] &= !(1 << DIGITAL_BIT_DIRECTION);
	}
	else if (cmd[idx] == 'O') {
		// set input as 1
		change_buffer[port_num] |= (1 << DIGITAL_BIT_DIRECTION);
	}
	else {
		return CMD_ERROR;
	}

	//  ********** STEP 3 **********
	// add pull info
	if (cmd[++idx] == 'D')
	{
		change_buffer[port_num] |= (1 << DIGITAL_BIT_PULLDOWN);
	}
	else if (cmd[idx] == 'U')
	{
		change_buffer[port_num] |= (1 << DIGITAL_BIT_PULLUP);
	}
	else if (cmd[idx] == 'N')
	{
		/*change_buffer[port_num] &= !(1 << DIGITAL_BIT_PULLDOWN);
		change_buffer[port_num] &= !(1 << DIGITAL_BIT_PULLUP);*/
	}

	//  ********** STEP 4 **********
	// add pin values
	uint8_t p1 = (cmd[++idx]-48)<< DIGITAL_BIT_PIN_0;
	uint8_t p2 = (cmd[++idx]-48)<< DIGITAL_BIT_PIN_1;
	uint8_t p3 = (cmd[++idx]-48)<< DIGITAL_BIT_PIN_2;
	uint8_t p4 = (cmd[++idx]-48)<< DIGITAL_BIT_PIN_3;
	change_buffer[port_num] |= (p1);
	change_buffer[port_num] |= (p2);
	change_buffer[port_num] |= (p3);
	change_buffer[port_num] |= (p4);
	return CMD_OK;
}

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_null_msg(uint8_t* cmd, uint8_t idx)
{
	uint8_t null_idx = 0;
	for (null_idx = 0; null_idx < 7; null_idx ++)
	{
		change_buffer[null_idx] = 0;
	}
	return CMD_OK;
}

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_change_msg(uint8_t* cmd, uint8_t idx)
{
	cmd_change_flag = 1;
	return CMD_OK;
}

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_trigger_msg(uint8_t* cmd, uint8_t idx)
{
	cmd_trigger_flag = 1;
	return CMD_OK;
}

/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
uint8_t process_trigger_event_msg(uint8_t* cmd, uint8_t idx)
{
	/* example1: *DTREV03210131001$
	 * example2: *DTREV1NOT$ -> DISABLE TRIGGER
	 * PROTOCOL: *D TREV X A BCD ... BCD$ -> X = trig event ID, A = number of ANDs, B = number of port, C = number of pin, D = pin trigger value
	 * X = MAX 2
	 * A = MAX 3
	 * B = MAX 6
	 * C = MAX 4
	 * D = MAX 1
	 * */

	uint8_t i = 0, num_of_ANDs = 0;

	//  ********** STEP 0 **********
	// Reset buffers
	for(i = 0; i < 6; i++)
	{
		trig_event_buffer[i] = 0;
	}

	//  ********** STEP 1 **********
	// Read trigger event ID (ASCII conversion) (X)
	trig_event_buffer[1] |= ((cmd[++idx] - 48) << 1);

	// Check ENABLE
	if(cmd[++idx] == 'N' && cmd[idx+1] == 'O' && cmd[idx+2] == 'T')
	{
		return CMD_OK;
	}
	else
	{
		// Add ENABLE bit
		trig_event_buffer[1] |= 1;
	}

	//  ********** STEP 2 **********
	// Read number of ANDs (ASCII conversion) (A)
	num_of_ANDs = (cmd[idx] - 48);
	trig_event_buffer[1] |= (num_of_ANDs << 5);

	//  ********** STEP3 **********
	// Read parameter arguments
	for (i = 0; i < num_of_ANDs + 1; i++)
	{
		trig_event_buffer[i + 2] |= (cmd[++idx] - 48);
		trig_event_buffer[i + 2] |= ((cmd[++idx] - 48) << 3);
		trig_event_buffer[i + 2] |= ((cmd[++idx] - 48) << 6);
	}

	cmd_trigger_event_flag = 1;

	return CMD_OK;
}


