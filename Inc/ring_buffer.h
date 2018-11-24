/*
 * ring_buffer.h
 *
 * \brief  This file contains the RingBuffer handler methods
 *
 * Author: Sipos Roland -  PCB Design Ltd.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 PCB Design Ltd</center></h2>
 */ 


#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

/* == include files ========================================================== */
#include "usbh_core.h"
/* == #defines ================================================================ */


#define RING_BUFFER_SIZE	(256)

/* == types ==================================================================== */
typedef struct ringBuffer_struct {
	uint8_t wrIdx;
	uint8_t rdIdx;
	uint8_t rdStart;
	uint8_t rdEnd;
	uint8_t data[RING_BUFFER_SIZE];
} ringBuffer_type;

/* == global constants ========================================================== */

/* == global variables ========================================================== */

/* == public function prototypes ============================================== */
/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
void Ring_Buffer_Reset(ringBuffer_type* ringBuffer);

/*! \brief Add the 'data' element to the chosen 'ringBuffer'
 * \param ringBuffer struct pointer like the target buffer, 8bit data value
 * \return void
 * \note Called from main.c -
 */
void Ring_Buffer_Add(ringBuffer_type* ringBuffer, uint8_t data);

/*! \brief Search for the next frame in the ring buffer and copy to the process buffer
 * \param ringBuffer struct pointer, 8bit command data pointer
 * \return find result (TRUE -> then return the length of the command or FALSE)
 * \note Called from main.c - main function -> before start the UART data processes
 */
uint8_t Ring_Buffer_Search_Frame(ringBuffer_type *ringBuffer, uint8_t *command);


#endif /* RING_BUFFER_H_ */
