/*
 * ring_buffer.c
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
#include <ring_buffer.h>

/* == #defines ================================================================ */

/* == types ==================================================================== */

/* == constants ================================================================= */

/* == global constants ========================================================== */

/* == global variables ========================================================== */

/* == file-scope (static) variables ============================================= */

/* == file-scope (static) function prototypes =================================== */

/* == file-scope (static) functions defines =================================== */

/* == Public functions defines =============================================== */
/*! \brief Reset the chosen 'ringBuffer'
 * \param ringBuffer struct pointer
 * \return void
 * \note Called from UART.c - UART_Init(...) function
 */
void Ring_Buffer_Reset(ringBuffer_type* ringBuffer)
{
	ringBuffer->wrIdx = 0;
	ringBuffer->rdIdx = 0;
	ringBuffer->rdStart = 0;
	ringBuffer->rdEnd = 0;
	//ringBuffer->fullFlag = FALSE;
}

/*! \brief Add the 'data' element to the chosen 'ringBuffer'
 * \param ringBuffer struct pointer like the target buffer, 8bit data value
 * \return void
 * \note Called from main.c - ISR(USART...) interrupt routines
 */
void Ring_Buffer_Add(ringBuffer_type* ringBuffer, uint8_t data)
{
	// add the data to the ringbuffer
	ringBuffer->data[ringBuffer->wrIdx] = data;
	// step the write index
	ringBuffer->wrIdx = (ringBuffer->wrIdx + 1) % RING_BUFFER_SIZE;
}

/*! \brief Search for the next frame in the ring buffer and copy to the process buffer
 * \param ringBuffer struct pointer, 8bit command data pointer
 * \return find result (TRUE or FALSE)
 * \note Called from main.c - main function -> before start the UART data processes
 */
uint8_t Ring_Buffer_Search_Frame(ringBuffer_type *ringBuffer, uint8_t *command)
{
	uint8_t i;
	uint8_t tempIdx = 0;
	 
	// Save indexes in local variables
	ringBuffer->rdEnd = ringBuffer->wrIdx;
	ringBuffer->rdStart = ringBuffer->rdIdx;
	 
	// step all elements between 'write' and 'read' indexes
	while(ringBuffer->rdStart != ringBuffer->rdEnd)
	{
		// handle frame, when locate the START OF FRAME character
		if (ringBuffer->data[ringBuffer->rdStart] == '*')
		{
			// handle ringBuffer is full
			/*if (ringBuffer->fullFlag == TRUE)
			{
				ringBuffer->rdIdx = ringBuffer->rdStart;
				ringBuffer->fullFlag = FALSE;
				return FALSE;
			}*/
			// search for the END OF FRAME character in the rest of the interval
			tempIdx = (ringBuffer->rdStart + 1) % RING_BUFFER_SIZE;
			while(tempIdx != ringBuffer->rdEnd)
			{
				// handle ringBuffer is full
				/*if (ringBuffer->fullFlag == TRUE)
				{
					ringBuffer->rdIdx = ringBuffer->rdStart;
					ringBuffer->fullFlag = FALSE;
					return FALSE;
				}*/
				// handle, when locate the END OF FRAME character
				if(ringBuffer->data[tempIdx] == '$')
				{
					// copy frame to the process buffer (named in this function: 'command')
					i = 0;
					while(ringBuffer->rdStart != tempIdx)
					{
						// handle ringBuffer is full
						/*if (ringBuffer->fullFlag == TRUE)
						{
							ringBuffer->rdIdx = ringBuffer->rdStart;
							ringBuffer->fullFlag = FALSE;
							return FALSE;
						}*/
						command[i] = ringBuffer->data[ringBuffer->rdStart];
						ringBuffer->rdStart = (ringBuffer->rdStart + 1) % RING_BUFFER_SIZE;
						i ++;
					}
					ringBuffer->rdStart = (ringBuffer->rdStart + 1) % RING_BUFFER_SIZE;
					// step the reader index
					ringBuffer->rdIdx = ringBuffer->rdStart;
					// enable to begin the process
					return i;
				}
				// iteration for the searching of END OF FRAME character
				else
				{
					tempIdx = (tempIdx + 1) % RING_BUFFER_SIZE;
				}
			}
			// step the reader index
			ringBuffer->rdIdx = ringBuffer->rdStart;
			// disable to begin the process
			return FALSE;
		}
		// iteration for the searching of START OF FRAME character
		else
		{
			ringBuffer->rdStart = (ringBuffer->rdStart + 1) % RING_BUFFER_SIZE;
		}
		// step the reader index
		ringBuffer->rdIdx = ringBuffer->rdStart;
	}
	// disable to begin the process
	return FALSE;
}
