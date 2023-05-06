/*
===============================================================================
 Name        : lpcopen_lpc1114fn28_ex06_adc.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include "my_lpc1114fn28.h"
#include "xprintf.h"

// TODO: insert other definitions and declarations here
void UART_IRQHandler(void)
{
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;
	unsigned char Buf;
	uint8_t d;
	int i, cnt;


	IIRValue = Chip_UART_ReadIntIDReg(LPC_USART);
	IIRValue >>= 1;			/* skip pending bit in IIR */
	IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */

	if (IIRValue == IIR_RLS){		/* Receive Line Status */
		LSRValue = Chip_UART_ReadLineStatus(LPC_USART);
		/* Receive Line Status */
		if (LSRValue & (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_RXFE | UART_LSR_BI)){
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UARTStatus = LSRValue;
			Dummy = Chip_UART_ReadByte(LPC_USART);	/* Dummy read on RX to clear interrupt, then bail out */
			return;
		}
		if (LSRValue & UART_LSR_RDR){	/* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			Buf = Chip_UART_ReadByte(LPC_USART);
		}
	}else if (IIRValue == IIR_RDA){	/* Receive Data Available */
		/* Receive Data Available */
		i = RxBuff.wi;
		cnt = RxBuff.ct;
		while (Chip_UART_ReadLineStatus(LPC_USART) & UART_LSR_RDR) {	/* Get all data in the Rx FIFO */
			d = Chip_UART_ReadByte(LPC_USART);
			if (cnt < BUFF_SIZE) {	/* Store data if Rx buffer is not full */
				RxBuff.buff[i++] = d;
				i %= BUFF_SIZE;
				cnt++;
			}
		}
		RxBuff.wi = i;
		RxBuff.ct = cnt;
	}else if (IIRValue == IIR_CTI){	/* Character timeout indicator */
		/* Character Time-out indicator */
		UARTStatus |= 0x100;		/* Bit 9 as the CTI error */
	}else if (IIRValue == IIR_THRE){	/* THRE, transmit holding register empty */
		cnt = TxBuff.ct;
		if(cnt){/* There is one or more byte to send */
			i = TxBuff.ri;
			for (d = 16; d && cnt; d--, cnt--){	/* Fill Tx FIFO */
				Chip_UART_SendByte(LPC_USART, TxBuff.buff[i++]);
				i %= BUFF_SIZE;
			}
			TxBuff.ri = i;
			TxBuff.ct = cnt;
		}else{
			TxBuff.act = 0; /* When no data to send, next putc() must trigger Tx sequense */
		}
	}
	return;
}

int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif

    // TODO: insert code here
	uint16_t dataADC;

	  /* Initialize the serial port to view the log messages */
	SysTick_Config(SystemCoreClock/1000 - 1); /* Generate interrupt each 1 ms   */

	IOCON_Config_Request();

	UART_Config_Request(115200);
	xdev_out(uart0_putc);
	xprintf ("lpcopen_lpc1114fn28_ex06_adc\n") ;

//	  ADC_Config_Request(ADC_CH0);
	ADC_Config_Request();

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
    	dataADC = ADC_Get_Data(0);
		dataADC = 3300*dataADC/0x3ff;
       xprintf("ADC value is %d mV\r\n", dataADC);
       Delay(500);
		i++ ;
        // "Dummy" NOP to allow source level single
        // stepping of tight while() loop
        __asm volatile ("nop");
    }
    return 0 ;
}
