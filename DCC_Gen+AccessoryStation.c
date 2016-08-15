// ARM C-software voor genereren van DCC-signalen i.s.m. IP's DCC_Gen (versie 3) en PWM8620 (versie 1)
// met aanpassing voor genereren van Accessory Decoder packets (voor Arduino)
// Test 04/08/2016: passed

//#include <stdio.h>	// disable when program itself directly controls UART ... (thus no xil_printf anymore...)

#include "platform.h"
#include "xparameters.h"
#include "xbasic_types.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xil_exception.h"
#include "xscugic.h"

#include "Pointers.h"	// Pointers for the Processing System General IO

	// Pointers van AXI-stream FIFO
// Interrupts
//static volatile u32 *ISR =  (u32 *) 0x43C00000 ;	// interrupt status register
//static volatile u32 *IER =  (u32 *) 0x43C00004 ;	// interrupt enable register

// Transmit (Tx) uP transmits to FIFO via S_AXI
static volatile u32 *TDFR = (u32 *) 0x43C00008 ;	// Transmit Data FIFO Reset
//static volatile u32 *TDFV = (u32 *) 0x43C0000C ;	// Transmit Vacancy
static volatile u32 *TDFD = (u32 *) 0x43C00010 ;	// Transmit Data FIFO Data Write Port
static volatile u32 *TLR =  (u32 *) 0x43C00014 ;	// Transmit length register

// AXI stream reset
static volatile u32 *SRR  = (u32 *) 0x43C00028 ;	// AXI4-Stream Reset
// 32 bit destination registers
static volatile u32 *TDR =  (u32 *) 0x43C0002C ;	// Transmit Destination Register
// Interruptflags
static volatile u32 TIMER1_FLAG = 0, TIMER2_FLAG = 0, TIMER3_FLAG = 0 ;
static volatile u32 DCC_FLAG = 0;	// van DCC_IP voor 'resend' FIFO

// Eigen variabelen
int Index, i, j;
char Adres, Instr;
char DCC[18]={170,170,170,172,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	//max 138 bits=18 bytes; preamble ingevuld, adres vanaf char4 bit7 (pos 32)

////////////////////////////////
//
//		Initialisation functions
//
////////////////////////////////

static volatile u32 PacketSize_FIFO_UART  = 64 ;	// Number of bytes transferred between LabVIEW and Zybo
													//  Number of bytes =  4 * number of 32-bit values
													//  MUST BE A MULTIPLE OF 4
													//  Threshold register of UART is only 6 bit ->  Writing 64 = 0100 0000 will result in 00 0000
													//  Then the program will not use the threshold but the RX FULL flag (for interrupt)

void my_GPIO_Init(void)
{
	*PUSH_TRIS   = 0x000000FF ;   // all bits input (push buttons)
	*SLIDER_TRIS = 0x000000FF ;   // all bits input (slider switches)
	*LED_TRIS	 = 0x00000000 ;   // all bits output (LEDs)
	*SPY_TRIS 	 = 0x00000000 ;   // all output (PMod)
    *GPIO_IER    = 0x00000003 ;    // both channel interrupt

    *LED_DATA    = 0x00000000 ;   // First all LEDs off
//	SLIDER_FLAG = 0 ;
//	PUSH_FLAG = 0 ;

    if (*GPIO_ISR == 1)				// Check D0 , if 1 make 0 by TOW
    		{
    		*GPIO_ISR=  *GPIO_ISR & 0x00000001;
    		}
    if (*GPIO_ISR == 2)				// Check D1 , if 1 make 0 by TOW
    		{
    		*GPIO_ISR=  *GPIO_ISR & 0x00000002;
    		}
    if (*GPIO_ISR == 3)				// Check D1 and D0 , if 1 make 0 by TOW
    		{
    		*GPIO_ISR=  *GPIO_ISR & 0x00000003;
    		}

			*GPIO_GIER = 0x80000000  ;  //  Global Interrupt Enable
//	xil_printf (" status GPIO  =  0x%08X   \n\r", *GPIO_ISR) ;
}

void my_UART_Init(void)			// see Tech.Ref. p.584
{
	// Configure UART character frame
	*mode_reg0 = 0x00000020 ;    // D9..D8 = 00   Normal mode
								 // D7..D6 = 00   1 stop bit
								 // D5..D3 = 100  no parity
	 	 	 	 	 	 	 	 // D2..D1 = 00   8 bit
	 	 	 	 	 	 	 	 // D0 = 0   Clock = uart_ref_clk (100 Mhz)

	*Control_reg0 = 0x00000028 ;	// D5 = 1    TXDIS
									// D4 = 0    TXEN
									// D3 = 1    RXDIS
									// D2 = 0    RXEN
									// D1 = 0    TXRES
									// D0 = 0    RXRES

	// Configure the Baud Rate    115200 ---->  CD=62     BDIV=6  (Tech.Ref. p.588)
	// Configure the Baud Rate    230400 ---->  CD=31     BDIV=6
	// Configure the Baud Rate    480800 ---->  CD=9      BDIV=11
	// Configure the Baud Rate    921600 ---->  CD=9      BDIV=5

	*Baud_rate_divider_reg0 = 6 ;	// BDIV = 6 (115.200 baud/sec)
	*Baud_rate_gen_reg0 = 62 ;		// CD = 62

	*Control_reg0 = 0x0000002B ;	// D5 = 1    TXDIS
									// D4 = 0    TXEN
									// D3 = 1    RXDIS
									// D2 = 0    RXEN
									// D1 = 1    TXRES
									// D0 = 1    RXRES

	*Control_reg0 = 0x00000017 ;	// D8 = 0    STPBRK (stop break)
									// D7 = 0	 STTBRK (start break)
									// D6 = 0    RSTTO  (restart receiver timeout counter)
									// D5 = 0    TXDIS
									// D4 = 1    TXEN
									// D3 = 0    RXDIS
									// D2 = 1    RXEN
									// D1 = 0    TXRES
									// D0 = 0    RXRES

	// Set the level of the RxFIFO trigger level and Enable the Interrupt
	// Only Interrupt :  RX FIFO LEVEL
	*Rcvr_FIFO_trigger_level0 = PacketSize_FIFO_UART ;  // received number of bytes =  PacketSize_FIFO_UART  (is multiple of 4)
	*Tx_FIFO_trigger_level0 = 0 ;

	*Control_reg0 = 0x00000017	;

	if (PacketSize_FIFO_UART == 64)				// FullSize FIFO is used
		{
			*Intrpt_en_reg0	 = 	0x00000004 ;  // D2=1  RTRIG  (Interrupt : RX FIFO  IS  FULL)
			*Intrpt_dis_reg0 =	0xFFFFFFFB ;  // inverse...

			while (*Intrpt_mask_reg0 != 0x00000004)
				{
					// wait for interrupt enabled on RX FIFO FULL
				}
		}
	else
		{
			*Intrpt_en_reg0	 = 	0x00000001 ;  // D0=1  RTRIG  (Interrupt : RX FIFO AT LEVEL)
			*Intrpt_dis_reg0 =	0xFFFFFFFE ;  // inverse...

			while (*Intrpt_mask_reg0 != 0x00000001)
				{
					// wait for interrupt enabled on RTRIG
				}
		}
	*Chnl_int_sts_reg0 = 0xFFFFFFFF ;	// Clear interrupt flag  (write 1 to clear)
	*Rcvr_timeout_reg0 = 0 ;			// No time_out
	UART_FLAG = 0 ;
}

void my_ZYNQ_TTC0_Init(void)
{
	//
	//  CPU Runs at 650 MHZ (default for Zybo).  Timers run at CPU 1x ---->  CPU / 6 = 650 / 6 = 108,3333 Mhz
	//

	//////////////////////////////////////////////////////////////////////////////////////////
	//  TTC0 Timer1 is configured for interval Interrupt (interval_counter programmed in main)
	//////////////////////////////////////////////////////////////////////////////////////////

	*Clock_Control_1    = 0x0000000E ;      // D6 = 0   (n.c. external clock)
										    // D5 = 0   Internal clock
										    // D4..D1 = 0111  prescale 2**(value+1)
										    // D0     = 0   Prescale disable: clock = 108,333 MHz

	*Counter_Control_1  = 0x0000001F ;		// D6 = 0   Wave Pol: No (Out = active high)
											// D5 = 0   Wave Enable (Yes, active low)
											// D4 = 1   Reset counter

											// D3 = 1   Counter Match Interrupt
											// D2 = 1   Counter DOWN
											// D1 = 1   Counter in INTERVAL mode
											// D0 = 1   Disable Counter (yeap, counter active ---> low)

	//*Counter_Value_1    = 0x00000000 ;		// Counter Value  READ ONLY
	*Interval_Counter_1 = 0x00000876 ;  	//  16 bit LOAD value: (108,333 * 20)-1 = 2166 = 0x876 : 1 interrupt/20 usec
	*Match_1_Counter_1  = 0x00000000 ;		//  Match_1 will control the WaveOut (50%)
	*Match_2_Counter_1  = 0x00000000 ;		//  not used
	*Match_3_Counter_1  = 0x00000000 ;		//  not used

	//*Interrupt_Register_1  = 0x00000000 ;	// Status Register for the Interrupts  CLEAR ON READ !!!!!
											//  D5 : Event timer overflow
											//	D4 : Counter overflow
											//	D3 : Match3
											//	D2 : Match2
											//	D1 : Match1
											//  D0 : Interval

	dummy = *Interrupt_Register_1 ;         //  Clear on READ

	*Interrupt_Enable_1    = 0x00000001 ;	// D5 = Enable INT on Event Timer Overflow
											// D4 = Enable INT on Counter Overflow
											// D3 = Enable INT on Match3
											// D2 = Enable INT on Match2
											// D1 = Enable INT on Match1 YES
											// D0 = Enable INT on Interval YES

	*Event_Control_Timer_1 = 0x00000000 ;	// D2 = 0   disable Event Timer
											// D1 = 0	(not used)
											// D0 = 0	(not used)

	//*Event_Register_1      = 0x00000000 ;	// D1 = 0     read only
											// D0 = 0

										// And now : activate Timer ...
	*Counter_Control_1  = 0x00000016 ;	// D6 = 0   Wave Pol   Out = active high
										// D5 = 0   Wave Enable (Yes, active low)
										// D4 = 1   Reset counter

										// D3 = 0   Counter Match Interrupt
										// D2 = 1   Counter DOWN
										// D1 = 1   Counter in INTERVAL mode
										// D0 = 0   Enable Counter (yeap, counter active ---> low)


	//////////////////////////////////////////////////////////////////////////////////////////////
	//    TTC0 Timer2: NOT USED
	//////////////////////////////////////////////////////////////////////////////////////////////


	*Clock_Control_2    = 0x0000000E ;      // D6 = 0   (n.c. external clock)
										    // D5 = 0   Internal Clock
										    // D4..D1 = 1111  prescale 2**(value+1)  ---> 2**16  ---> 108Mhz/ 1,653 kHz
										    // D0     = 0   Prescale Disable

	*Counter_Control_2  = 0x00000017 ;		// D6 = 0   Wave Pol   Out = active low
											// D5 = 0   Wave Enable (yeap, active low)
											// D4 = 1   Reset counter

											// D3 = 0   Counter Match Interrupt
											// D2 = 1   Counter DOWN
											// D1 = 1   Counter in INTERVAL mode
											// D0 = 1   Disable Counter (yeap, counter active ---> low)

	//*Counter_Value_2    = 0x00000000 ;		// Counter Value  READ ONLY
	*Interval_Counter_2 = 0x00001528 ;  	//  16 bit LOAD value: (108,333 * 50)-1 = 0x1528 = 1 int. / 50 usec
	*Match_1_Counter_2  = 0x00000000 ;		//  Match_1 will control the WaveOut
	*Match_2_Counter_2  = 0x00000000 ;		//  not used
	*Match_3_Counter_2  = 0x00000000 ;		//  not used

	//*Interrupt_Register_2  = 0x00000000 ;	// Status Register for the Interrupts  CLEAR ON READ !!!!!
											//  D5 : Event timer overflow
											//	D4 : Counter overflow
											//	D3 : Match3
											//	D2 : Match2
											//	D1 : Match1
											//  D0 : Interval

	dummy = *Interrupt_Register_2 ;         //  Clear on READ

	*Interrupt_Enable_2    = 0x00000001 ;	// D5 = Enable INT on Event Timer Overflow
											// D4 = Enable INT on Counter Overflow
											// D3 = Enable INT on Match3
											// D2 = Enable INT on Match2
											// D1 = Enable INT on Match1
											// D0 = Enable INT on Interval

	*Event_Control_Timer_2 = 0x00000000 ;	// D2 = 0   disable Event Timer
											// D1 = 0	(not used)
											// D0 = 0	(not used)

	//*Event_Register_2      = 0x00000000 ;	// D1 = 0     read only
											// D0 = 0

										// And now : activate Timer ...
//	*Counter_Control_2  = 0x00000006 ;	// D6 = 0   Wave Pol   Out = active high
										// D5 = 0   Wave Enable (yeap, active low)
										// D4 = 0   Reset counter

										// D3 = 0   Counter Match Interrupt
										// D2 = 1   Counter DOWN
										// D1 = 1   Counter in INTERVAL mode
										// D0 = 0   Enable Counter (yeap, counter active ---> low)

	/////////////////////////////////////////////////////////////////////////////////////////////
	//    TTC0 Timer3: NOT USED
	/////////////////////////////////////////////////////////////////////////////////////////////

	*Clock_Control_3    = 0x00000000 ;      // D6 = 0   (n.c. external clock)
										    // D5 = 0   Internal Clock
										    // D4..D1 = 0000  prescale 2**(value+1)  ---> 2**1 = 2   108Mhz/2 = 54 MHz
										    // D0     = 0   Prescale Disable

	*Counter_Control_3  = 0x00000017 ;		// D6 = 0   Wave Pol   Out = active high
											// D5 = 0   Wave Enable (yeap, active low)
											// D4 = 1   Reset counter

											// D3 = 0   Counter Match Interrupt
											// D2 = 1   Counter DOWN
											// D1 = 1   Counter in INTERVAL mode
											// D0 = 1   Disable Counter (yeap, counter active ---> low)

	//*Counter_Value_3    = 0x00000000 ;		// Counter Value  READ ONLY
	*Interval_Counter_3 = 0x00000874 ;  	//  16 bit LOAD value: (108,33 * 20)-1 = 0x874 = 1 int. / 20 usec (=50KHz)
	*Match_1_Counter_3  = 0x00000000 ;		//  Match_1 will control the WaveOut
	*Match_2_Counter_3  = 0x00000000 ;		//  not used
	*Match_3_Counter_3  = 0x00000000 ;		//  not used

	//*Interrupt_Register_3  = 0x00000000 ;	// Status Register for the Interrupts  CLEAR ON READ !!!!!
											//  D5 : Event timer overflow
											//	D4 : Counter overflow
											//	D3 : Match3
											//	D2 : Match2
											//	D1 : Match1
											//  D0 : Interval

	dummy = *Interrupt_Register_3 ;         //  Clear on READ

	*Interrupt_Enable_3    = 0x00000001 ;	// D5 = Enable INT on Event Timer Overflow
											// D4 = Enable INT on Counter Overflow
											// D3 = Enable INT on Match3
											// D2 = Enable INT on Match2
											// D1 = Enable INT on Match1
											// D0 = Enable INT on Interval

	*Event_Control_Timer_3 = 0x00000000 ;	// D2 = 0   disable Event Timer
											// D1 = 0	(not used)
											// D0 = 0	(not used)

	//*Event_Register_3      = 0x00000000 ;	// D1 = 0     read only
											// D0 = 0

										// And now : activate Timer ...
//	*Counter_Control_3  = 0x00000016 ;	// D6 = 0   Wave Pol   Out = active high
										// D5 = 0   Wave Enable (yeap, active low)
										// D4 = 1   Reset counter

										// D3 = 0   Counter Match Interrupt
										// D2 = 1   Counter DOWN
										// D1 = 1   Counter in INTERVAL mode
										// D0 = 0   Enable Counter (yeap, counter active ---> low)

	TIMER1_FLAG = 0; TIMER2_FLAG = 0; TIMER3_FLAG = 0;
}

/////////////////////////////
//
// INTERRUPT SERVICE ROUTINES
//
/////////////////////////////

void my_GPIO_ISR (void)    //   ISR for my_push
{
	// xil_printf(" Switch change is detected  New Switch Value = 0x%08X  \n\r ", *PUSH_DATA) ;
	if (*GPIO_ISR ==1)
	{
		*GPIO_ISR = *GPIO_ISR & 0x00000001 ;		// reset Interrupt Flag on GPIO  (clear by write)
		PUSH_FLAG = 1 ;
	}
	if (*GPIO_ISR ==2)
	{
		*GPIO_ISR = *GPIO_ISR & 0x00000002 ;		// reset Interrupt Flag on GPIO (clear by write)
		SLIDER_FLAG = 1 ;
	}
}

void my_ZYNQ_Timer1_ISR(void)     //  ISR for ZYNQ TTC0 Timer1
{
	TIMER1_FLAG = *Interrupt_Register_1 & 0x00000003 ;        		// Check if ZYNQ TTC0 Timer1 generated an interrupt
	//xil_printf("Timer Interrupt \n\r");
}

void my_ZYNQ_Timer2_ISR(void)     //  ISR for ZYNQ TTC0 Timer2
{
	TIMER2_FLAG = *Interrupt_Register_2 & 0x00000003 ;        		// Check if ZYNQ TTC0 Timer2 generated an interrupt
	//xil_printf("Timer Interrupt \n\r");
}

void my_ZYNQ_Timer3_ISR(void)     //  ISR for ZYNQ TTC0 Timer3
{
	TIMER3_FLAG = *Interrupt_Register_3 & 0x00000003 ;        		// Check if ZYNQ TTC0 Timer3 generated an interrupt
	//xil_printf("Timer Interrupt \n\r");
}

void my_DCC_Gen_ISR(void)
{
	DCC_FLAG = 1 ;
}

void my_UART_RX_ISR (void)		// Interrupt Routine for the UART  RECEIVER
{
	//Xil_ExceptionDisable();
	u8  i ;

	if ( ((*Chnl_int_sts_reg0 &  0x00000001 ) == 0x00000001 ) || ((*Chnl_int_sts_reg0 &  0x00000004 ) == 0x00000004 ) )	// check for RTRIG = 1 or FIFO FULL
	{
		for (i=0;i<PacketSize_FIFO_UART;i++)
		{
			RX_BUFFER[i] = *TX_RX_FIFO0 ;      							// Read_out RX  FIFO
		}

		while  ((*Channel_sts_reg0  &  0x00000002) == 0x00000000 )      // check for REMPTY = 1
		{
			RX_BUFFER[64] = *TX_RX_FIFO0 ;      						// Dummy Read_out RX  FIFO
		}

		UART_FLAG = 1 ;
	}
	*Chnl_int_sts_reg0 = 0x000000FF ;          							// Clear Flag RTRIG (Write to Clear)
	//Xil_ExceptionEnable();
}

XScuGic InterruptController ;  // Instance of interrupt controller
static XScuGic_Config *GicConfig; // Config parameter of controller

int ScuGicInterrupt_Init()
{
	u32 Status ;
	Xil_ExceptionInit();

	GicConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID) ;

	Status = XScuGic_CfgInitialize(&InterruptController,GicConfig,GicConfig->CpuBaseAddress);
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,(Xil_ExceptionHandler) XScuGic_InterruptHandler,(void *) &InterruptController);

//	 GPIO	  Connected to IRQ_F2P(0) : lowest priority and ID = 61     IRQ_F2P(D15..D0)   connected to D0 ...)
	Status = XScuGic_Connect(&InterruptController,61,(Xil_ExceptionHandler) my_GPIO_ISR, (void *) &InterruptController) ;
	XScuGic_Enable(&InterruptController,61) ;
	XScuGic_SetPriorityTriggerType(&InterruptController,61,0xA0,0x3) ;      //0x03 = Rising Edge

//	 DCC_generator (eigen IP)  Connected to IRQ_F2P(1): ID = 62 Geen reset van interrupt nodig (IP zet zelf de INT-lijn terug laag)
	Status = XScuGic_Connect(&InterruptController,62,(Xil_ExceptionHandler) my_DCC_Gen_ISR, (void *) &InterruptController) ;
	XScuGic_Enable(&InterruptController,62) ;
	XScuGic_SetPriorityTriggerType(&InterruptController,62,0xA0,0x3) ;      //0x03 = Rising Edge

//	 ZYNQ_Timer1 Connected internally to interrupt controller: ID = 42      See p.233 Zynq Manual
	Status = XScuGic_Connect(&InterruptController,42,(Xil_ExceptionHandler) my_ZYNQ_Timer1_ISR, (void *) &InterruptController) ;
	XScuGic_Enable(&InterruptController,42) ;
	XScuGic_SetPriorityTriggerType(&InterruptController,42,0xA0,0x3) ;

//	 ZYNQ_Timer2 Connected internally to interrupt controller:  ID = 43
	Status = XScuGic_Connect(&InterruptController,43,(Xil_ExceptionHandler) my_ZYNQ_Timer2_ISR, (void *) &InterruptController) ;
	XScuGic_Enable(&InterruptController,43) ;
	XScuGic_SetPriorityTriggerType(&InterruptController,43,0xA0,0x3) ;

//	 ZYNQ_Timer3 Connected internally to interrupt controller:  ID = 44
	Status = XScuGic_Connect(&InterruptController,44,(Xil_ExceptionHandler) my_ZYNQ_Timer3_ISR, (void *) &InterruptController) ;
	XScuGic_Enable(&InterruptController,44) ;
	XScuGic_SetPriorityTriggerType(&InterruptController,44,0xA0,0x3) ;

	// UART  RECEIVER   :  ID = 82      See p.218 Zynq Manual
	Status = XScuGic_Connect(&InterruptController,82,(Xil_ExceptionHandler) my_UART_RX_ISR, (void *) &InterruptController) ;
	XScuGic_Enable(&InterruptController,82) ;
	XScuGic_SetPriorityTriggerType(&InterruptController,82,0xA0,0x3) ;

	Xil_ExceptionEnable();
	return 0 ;
}

void my_UART_TX_64(void)		// Function to transmit 64 bytes UART data
{
	u8 k ;

	for (k=0;k<(PacketSize_FIFO_UART/4);k=k+1)		// Convert 16 TX values (4 bytes) into 64 TX values (1 byte)
	{
		TX_BUFFER[(4*k)+0]  = (TX_VAL[k] >> 24) & 0x000000FF ;
		TX_BUFFER[(4*k)+1]  = (TX_VAL[k] >> 16) & 0x000000FF ;
		TX_BUFFER[(4*k)+2]  = (TX_VAL[k] >>  8) & 0x000000FF ;
		TX_BUFFER[(4*k)+3]  = (TX_VAL[k]      ) & 0x000000FF ;
	}

	for (k=0;k<PacketSize_FIFO_UART;k=k+1)	//  TX the 64 bytes
	{
		*TX_RX_FIFO0 = TX_BUFFER[k] ;
	}
}

void Adapt_Ind_j(void)
	{Index += 2; j = (j + 6) - (((j + 6)>>3)<<3);}

void PlusEen(void)
	{ DCC[Index>>3] |= (1<<j); DCC[Index>>3] &= ~(1<<(j-1)); Adapt_Ind_j(); }

void PlusNul(void)
	{ DCC[Index>>3] |= ((1<<j) | (1<<(j-1))); Adapt_Ind_j(); DCC[Index>>3] &= ~((1<<j) | (1<<(j-1))); Adapt_Ind_j(); }

void VulDCC(Adr,Ins)	// Functie die adres- en instructiebytes omzet in DCC[bits]
{
	Index = 32; j = 7;		// adres vanaf char4 bit7 (=pos 32), pramble ingevuld
	for (i=7;i>=0;i--) {	// adresbyte in DCC zetten vanaf bit 32 (eerste bit, MSB = bit 0)
		if (Adr & (1<<i)) { PlusEen(); } else { PlusNul(); }	// '10' erbij else 1100 erbij
	}
	PlusNul();				// '1100' erbij: 0 tussen adres en instructie
	for (i=7;i>=0;i--) {	// instructiebyte erachter zetten
		if (Ins & (1<<i)) { PlusEen(); } else { PlusNul(); }
	}
	PlusNul();				// 0 tussen instructie en controle
	for (i=7;i>=0;i--) {	// controlebyte erachter zetten
		if ((Adr & (1<<i)) ^ (Ins & (1<<i))) { PlusEen(); } else { PlusNul(); }	// exor van adresbit en instructiebit
	}
	PlusEen();				// '10' erbij: packet end bit=1
	// in (char) DDC[] is de pointer naar 1e bit= 0, naar laatste bit= (Index-1), aantal te laden 32bit-FIFOwoorden = Index
}

void Zend_DCC(void)		// Zenden vd bits in DCC-buffer = FIFO laden & transmissie starten
{
    *TDFR = 0x000000A5;	// Reset Transmit Data FIFO reg.
	*TDFD = 0x40000001;	// eerste 32 bit woord: bit 30 = 1 en bit 0 = 1 (eerste "1" van preamble)
	for ( i=1;i<(Index-1);i++) { // volgende woorden: bitje uit DCC[] met Index=i
		if (DCC[i>>3] & (1<<(7 - (i - ((i>>3)<<3))))) { *TDFD = 0x00000001; } else { *TDFD = 0x00000000; }
	}
	*TDFD = 0x80000000;	// laatste woord: bit31 = 1, bit 0 is steeds 0 gezien packet end = 1 of dus 2 bits='10' > laatste bit=0!
	*TLR = Index * 4;	// Transmit lengte in bytes: start transmissie
}

int main()
{
	init_platform();
	my_GPIO_Init() ;					// Init GPIO
	my_UART_Init() ;					// Init UART1
//	my_ZYNQ_TTC0_Init();				// Init ZYNQ Timers (Timers hier niet gebruikt)
	dummy = ScuGicInterrupt_Init();		// Init Interrupts
	u8 m = 1, n, t, AccAdres = 0;
//  Transmit Destination address:
	*TDR  = 0x00000002;	// destination device address: 2
	*SRR  = 0x000000A5;	// reset AXI4 stream
	*SPY_DATA = 0x00000000 ;	// zet Pmod JC lijnen = 0
//	DCC-UITGANG AANGESLOTEN OP Pmod JB 1 (DCC) en 2 (not DCC)

	VulDCC(0xFF, 0); Zend_DCC();	// Zend Idle packets

	while(1)
	{
		if (UART_FLAG)	// INT van LabVIEW Station: 64 bytes ontvangen in RX_BUFFER[]
		{
			for (n=0;n<PacketSize_FIFO_UART;n++) { *TX_RX_FIFO0 = RX_BUFFER[n]; }	// transmit RX-buffer back to LabVIEW (handshake)
			if (((RX_BUFFER[0] << 24) + (RX_BUFFER[1] << 16) + (RX_BUFFER[2] << 8) + RX_BUFFER[3] == 0xC000FFFF) &&		// eerste &
				((RX_BUFFER[60] << 24) + (RX_BUFFER[61] << 16) + (RX_BUFFER[62] << 8) + RX_BUFFER[63] == 0xAFAFAFAF))	// laatste woord OK?
			{
				Adres = RX_BUFFER[7]; Instr = RX_BUFFER[11];
				if (((Adres & 0x80) > 0) && Adres != 0xFF) { AccAdres = 1; }
				else {  AccAdres = 0; }	// indien geen idle packet en bit 7 van Adres is 1 => Accessory Decoder adres
				VulDCC(Adres, Instr); Zend_DCC();	// Vul de FIFO en start DCC transmissie
			}
			UART_FLAG = 0;
		}
		if (DCC_FLAG)	// interrupt van DCC_generator: packet terug opsturen behalve Accessory Decoder packet
		{
			if (AccAdres) { Adres = 0xFF; Instr = 0; VulDCC(Adres, Instr); Zend_DCC(); }	// Zend idle packets
			else
			{
				t ++; if (!(t % 47)) {*LED_DATA = m; m = (m << 1); if (m == 16) {m = 1;} }	// LEDs circulerend aan en uit
				Zend_DCC();		// herhaal de 'laad FIFO en zend' procedure
				DCC_FLAG = 0;	// reset DCC_Gen interrupt flag
			}
		}
		if (PUSH_FLAG)	// Accessory decoder packet 1 x opsturen, daarna idle packets
		{
			if (*PUSH_DATA)		// indien een knop wordt ingedrukt
			{
				Adres = 0x84;	// Accessory adres = 4 voor Arduino decoder met 4 relais
				if (*PUSH_DATA == 1) { Instr = 0xF0; }	// relais sturing is active low!
				if (*PUSH_DATA == 2) { Instr = 0xF2; }
				if (*PUSH_DATA == 4) { Instr = 0xF4; }
				if (*PUSH_DATA == 8) { Instr = 0xF6; }
			}
			else { Instr |= 0x01; }	// zet relais af wanneer knop wordt losgelaten (dan is *PUSH_DATA = 0)
			VulDCC(Adres, Instr); Zend_DCC();
			PUSH_FLAG = 0;
		}
	}
	return 0;
}
