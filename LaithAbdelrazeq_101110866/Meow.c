#include "msp.h"

/**
The stateChange function is responsible for togglin the state of the LED's on the board. 
sateChange takes the parameter 1 for "next"
stateChange takes the parameter 2 for "previous"
If we are at the state 3 and press next, we go to state 0
If we are in the state 0 and press previous, we go to state 3 
**/
void stateChange(int value) {
	static uint8_t state = (uint8_t)0x30;	//This sets the state to "0" in ASCI
	//if we are at state 0 and we go previous, go to state 3
	if (state == (uint8_t)0x30 && value == 2) {	
		state = (uint8_t)0x33;
	}
	//if we are at state 4 and we go next, go to state 0
	else if (state == (uint8_t)0x33 && value == 1) { 
		state = (uint8_t)0x30;
	}
	//Go to next state
	else if (state < (uint8_t)0x33 && value == 1) {
		state++;
	}
	//Go to previous state
	else if (state > (uint8_t)0x30 && value == 2) {
		state--;
	}
	
	// Managing state switch statement
	switch(state){
	
		case 0x30:	//state 0
			P1OUT &= (uint8_t)~0x01;	//Clear LEDs
			P2OUT &= (uint8_t)~0x04;	//Clear LEDs
			break;
		
		case 0x31: //state 1
			P2OUT &= (uint8_t)~0x04;	//Clear LEDs
			P1OUT |= (uint8_t)0x01;	//only first LED on
			break;
		
		case 0x32:	//state 2
			P1OUT &= (uint8_t)~0x01;	//Clear LEDs
			P2OUT |= (uint8_t)0x04;	//only second LED on
			break;
		
		case 0x33:	//state 3
			P1OUT |= (uint8_t)0x01;	// turn on first LED
			P2OUT |= (uint8_t)0x04;	// turn on first LED
			break;	
	}
	EUSCI_A0->TXBUF = state;	// Print/recieve/show the current state
}
// UART Interrupt Service Routine
void EUSCIA0_IRQHandler(void) {
    static uint8_t Rx;
	
		if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
				
				Rx = EUSCI_A0->RXBUF;
				if (Rx == (uint8_t)0x2E) {  //0x2E ASCI = ". or >"
					stateChange(1);	//Go to next State
				}
				else if (Rx == (uint8_t)0x2C) {	//0x2C ASCI = ", or <"
					stateChange(2);	//Go to previous State
				}
		}
}

// Port1 Interrupt Service Routine
void PORT1_IRQHandler(void) {
	// Go to the next state
	if((P1IFG & 0x10)) {	//check if flag on P1.4
		P1IFG &= (~0x10);	//clearing the interrupt flag
		stateChange(1);	//stateChange to next
		}
	
	// Go to previous state
	else if((P1IFG & (uint8_t)0x02)) { //check if the flag on P1.1
		P1IFG &= (~(uint8_t)0x02);	//clearing the interrupt flag
			stateChange(2);	//stateChange to previous
	}
}


// main function
int main(void) {
	
	/**
Configuring the UART
**/
	CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 826MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
    EUSCI_A0->BRW = 78;                     // 12000000/16/9600
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt

    // Enable sleep on exit from ISR
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
//////////////////////////////////////////////////////////////////////////////////////////////////

	/**
Configuring the GPIO Pins
**/
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;	//Stop watchdog timer
	
	//Setting up ports one and two as GPIO, and initializing pins
	P1SEL0 &=(uint8_t)(~((0x10) | (0x02) | (0x01)));
	P1SEL1 &=(uint8_t)(~((0x10) | (0x02) | (0x01)));
	
	P2SEL0 &=(uint8_t)~((0x01)|(0x04));
	P2SEL1 &=(uint8_t)~((0x01)|(0x04));
	
	P1DIR &=(uint8_t)(~((0x10) | (0x02)));
	P1DIR |=(uint8_t)(0x01);
	P2DIR |=(uint8_t)((0x01)|(0x04));
	
	P1OUT &=~(uint8_t)(0x01);
	P1OUT |=(uint8_t)((0x10) | (0x02));
	P2OUT &=~(uint8_t)((0x01)|(0x04));
	
	P1REN |=(uint8_t)((0x10) | (0x02));
	
	//Interupts Configuration
	P1IES |= (uint8_t)((0x10)|(0x02));
	P1IFG &= (uint8_t)~((0x10)|(0x02));
	P1IE |= (uint8_t)((0x10)|(0x02));
	
	//NVIC config
	NVIC_SetPriority(PORT1_IRQn, 2);
	NVIC_ClearPendingIRQ(PORT1_IRQn);
	NVIC_EnableIRQ(PORT1_IRQn);
	
	__ASM("CPSIE I");	//globally enable interrupts
	
  // Enter LPM0
  __sleep();
  __no_operation();       // For debugger
}
