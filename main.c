//*****************************************************************
// A custom device for transmitting/recieving CAN traffic
// Designed to work with CanHacker v2
// You are free to use this code at your own discretion
// Alex Svetlichnyy 2019
//*****************************************************************
#include "stm32f10x.h"
#include "main.h"

#define CAN_LOOP_BUFFER_SIZE 555 															 // The loop buffer used to store recieved CAN-traffic and process it in between receptions

volatile uint32_t TimingDelay;    														 // The value for Systick delay
																															 // The loop buffer used to store recieved CAN-traffic
static Std_Can_Message_Type CanRxBuffer[CAN_LOOP_BUFFER_SIZE]; // and process it in between receptions
static uint32_t CanRxBufferWriteCounter=0; 										 // The loop buffer write index
static uint32_t CanRxBufferReadCounter=0;											 // The loop buffer read index		
 volatile uint8_t LawicelBuffer[LAWICEL_COMMAND_BUFFER_SIZE];   // This buffer is used to store and process  Lawicel TX commands from the host
 volatile uint8_t LawicelCommandCounter;												 // This value counts the bytes recieved within a TX command from CAN-Hacker 
//static uint8_t CanRxMessage_For_Usart[33];										 // If not using CanHacker this buffer contains a CAN-Message data to send via USART


// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
static Std_Can_Message_Type GateWayCanRxBuffer[CAN_LOOP_BUFFER_SIZE]; 
static uint32_t GateWayCanRxBufferWriteCounter=0; 										 
static uint32_t GateWayCanRxBufferReadCounter=0;											 		
volatile uint8_t GateWayLawicelBuffer[LAWICEL_COMMAND_BUFFER_SIZE];  
volatile uint8_t GateWayLawicelCommandCounter;	
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************



/* Setting up a systick interrupt. It counts from a delay value to zero*/
void SysTick_Handler(void) {   
	if (TimingDelay){
		TimingDelay--;
  }      
}
/* Setting up a delay value*/
void Delay_mS(uint32_t nTime){
	TimingDelay = nTime;
	while (TimingDelay);
 }

 
 
 
 /******************LOOP BUFFER BEGINNING**************************/
 /******************LOOP BUFFER BEGINNING**************************/
 /******************LOOP BUFFER BEGINNING**************************/
 
/* Check the loop buffer, if it is empty */
uint32_t isCanRxBufferEmpty(){
	if(CanRxBufferReadCounter==CanRxBufferWriteCounter){
		return 0; //empty
		}
	else if(CanRxBufferReadCounter!=CanRxBufferWriteCounter){
		return 1; //not empty
		}
	return 0; 
}

/* Get next counter for a loop buffer index */	
static uint32_t GetNextCount(uint32_t currentCount){
	uint32_t tmp;
	tmp = (currentCount + 1);
	if (tmp > CAN_LOOP_BUFFER_SIZE){
		tmp = 0;
	};
	return tmp;
}

/* Write a new message to the loop Buffer */
void CanRxBufferWrite(Std_Can_Message_Type rxData){ 
	// CanRxBuffer[CanRxBufferWriteCounter] = rxData;	
	uint32_t tmp = GetNextCount(CanRxBufferWriteCounter); 
	// If the write counter catches up to the read counter this causes buffer overflow
	if (tmp == CanRxBufferReadCounter){ // Buffer is full
//		HandleProgrammError(ERR_CAN_LOOP_BUFFER_OVERFLOW); 
	//rewrite last available slot if the buffer is full, keep the writecounter in the same position
	} 
	else {
		CanRxBuffer[CanRxBufferWriteCounter] = rxData;	
		CanRxBufferWriteCounter = tmp; 
	}
}
/* Read the last unread message from the loop Buffer */	
Std_Can_Message_Type CanRxBufferRead(void){
	if(CanRxBufferReadCounter==CanRxBufferWriteCounter){ //Buffer is empty
		Std_Can_Message_Type nullMessage ={};		
		return nullMessage; //Return null if buffer is empty
	}
	Std_Can_Message_Type tmpData = CanRxBuffer[CanRxBufferReadCounter];
	CanRxBufferReadCounter = GetNextCount(CanRxBufferReadCounter);
	return tmpData;
}	

 /******************LOOP BUFFER END**************************/
 /******************LOOP BUFFER END**************************/
 /******************LOOP BUFFER END**************************/




// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************

uint32_t isGateWayCanRxBufferEmpty(){
	if(GateWayCanRxBufferReadCounter==GateWayCanRxBufferWriteCounter){
		return 0; //empty
		}
	else if(GateWayCanRxBufferReadCounter!=GateWayCanRxBufferWriteCounter){
		return 1; //not empty
		}
	return 0; 
}

/* Write a new message to the loop Buffer */
void GateWayCanRxBufferWrite(Std_Can_Message_Type rxData){ 
	GateWayCanRxBuffer[GateWayCanRxBufferWriteCounter] = rxData;	
	uint32_t tmp = GetNextCount(GateWayCanRxBufferWriteCounter); 
	// If the write counter catches up to the read counter this causes buffer overflow
	if (tmp == GateWayCanRxBufferReadCounter){ // Buffer is full
	//	HandleProgrammError(ERR_CAN_LOOP_BUFFER_OVERFLOW); 
	//rewrite last available slot if the buffer is full, keep the writecounter in the same position
	} 
	else {
		GateWayCanRxBufferWriteCounter = tmp; 
	}
}

Std_Can_Message_Type GateWayCanRxBufferRead(void){
	if(GateWayCanRxBufferReadCounter==GateWayCanRxBufferWriteCounter){ //Buffer is empty
		Std_Can_Message_Type nullMessage ={};		
		return nullMessage; //Return null if buffer is empty
	}
	Std_Can_Message_Type tmpData = GateWayCanRxBuffer[GateWayCanRxBufferReadCounter];
	GateWayCanRxBufferReadCounter = GetNextCount(GateWayCanRxBufferReadCounter);
	return tmpData;
}	
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************









 /******************INTERRUPTS BEGINNING**************************/
 /******************INTERRUPTS BEGINNING**************************/
 /******************INTERRUPTS BEGINNING**************************/


/* Setting up a CAN RX interrupt. Check for FIFO0 not empty event*/	
void USB_LP_CAN1_RX0_IRQHandler(void) {
	if(CAN1->RF0R & CAN_RF0R_FMP0){
		// Recieve a new message
		Std_Can_Message_Type newMessage = Can1_Recieve_StdMessage(CAN_FIFO_0);
		// And write it to the loop buffer
		if(!(CAN1->sFIFOMailBox[CAN_FIFO_0].RIR & CAN_RI0R_IDE)){         // if id is standart
		CanRxBufferWrite(newMessage);
		}
// CAN_RF0R_FMP0 bit is cleared by hardware. 
	}
}

/* Setting up an USART1 RX interrupt. Check if USART1 buffer is not empty*/		
void USART1_IRQHandler(void){
	uint8_t recievedByte;
	if (USART1->SR & USART_SR_RXNE){
//		__disable_irq();
		recievedByte = (USART1->DR); 					// Recieve a byte
		Lawicel_ProcessCommand(&recievedByte, GATEWAY_MODE_OFF);// Process a recieved data		
//		__enable_irq();
	}	
}


 //TIM1 Update Interrupt
void TIM2_IRQHandler(void){
	if(TIM2->SR & TIM_SR_UIF){
	TIM2->CNT = 0;
	TIM2->SR &= ~TIM_SR_UIF; // Flag down
	}
}	
	
 /******************INTERRUPTS END**************************/
 /******************INTERRUPTS END**************************/
 /******************INTERRUPTS END**************************/




/******************TEST AREA**************************/
/* This function is designed to test CAN transmisson by sending CAN data via USART*/	
/*
 void Set_Can1Message_for_Usart(Std_Can_Message_Type RxMsg){
	CanRxMessage_For_Usart[0]=halfbyte_to_hexascii(RxMsg.id_highbyte>> 4);
	CanRxMessage_For_Usart[1]=halfbyte_to_hexascii(RxMsg.id_highbyte);
	CanRxMessage_For_Usart[2]=halfbyte_to_hexascii(RxMsg.id_lowbyte>> 4);
	CanRxMessage_For_Usart[3]=halfbyte_to_hexascii(RxMsg.id_lowbyte);
	CanRxMessage_For_Usart[4]=' ';			
	CanRxMessage_For_Usart[5]=halfbyte_to_hexascii(RxMsg.dlc>> 4);
	CanRxMessage_For_Usart[6]=halfbyte_to_hexascii(RxMsg.dlc);
	CanRxMessage_For_Usart[7]=' ';		
	CanRxMessage_For_Usart[8]=halfbyte_to_hexascii(RxMsg.data[7]>> 4);
	CanRxMessage_For_Usart[9]=halfbyte_to_hexascii(RxMsg.data[7]);
	CanRxMessage_For_Usart[10]='.';
	CanRxMessage_For_Usart[11]=halfbyte_to_hexascii(RxMsg.data[6]>> 4);
	CanRxMessage_For_Usart[12]=halfbyte_to_hexascii(RxMsg.data[6]);
	CanRxMessage_For_Usart[13]='.';
	CanRxMessage_For_Usart[14]=halfbyte_to_hexascii(RxMsg.data[5]>> 4);
	CanRxMessage_For_Usart[15]=halfbyte_to_hexascii(RxMsg.data[5]);
	CanRxMessage_For_Usart[16]='.';
	CanRxMessage_For_Usart[17]=halfbyte_to_hexascii(RxMsg.data[4]>> 4);
	CanRxMessage_For_Usart[18]=halfbyte_to_hexascii(RxMsg.data[4]);
	CanRxMessage_For_Usart[19]='.';
	CanRxMessage_For_Usart[20]=halfbyte_to_hexascii(RxMsg.data[3]>> 4);
	CanRxMessage_For_Usart[21]=halfbyte_to_hexascii(RxMsg.data[3]);
	CanRxMessage_For_Usart[22]='.';
	CanRxMessage_For_Usart[23]=halfbyte_to_hexascii(RxMsg.data[2]>> 4);
	CanRxMessage_For_Usart[24]=halfbyte_to_hexascii(RxMsg.data[2]);
	CanRxMessage_For_Usart[25]='.';
	CanRxMessage_For_Usart[26]=halfbyte_to_hexascii(RxMsg.data[1]>> 4);
	CanRxMessage_For_Usart[27]=halfbyte_to_hexascii(RxMsg.data[1]);
	CanRxMessage_For_Usart[28]='.';
	CanRxMessage_For_Usart[29]=halfbyte_to_hexascii(RxMsg.data[0]>> 4);
	CanRxMessage_For_Usart[30]=halfbyte_to_hexascii(RxMsg.data[0]);
	CanRxMessage_For_Usart[31]='*';
	CanRxMessage_For_Usart[32]='*';
}
*/
/******************TEST AREA ENDS**************************/


int main(void){
	Init_IWDG(500);
	ClockInit();   							// Start HSE, PLL, Flash latency, all the RCC configuration
	EnablePeripherals(); 				// Enable all the Peripherial clocks
	SysTick_Config(72000-1); 		// interrupt in 10 states
	

	Usart1PinConfig(REMAP_USART1_RX_TX_PORTS, USART1_CTS_RTS_DISABLED); // Configure USART pins
	Usart1Config(USART1_IRQ_ON, USART1_DMA_ON, BAUDRATE_APB1_36_115200);			// Configure USART peripherals
	Usart1Start(USART1_IRQ_ON);																					// Start USART with its interrupts enabled
	


	NVIC_EnableIRQ (USART1_IRQn);						// Enable Interrupts 
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
//	NVIC_EnableIRQ(TIM2_IRQn);
	
	__enable_irq ();		


	Can1_Initializate(NO_REMAP_CAN1, CAN_BAUDRATE_500KB);  
	Can1_InitializateFilters();	
	


// ***************************************** STB PIN CONTROL **********************************
GPIOB->CRL	&= ~GPIO_CRL_CNF3;						// 
GPIOB->CRL 	|= GPIO_CRL_MODE3_0;					//
GPIOB->CRL 	|= GPIO_CRL_MODE3_1;
GPIOB->BSRR |= GPIO_BSRR_BR3;						// 

GPIOB->CRL	&= ~GPIO_CRL_CNF4;						// 
GPIOB->CRL 	|= GPIO_CRL_MODE4_0;					//
GPIOB->CRL 	|= GPIO_CRL_MODE4_1;
GPIOB->BSRR |= GPIO_BSRR_BR4;						// 


// ***************************************** STB PIN CONTROL **********************************

// StartBasicTimer(32000, 60000, TIMER2);



	// ***************************************** TEST LED **********************************
GPIOC->CRH	&= ~GPIO_CRH_CNF13;	// Сбрасываем биты CNF для бита 5. Режим 00 - Push-Pull 
GPIOC->CRH 	|= GPIO_CRH_MODE13_0;	// Выставляем бит MODE0 для 13 пина. Режим MODE01 = Max Speed 10MHz
GPIOC->CRH 	|= GPIO_CRH_MODE13_0; 	
	

		GPIOC->BSRR |= GPIO_BSRR_BR13;		// Сбросили бит. 
	Delay_mS(50);
		GPIOC->BSRR |= GPIO_BSRR_BS13;		// Сбросили бит. 
	Delay_mS(50);
	
// ***************************************** TEST LED **********************************





// ***************************************** TEST START MESSAGES **********************************
	// Send 50 different messages to CAN bus in order to test the device and network
/*
		static Std_Can_Message_Type startTestMessage;	
		for(int i =0;i<50;i++){
		startTestMessage.id_highbyte=0;
		startTestMessage.id_lowbyte=i;
		startTestMessage.dlc=8;
		startTestMessage.data[0]=i;
		startTestMessage.data[1]=i;
		startTestMessage.data[2]=i;
		startTestMessage.data[3]=i;
		startTestMessage.data[4]=i;
		startTestMessage.data[5]=i;
		startTestMessage.data[6]=i;
		startTestMessage.data[7]=i;
		Can1_Transmit_StdMessage(startTestMessage);
	}
*/
// ***************************************** TEST START MESSAGES **********************************
	
	
	// In the main loop check for new messages (written to the loop buffer)
	// If there are any, send them via USART
	
	uint32_t bufferStatus;
	Std_Can_Message_Type RxMsg;

// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
//	uint32_t GateWaybufferStatus;		
//	Std_Can_Message_Type GateWayRxMsg;
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************
// ******************** GATEWAY MODE ONLY ***************************************	

//				uint32_t x=0x00;
	while(1){
		IWDG_reset();	
		bufferStatus = isCanRxBufferEmpty();

//	uint32_t injection_delay = 10; //injection delay - 12ms
			if(bufferStatus!=0){
	//			__disable_irq();
				RxMsg = CanRxBufferRead();	
						//					Delay_mS(injection_delay);

								Send_Std_LawicelMessage(RxMsg, LAWICEL_DESTINATION_CANHACKER);

					
//				Dma1Usart1_SendByteArray(&RxMsg.dlc,1);


					

			}
	//		__enable_irq();
							
	}
}
