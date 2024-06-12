
#include "stm32f4xx.h"

//-----------Main---------

int main(void) //__main loop()
{
  settingup();			
  __asm("SVC 0XF3");	
	while(1)
	{
		__asm("nop"); //Never
	}
}//Main closes


//------------------------Functions-------------------------------
void Systick_initialize (void) 
{
	SysTick ->CTRL = 0; 
	SysTick->LOAD = (16000000 - 1); 
	SysTick->VAL = 0; 
	SysTick->CTRL |= (1<<0 | 1<<1 | 1<<2) ;
	NVIC_SetPriorityGrouping(3);
	SCB->SHP[11]=0x40; 
} //Systick Init closes


__asm void settingup(void)			//to setup PSP, Unprivelleged mode and thread mode
{
	LDR R1,=0X20000200
	MSR PSP,R1
	MOV R0,#3
	MSR CONTROL,R0
} //Setup func closes

volatile static uint32_t Task1_Counter=0;

void extern Task1_main(void)
{
	while(1)
		{
		Task1_Counter++;
		__ASM("wfi");
	}
}//T1main

volatile static uint32_t Task2_Counter=0;

void Task2_main(void)
{
	while(1)
		{
		Task2_Counter++;
		__ASM("wfi");
	}
}//T2main

volatile static uint32_t Task3_Counter=0;

void Task3_main(void)
{
	while(1)
		{
		Task3_Counter++;
		__ASM("wfi");
	}
}//T3 main

volatile static uint32_t Task4_Counter=0;

void Task4_main(void)
{
	while(1)
		{
		Task4_Counter++;
		__ASM("wfi");
	}
}//T4main

volatile static uint32_t Task5_Counter=0;

void Task5_main(void)
{
	while(1)
		{
		Task5_Counter++;
		__ASM("wfi");
	}
}//T5Main


volatile static uint8_t svc_number; //to hold SVC_number
volatile static uint32_t TaskPsPs[5]={0X2000A000,0X2000B000,0X2000C000,0X2000D000,0X2000E000}; //PSP array
volatile static uint32_t SPAdd=0x00,NewPSP=0x00;

__asm void SVC_HandlerAsm(void){ 
	// OS initialisation SVC 0XF3
	
			push {lr}
			TST LR,#4 
			ITE EQ
			MRSEQ R0,MSP
			MRSNE R0,PSP
			LDR R0,[R0,#24]
			LDRB R0,[R0,#-2]
			LDR R7, =svc_number
			STR R0,[R7]
	}//SVC handler ASM closes


	
//--------------TASK -1---------------			
__asm void Task1(void){
		LDR R1,=0X2000A000
		LDR R0,=0x00000000
		STR R0,[R1],#4  //R0
		STR R0,[R1],#4  //R1
		STR R0,[R1],#4  //R2
		STR R0,[R1],#4  //R3
		STR R0,[R1],#4  //R12
		LDR R0,=0xFFFFFFFF
		STR R0,[R1],#4  //LR
		LDR R0, =Task1_main
		STR R0,[R1],#4  //PC
		LDR R0,=0x01000000
		STR R0,[R1],#4  //xPSR	
}// Task1
	
//-----------------TASK2--------------------
__asm void Task2(void){
		LDR R1,=0X2000B000
		LDR R0,=0x00000000
		STR R0,[R1],#4  //R0
		STR R0,[R1],#4  //R1
		STR R0,[R1],#4  //R2
		STR R0,[R1],#4  //R3
		STR R0,[R1],#4  //R12
		LDR R0,=0xFFFFFFFF
		STR R0,[R1],#4  //LR
		LDR R0, =Task2_main
		STR R0,[R1],#4  //PC
		LDR R0,=0x01000000
		STR R0,[R1],#4  //xPSR
}//Task2

//-------------TASK3----------------
__asm void Task3(void){
		LDR R1,=0X2000C000
		LDR R0,=0x00000000
		STR R0,[R1],#4  //R0
		STR R0,[R1],#4  //R1
		STR R0,[R1],#4  //R2
		STR R0,[R1],#4  //R3
		STR R0,[R1],#4  //R12
		LDR R0,=0xFFFFFFFF
		STR R0,[R1],#4  //LR
		LDR R0, =Task3_main
		STR R0,[R1],#4  //PC
		LDR R0,=0x01000000
		STR R0,[R1],#4  //xPSR
}//Task3

//--------------TASK4--------------------

__asm void Task4(void){
		LDR R1,=0X2000D000
		LDR R0,=0x00000000
		STR R0,[R1],#4  //R0
		STR R0,[R1],#4  //R1
		STR R0,[R1],#4  //R2
		STR R0,[R1],#4  //R3
		STR R0,[R1],#4  //R12
		LDR R0,=0xFFFFFFFF
		STR R0,[R1],#4  //LR
		LDR R0, =Task4_main
		STR R0,[R1],#4  //PC
		LDR R0,=0x01000000
		STR R0,[R1],#4  //xPSR
}//Task4

//--------------TASK5--------------------

__asm void Task5(void){
		LDR R1,=0X2000E000
		LDR R0,=0x00000000
		STR R0,[R1],#4  //R0
		STR R0,[R1],#4  //R1
		STR R0,[R1],#4  //R2
		STR R0,[R1],#4  //R3
		STR R0,[R1],#4  //R12
		LDR R0,=0xFFFFFFFF
		STR R0,[R1],#4  //LR
		LDR R0, =Task5_main
		STR R0,[R1],#4  //PC
		LDR R0,=0x01000000
		STR R0,[R1],#4  //xPSR
}//Task5

__asm void pop(void)
{
	pop {pc}
}

void SVC_Handler(void){
	SVC_HandlerAsm();
	if(svc_number==0XF3) //SVC number is 0XF3 do the OS initialisation here
		{
		
		Task1();		
		Task2();
		Task3();
		Task4();
		Task5();
		
		Systick_initialize(); 
		
		SCB->SHP[10]=0x60; 
			
		NVIC_SetPriority(TIM4_IRQn,5); //TIM4
		NVIC->ISER[0]|=(1<<30);
	
		pop(); 
	}	//if closes
	else
	{
		pop(); 
	}	//else closes
}//SVC handler closes

volatile static uint32_t ticks=0; 

void SysTick_Handler(void)
{
	ticks++;
	if(ticks%5==0) 
	{	
		SCB->ICSR|=SCB_ICSR_PENDSVSET_Msk;	
	}
	if(ticks%15==0) 
	{
		ticks=0;
		NVIC->ISPR[0]|=(1<<30); 
		__asm("nop");
	}
}//Systick Handler

volatile static uint32_t TaskNum=(uint32_t)-1; 

__asm void updatePSP(uint32_t Upsp)
	{
	LDR R1, =Upsp
	MRS R2,PSP
	STR R2,[R1]
}//UpdatePSP 

__asm void NextPsP(uint32_t Npsp)
	{	
	LDR R1, =Npsp;
	LDR R1,[R1];
	MSR PSP,R1;
}// NextPSP

void PendSV_Handler(void) //Rround robin
{
	if(TaskNum!=(uint32_t)-1)
		{ 
			updatePSP(NewPSP);
			TaskPsPs[TaskNum]=NewPSP; // Update the PSP array
	}//if closes
	
	TaskNum=(TaskNum+1)%5; 
	SPAdd=TaskPsPs[TaskNum]; // next context ka PSP
	NextPsP(SPAdd);
}//pendsv closes	

static volatile uint32_t NoOfInterruptsThrown=0x00;

void TIM4_IRQHandler (void)   //Timer interrupt
{ 
	NoOfInterruptsThrown++;
	__asm("nop");
} //TIM4 IRQ closes

