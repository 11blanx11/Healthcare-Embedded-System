// Two interrupt sources 
#include<stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

#define DUMMY_XPSR  0x01000000U
#define MAX_TASKS   5
#define TASK_READY_STATE  0x00
#define TASK_BLOCKED_STATE  0XFF

#define SIZE_TASK_STACK          1024U
#define SIZE_SCHED_STACK         1024U

#define SRAM_START               0x20000000U
#define SIZE_SRAM                ( (128) * (1024))
#define SRAM_END                 ((SRAM_START) + (SIZE_SRAM) )

#define T1_STACK_START           SRAM_END
#define T2_STACK_START           ( (SRAM_END) - (1 * SIZE_TASK_STACK) )
#define T3_STACK_START           ( (SRAM_END) - (2 * SIZE_TASK_STACK) )
#define T4_STACK_START           ( (SRAM_END) - (3 * SIZE_TASK_STACK) )
#define T5_STACK_START         ( (SRAM_END) - (4 * SIZE_TASK_STACK) )
#define SCHED_STACK_START        ( (SRAM_END) - (5 * SIZE_TASK_STACK) )


volatile uint32_t ticks = 0;
volatile uint32_t PendSV_count=0;
void timer_init(void);
void TimerDelay(uint32_t ms);

//irq handlers
void NonMaskableInt_IRQHandler(void);
void TIM4_IRQHandler(void);

/* task handler function prototypes */
void task1_handler(void); //This is task1
void task2_handler(void); //this is task2
void task3_handler(void); //this is task3
void task4_handler(void); 
void task5_handler(void);// this is task4 of the application

void Systick_handler(void);
void SVC_Handler(void);
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
void Systick_initialize (void)
{
uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (1680000)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter//enable clock source, interrupt, enable systick
	
	
}
/* This variable tracks the current_task being executed on the CPU */
uint8_t current_task = 0; //task1 is running

/* This variable gets updated from systick handler for every systick interrupt */
uint32_t g_tick_count = 0;
void timer_init(void)
{
	//Enable clock values and TIM4 clock
	RCC->CR |= (1<<0);
	RCC->CFGR &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3);
	RCC->APB2ENR |= (1<<14);
	RCC->APB1ENR |= (1<<2);
	
	TIM4->PSC = 167;
	TIM4->ARR = 1000;
	TIM4->CR1 |= ( 1<<1 | 1<<2 | 1<<4);
	TIM4->DIER |= (1<<0);
	TIM4->EGR |= (1<<0);
	NVIC_EnableIRQ(TIM4_IRQn);
}
void TimerDelay(uint32_t ms)
{
		TIM4->CR1 |= (1<<0);
		ticks = 0;
		while(ticks<ms);
		NVIC->STIR |= (uint32_t)NonMaskableInt_IRQn; //sets software interrupt
		TIM4->CR1 &= ~(1<<0);
}
typedef struct
{
	uint32_t psp_value;
	uint32_t block_count;
	uint8_t  current_state;
	void (*task_handler)(void);
}TCB_t;

/* Each task has its own TCB */
TCB_t user_tasks[5];
void init_tasks_stack(void)
{

	user_tasks[0].current_state = TASK_READY_STATE;
	user_tasks[1].current_state = TASK_BLOCKED_STATE;
	user_tasks[2].current_state = TASK_BLOCKED_STATE;
	user_tasks[3].current_state = TASK_BLOCKED_STATE;
	user_tasks[4].current_state = TASK_BLOCKED_STATE;

	user_tasks[0].psp_value = T1_STACK_START;
	user_tasks[1].psp_value = T2_STACK_START;
	user_tasks[2].psp_value = T3_STACK_START;
	user_tasks[3].psp_value = T4_STACK_START;
	user_tasks[4].psp_value = T5_STACK_START;

	user_tasks[0].task_handler = task1_handler;
	user_tasks[1].task_handler = task2_handler;
	user_tasks[2].task_handler = task3_handler;
	user_tasks[3].task_handler = task4_handler;
	user_tasks[4].task_handler = task5_handler;
	


uint32_t *pPSP;
	int i;
	for(i = 0 ; i < MAX_TASKS ;i++)
	{
		pPSP = (uint32_t*) user_tasks[i].psp_value;

		pPSP--;
		*pPSP = DUMMY_XPSR;//0x01000000

		pPSP--; //PC
		*pPSP = (uint32_t) user_tasks[i].task_handler;

		pPSP--; //LR
		*pPSP = 0xFFFFFFFD;
		int j;
		for(j = 0 ; j < 13 ; j++)
		{
			pPSP--;
		    *pPSP = 0;

		}

		user_tasks[i].psp_value = (uint32_t)pPSP;


	}
}
uint32_t get_psp_value(void)
{

	return user_tasks[current_task].psp_value;
}


void save_psp_value(uint32_t current_psp_value)
{
	user_tasks[current_task].psp_value = current_psp_value;
}
void update_next_task(void)
{
	//int state = TASK_BLOCKED_STATE;
	//int i;
	//for(i= 0 ; i < (MAX_TASKS) ; i++)
	//{
		current_task++;
	  current_task %= MAX_TASKS;
		//state = user_tasks[current_task].current_state;
		//if( (state == TASK_READY_STATE) && (current_task != 0) )
		//	break;
	//}

	//if(state != TASK_READY_STATE)
		//current_task = 0;
}

void enable_processor_faults(void)
{
	uint32_t *pSHCSR = (uint32_t*)0xE000ED24;

	*pSHCSR |= ( 1 << 16); //mem manage
	*pSHCSR |= ( 1 << 17); //bus fault
	*pSHCSR |= ( 1 << 18); //usage fault
}


volatile int t1,t2,t3,t4,t5;
void task1_handler(void)
{
	while(1)
	{
		t1++;
	}

}

void task2_handler(void)
{
	while(1)
	{
		t2++;
	}

}

void task3_handler(void)
{
	while(1)
	{
		t3++;
	}

}

void task4_handler(void)

{
	while(1)
	{
		t4++;
	}


}
void task5_handler(void)

{
	while(1)
	{
		t5++;
	}


}

__asm void save_current(void){
	MRS R0,PSP
	//Using that PSP value store SF2( R4 to R11)
	STMDB R0!,{R4-R11}
	PUSH {LR}
}
__asm void retrieve_next(void){
	//3. Using that PSP value retrieve SF2(R4 to R11)
		LDMIA R0!,{R4-R11}

	//4. update PSP and exit
		MSR PSP,R0
		POP {LR}
		BX LR
}
int delay=10;
__asm void switch_sp_to_psp(void)
{ PRESERVE8
	IMPORT get_psp_value
	PUSH {LR}
	BL get_psp_value
	MSR PSP,R0
	POP {LR}
	MOV R0,#0X02
	MSR CONTROL,R0
	BX LR
}
__asm void SVC_Handler(void){
			push {lr}
			TST LR,#4 
			ITE EQ
			MRSEQ R0,MSP
			MRSNE R0,PSP
			LDR R0,[R0,#28]
			LDRB R5,[R0,#-2]
			IMPORT  init_tasks_stack
			CMP R5,#0X000000F3
			LDREQ     R0, =init_tasks_stack
			BLX R0
			pop {pc}
}
int t=0;
int main(void){
		//SystemClock_Config();
		if(t==0){
		__asm("SVC 0XF3");}
			
			SysTick->LOAD=0;
		Systick_initialize();
			user_tasks[current_task].task_handler();
			while(1);
}
void SysTick_Handler(void)
{   
		
		if(delay>0){
			delay--;
			return;
		}
		t=1;
		update_next_task();
		delay=10;
		main();
	}
   

int TimeDelay;
void Delay(uint32_t nTime) { 
	TimeDelay = nTime; 
	while(TimeDelay != 0);
} 


/*handler for svc number 0xF3 to initialize os for scheduling task
retrieve and execute from tcb
pc of clockconfig
pc of systickconfig
pc of pendsv
*/
void TIM4_IRQHandler(void)
{
	ticks++;
	TIM4->SR &= ~(1<<0);			//clears interrupt after causing interrupt
}

void NonMaskableInt_IRQHandler(void)
{
	if((uint32_t) (NVIC->STIR) &(uint32_t) NonMaskableInt_IRQn) //checks for pending software interrupt
	{
		NVIC->STIR &= ~(0b11111111<<0);  //clears software interrupt
		TimerDelay(15000);
	}
}
void PendSV_Handler(void) {


	PendSV_count=PendSV_count+1; 

}


