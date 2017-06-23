#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <stdlib.h>
#include <stdio.h>



// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4))) // off-board red LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) // off-board orange LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4))) // off-board yellow LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4))) // off-board green LED
#define PB1  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PB2  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PB3  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define PB4  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))

#define MAX_CHAR 0x50

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE];  // store task index here
  char name1[20];

} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;
uint8_t flag = 0;
uint32_t value;


struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer

#define MAX_TASKS 10 // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskWaiting = 0;
uint8_t taskNext = 0;
uint8_t taskCount = 0;     // total number of valid tasks
uint16_t j,t=0;
uint32_t i,y;
char stri [MAX_CHAR + 1];
char str_new1[MAX_CHAR + 1];
char str_new2[MAX_CHAR + 1];
char str[10];
uint8_t field_position[20];
uint8_t field_type[20];
uint8_t field_count=0;
uint8_t maxCount;
uint32_t pid, countTime = 0;
char name1[20];
unsigned char *name;
char A[20];
	char B[20];
	char C[16];
	char D[20];
	char arg[20];
	char time[20];

uint32_t N;


struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  uint8_t skipCount;
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
  uint32_t initialCount;
   uint32_t finalCount;
   uint32_t totalCount;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------


void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
//Source: http://users.ece.utexas.edu/~valvano/Volume1/E-Book/C12_Interrupts.html
    NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
    NVIC_ST_RELOAD_R = 0x9C3F;              // maximum reload value
    NVIC_ST_CURRENT_R = 0;                // any write to current clears it
    //NVIC_ST_CTRL_R = 0x07;                   // enable SysTick during setup
}

uint32_t getsp()
{

	__asm (" MOV R0,SP");
	__asm(" BX LR");
	}
void setsp(uint32_t a)
   {
	__asm(" MOV SP,R0");
	__asm(" BX LR");
	}

int rtosScheduler()
{
  // REQUIRED: Implement prioritization to 16 levels
  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  tcb[taskCurrent].finalCount = countTime;
    tcb[taskCurrent].totalCount = (tcb[taskCurrent].finalCount - tcb[taskCurrent].initialCount);
  while (!ok)
  {
	/*task++;
	   if (task >= MAX_TASKS)
	       task = 0;
	     ok = (tcb[task].state == STATE_READY);*/

	 task++;
	  if (task >= taskCount)
	  task = 0;

		 // maxCount = tcb[task].priority;
	  if (tcb[task].skipCount > tcb[task].currentPriority)
	  {
		  	if((tcb[task].state == STATE_READY))
		  {
	  			ok = 1;
	  			tcb[task].skipCount = 0;
	  			tcb[task].initialCount = countTime;

		  }
	  }
		  else
		  {
			  tcb[task].skipCount++;
			  ok = 0;
9
		  }
	  }




 return task;

}

void rtosStart()
{
	/* NVIC_ST_CTRL_R = 0x07;                   // enable SysTick during setup
  // REQUIRED: add code to call the first task to be run, restoring the preloaded context
  _fn fn;


  taskCurrent = rtosScheduler();
  setsp(tcb[taskCurrent].sp);
  __asm(" POP {R3}");
  __asm(" POP {R3}");
  __asm(" POP {R3}");
  __asm(" POP {R0-R12}");

  __asm(" POP {PC}");*/
	TIMER1_CTL_R = 1;                       // Turning the timer on
	 NVIC_ST_CTRL_R = 0x07;                   // enable SysTick during setup

  __asm(" SVC #05");
}

bool createThread(_fn fn, char name[], int priority)
{
  /*bool ok = false;
  uint8_t i = 0;
  bool found = false;
  // REQUIRED: store the thread name
  // REQUIRED: take steps to ensure a task switch cannot occur
  // add task if room in task list
  if (taskCount < MAX_TASKS)
  {
    // make sure fn not already in list (prevent reentrancy)
    while (!found && (i < MAX_TASKS))
    {;
      found = (tcb[i++].pid ==  fn);
    }
    if (!found)
    {
      // find first available tcb record
      i = 0;
      while (tcb[i].state != STATE_INVALID) {i++;}
      tcb[i].state = STATE_READY;
      tcb[i].pid = fn;
      strcpy(tcb[i].name,name);

      for (j=252;j > 239 ;j--)
       	{
      		  stack[i][j] = j;
   		}
      //stack[i][255] = 1;

      stack[i][253]= tcb[i].pid;
      stack[i][247] = 0xFFFFFFFF9; //value of SP
      stack[i][254] =  tcb[i].pid;
      stack[i][255] = 0x01000000;

      // REQUIRED: preload stack to look like the task had run before
      tcb[i].sp = &stack[i][238]; // REQUIRED: + offset as needed for the pre-loaded stack
      tcb[i].priority = priority;
      tcb[i].skipCount=0;
      tcb[i].currentPriority = priority;
      // increment task count
      taskCount++;
      ok = true;
    }
  }
  // REQUIRED: allow tasks switches again
  return ok;*/
	__asm(" SVC #06");
}

// REQUIRED: modify this function to destroy a thread
void destroyThread(_fn fn)
{
	/*for(i=0;i<MAX_TASKS;i++)
	{
		if(tcb[i].pid == fn)
		{
			tcb[i].state = STATE_INVALID;
		}


	}*/
	__asm(" SVC #07");


}

struct semaphore* createSemaphore(int count,char name1[])
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {

    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
    strcpy(pSemaphore->name1,name1);
  }
  return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler
void yield()
{

	__asm (" SVC #01");



}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses
void sleep(uint32_t tick)
{

			__asm(" SVC #02");


}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler
void wait(struct semaphore *pSemaphore)
{

	__asm(" SVC #03");



}

// REQUIRED: modify this function to signal a semaphore is available
void post(struct semaphore *pSemaphore)
{
		__asm(" SVC #04");
}

// REQUIRED: modify this function to add support for the system timer
void systickIsr()
{

	__asm("  PUSH {R4-R11}");
 	for (i=0; i < MAX_TASKS; i++)
		{
		if (tcb[i].state == STATE_DELAYED)
		{
			tcb[i].ticks--;

				if(tcb[i].ticks == 0)
				{
					tcb[i].state = STATE_READY;
				}

		}
		}

		NVIC_INT_CTRL_R = 0x10000000;

		__asm("  POP {R4-R11}");





}
uint32_t get_N()
{
	}
// REQUIRED: modify this function to add support for the service call
void svCallIsr(uint32_t *arg1,unsigned char *arg2,uint32_t *arg3)
{
    __asm(" LDR  R0, [SP, #64]");         //Source: Book: The Definitive Guide to ARM Cortex-M3 and M4 Processors; Pg 332
     __asm(" LDRB R0, [R0, #-2]");
     value = get_N();


	switch(value)
	{
	case 01:
	{
		NVIC_INT_CTRL_R = 0x10000000;
	 break;
	}
	case 02:
	{

		tcb[taskCurrent].ticks = arg1;
		tcb[taskCurrent].state = STATE_DELAYED;
		NVIC_INT_CTRL_R = 0x10000000;
     break;
	}
	case 03:
	{
		struct semaphore *pSemaphore;
		pSemaphore = arg1;
	if (pSemaphore->count > 0)
			{

				 pSemaphore->count--;
			}

			else
			{
				tcb[taskCurrent].state = STATE_BLOCKED;
				tcb[taskCurrent].semaphore = pSemaphore;
				pSemaphore->processQueue[pSemaphore->queueSize] = tcb[taskCurrent].pid;
				pSemaphore->queueSize ++;
				 for(i=0; i < taskCount; i++)
							{
								 if( tcb[i].semaphore == tcb[taskCurrent].semaphore && i != taskCurrent)
							   {
								taskWaiting = i;

								tcb[taskWaiting].currentPriority = tcb[taskCurrent].priority;
							   }
							}

	          }
	NVIC_INT_CTRL_R = 0x10000000;
		break;
	}
	case 4:
	{
		struct semaphore *pSemaphore;
				pSemaphore = arg1;
		pSemaphore->count++;
				if(pSemaphore->count == 1)
				{

					for(i=0; i< taskCount; i++)
					{
						if (tcb[i].pid == pSemaphore->processQueue[0])
						{
							tcb[i].state = STATE_READY;
							pSemaphore->count--;
						}

					}
					if(pSemaphore->queueSize > 0)
								{
									pSemaphore->queueSize--;
								}


					for(i=1;i<MAX_QUEUE_SIZE-1;i++)
					{
						pSemaphore->processQueue[i-1] = pSemaphore->processQueue[i];
					}
					tcb[taskWaiting].currentPriority = tcb[taskWaiting].priority;
				}
				NVIC_INT_CTRL_R = 0x10000000;
				break;
	}
	case 5:
	{

		  NVIC_INT_CTRL_R = 0x10000000;
		  				break;
	}
	case 6:
	{
		 _fn fn = arg1;
		 name=arg2;
		//strcpy(name,arg);
		uint32_t priority = arg3;
		 bool ok = false;
		  uint8_t i = 0;
		  bool found = false;
		  // REQUIRED: store the thread name
		  // REQUIRED: take steps to ensure a task switch cannot occur
		  // add task if room in task list
		  if (taskCount < MAX_TASKS)
		  {
		    // make sure fn not already in list (prevent reentrancy)
		    while (!found && (i < MAX_TASKS))
		    {;
		      found = (tcb[i++].pid ==  fn);
		    }
		    if (!found)
		    {
		      // find first available tcb record
		      i = 0;
		      while (tcb[i].state != STATE_INVALID) {i++;}
		      tcb[i].state = STATE_READY;
		      tcb[i].pid = fn;
		      strcpy(tcb[i].name,name);

		      for (j=252;j > 239 ;j--)
		       	{
		      		  stack[i][j] = j;
		   		}

		      stack[i][253]= tcb[i].pid;
		      stack[i][247] = 0xFFFFFFFF9;
		      stack[i][254] =  tcb[i].pid;
		      stack[i][255] = 0x01000000;

		      // REQUIRED: preload stack to look like the task had run before
		      tcb[i].sp = &stack[i][238]; // REQUIRED: + offset as needed for the pre-loaded stack
		      tcb[i].priority = priority;
		      tcb[i].skipCount=0;
		      tcb[i].currentPriority = priority;
		      // increment task count
		      taskCount++;
		      ok = true;
		    }
		    else
		    {
		    	i--;
		    	tcb[i].state = STATE_READY;
		    	 tcb[i].priority = priority;
		    	 tcb[i].skipCount=0;
		    			      tcb[i].currentPriority = priority;
		    			      ok = true;
		    }
		  }
		  // REQUIRED: allow tasks switches again
		  return ok;

	}
	case 7:
		{
			uint16_t p,o;
			 _fn fn = arg1;

			for(i=0;i<MAX_TASKS;i++)
				{
					if(tcb[i].pid == fn)
					{
						tcb[i].state = STATE_INVALID;
					/*	struct semaphore *pSema;
						 pSema = tcb[i].semaphore;
						 for(o=0; o < semaphoreCount; o++)
						 {
							 if (strcmp(pSema->name1,semaphores[o].name1) == 0)
									 {
								 for( p = 0; p < pSema->queueSize ; p++)
											{
												if (pSema->processQueue[p] == tcb[i].pid)
												{
													pSema->processQueue[p] = 0;
													pSema->processQueue[p] = pSema->processQueue[p+1] ;
													pSema->queueSize--;


												}

										   }

									 }
						 }*/
						}


				}

		}
	case 8:
	{
		float cpuTime;
					uint8_t k;
					uint32_t totalTime=0;
					//uint8_t countTime = 0;
					 for(k=0;k<taskCount;k++)
					 {
						totalTime += tcb[k].totalCount;
					}
					  for(k = 0;k< taskCount;k++)
					  {
						 putsUart0("\r\n");

							putsUart0(tcb[k].name);
							putsUart0("   ");
							sprintf(A, " %0.1i", tcb[k].pid);

						   putsUart0(A);
						 putsUart0("   ");

						 putsUart0("   ");

						   cpuTime = ((tcb[k].totalCount * 100 )/(totalTime));
						   sprintf(time," %0.1f",cpuTime);
						   putsUart0(time);

					  }

	}


	}

}

// REQUIRED: modify this function to add support for the service call
void pendSvIsr()
{
	    __asm("  PUSH {R4-R11}");
	   if(t==1)
	   {
		tcb[taskCurrent].sp  = getsp();
	   }
	   t=1;

		taskCurrent = rtosScheduler();
		setsp(tcb[taskCurrent].sp);

		__asm("  POP {R4-R11}");

		NVIC_INT_CTRL_R = 0x00;



}
void Timer1Isr()
{
	countTime++;

	TIMER1_ICR_R = TIMER_ICR_TATOCINT;   //clear interrupt flag
	}



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


// Initialize Hardware
void initHw()
{
  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           4 pushbuttons, and uart

	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
	    // Note UART on port A must use APB
	    SYSCTL_GPIOHBCTL_R = 0;

	    // Enable GPIO port A and F peripherals
	    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF;

	    // Configure LED  pins
	    GPIO_PORTD_DIR_R = 0x0F;  // bits 1 and 3 are outputs, other pins are inputs
	    GPIO_PORTD_DR2R_R = 0x0F; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	    GPIO_PORTD_DEN_R = 0x0F;  // enable LEDs

	   GPIO_PORTF_DIR_R = 0x04;  // bits 1 and 3 are outputs, other pins are inputs
	    GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	   GPIO_PORTF_DEN_R = 0x04;  // enable LEDs

	    // Configure pushbutton pins
		GPIO_PORTC_DEN_R = 0xF0;  // enable LEDs and pushbuttons
	    GPIO_PORTC_PUR_R = 0xF0;  // enable internal pull-up for push button

	    // Configure UART0 pins
		SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
	    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
		GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
	    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
		UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
	    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
	    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


	    // Configure Timer 1 as the time base
	   		 SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
	   		 TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
	   		 TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
	   		 TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
	   		 TIMER1_TAILR_R = 0xFA0;                          // set load value to 400 for 100 kHz interrupt rate
	   		 TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	   		 NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)*/


}
void putcUart0(char c)
{

	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE)
	yield();
	return UART0_DR_R & 0xFF;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
   __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
//Step 2 Taking a string
char getstring(char *stri)
{
	uint8_t char_count = 0;
	char c;

	while(1)
	{
		c=getcUart0();
		if (c == 8 && char_count == 0)
		{
			continue;
		}
		if( char_count != MAX_CHAR )
		{
			if (c == 0x0D)
			{
				break;
			}
			if (c == 8)
			{
				char_count --;
			}
			if (c >= 0x20)
			{
				stri [char_count] = c;
				char_count++;
			}


		}
		else
		{

			break;
		}
	}
	stri [char_count+1] = 0;
	 putsUart0(stri);

}



// Step3 Checking string for the string values and arguments
	 void check_string(char stri[],uint8_t *field_count, char *str_new1,uint8_t *field_position)
	 {
		            //char str_new1[81] = {0};
		 			char str_new2[81] = {0};
		 			char field_type[20]={0};
		 			uint8_t i=0,j,l=0;
		 			char first_str[40] = {0};
		 			char second_str[40] = {0};
		 			(*field_count) = 0;

	 while(i< 20)
	     {

	 	   if (stri[i]>=0x41 && stri[i]<= 0x5A || stri[i]>=0x61 && stri[i]<= 0x7A)
	 	   {
	 		   str_new1[i]=stri[i];
	 	   }
	 	   else if ((stri[i]>=0x30 && stri[i]<=0x39) || stri[i]==0x2E || stri[i]== 0x2A || stri[i] == 0x2D)
	 	   {
	 		   str_new1[i]=stri[i];
	 	   }
	 	   else str_new1[i]='\0';

	 	   i++;
	    }
	 while(l< 20)

    {

	   if (stri[l]>=0x41 && stri[l]<= 0x5A || stri[l]>=0x61 && stri[l]<= 0x7A)
	   {
		   str_new2[l]='c';
	   }
	   else if ((stri[l]>=0x30 && stri[i]<=0x39) || stri[l]==0x2E || stri[l]== 0x2A || stri[l] == 0x2D)
	   {
		   str_new2[l]='n';
	   }
	   else str_new2[l]='\0';

	   l++;
   }



	 int x=0,y=0;

for(j=0; j<40; j++)
{
	first_str[x] = str_new2[j];
	second_str[x] = str_new2[j+1];

	if (second_str[x] != first_str[x] && second_str[x] != '\0')
	{
		field_position[y]=j+1;
		field_type[y]=second_str[x];
	    (*field_count)++;
		y++;
	}
	x++;
}
putsUart0(str_new1);
putsUart0(str_new2);
}


_Bool iscommand(char *entered_field_name, int no_of_arguments)
	 {
	 	int m = strcmp (&str_new1[field_position[0]],entered_field_name);
	 	if (m == 0 && (field_count > no_of_arguments))
	 	{
	 		return 1;
	 	}
	 	else
	 	{

	 		return 0;

	 	}
	 }
float get_number(int field_index)
	{
		float number = 0;
		number = atof(&str_new1[field_position [field_index]]);
		return number;

	}
char *get_string2(int field_index)
	{
		static char string2[20];
		strcpy(string2,&str_new1[field_position [field_index]]);
		return string2;

	}
void get_commands()
		  	  {

		  		  if (iscommand("kill",1))
		  		{
		  			   pid = get_number(1);
		  			   destroyThread(pid);

		  		}
		  		  else if (iscommand("ps",0))
		  		{

		  			  __asm(" SVC #8");

		  		}
		  		  else if(iscommand("ipcs",0))
		  		  {
		  			  uint8_t l,m;
		  			  for(l = 0; l < semaphoreCount ; l++ )
		  			  {
		  				putsUart0("\r\n");
		  				putsUart0(semaphores[l].name1);
		  			    putsUart0("    ");
		  				 sprintf(B, " %u ",semaphores[l].count);
		  				 putsUart0(B);
		  				putsUart0("    ");
		  				sprintf(C, "%u ",semaphores[l].queueSize);
		  			    putsUart0(C);
		  			  for(m = 0; m < 5; m++)
						{
							//putsUart0("\r\n");
							sprintf(A,"%u",semaphores[l].processQueue[m]);
							putsUart0(A);
							putsUart0("   ");

						}

		  			  }


		  		  }
		  		 else if (iscommand("pidof",1))
		  				  		  {

		  				  			  uint8_t n;


		  				  		const char *striing = get_string2(1);
		  		                  for (n=0; n < taskCount; n++)
		  				  			{

		  				  				if ((strcmp(tcb[n].name, striing)) == 0)
		  				  				{
		  				  					putsUart0("\r\n");
		  				  					putsUart0("The process ID is :");
		  				  					sprintf(C, " %0.1i", tcb[n].pid);
		  				  					putsUart0(C);
		  				  				}
		  				  			}


		  				  		  }
		  		  else if (iscommand("pname",1))
		  				  		  {
		  		uint8_t n;


				const char *striing = get_string2(1);
				for (n=0; n < taskCount; n++)
				{

					if ((strcmp(tcb[n].name, striing)) == 0)
					{
				   createThread(tcb[n].pid, "tcb[n].name", tcb[n].priority);
					}
				}


		  				  		  }
		  		  else if (iscommand("reboot",0))
		  		{
		  				__asm("    .global _c_int00\n"
		  				       "    b.w     _c_int00");
		  		}
		  		else
		  			 	{
		  			 		putsUart0("\r\n Please Enter a valid command\r\n");
		  			 	}

		  	  }

void shell()
{
	// REQUIRED: add processing for the shell commands here through the UART
  while (true)
  {
		putsUart0("\r\nPlease Enter the commands\r\n");
 		getstring(stri);
		check_string(stri, &field_count, str_new1, field_position);
		get_commands();
		yield();
  }

}

// REQUIRED: add code to return a value from 0-15 indicating which of 4 PBs are pressed
uint8_t readPbs()
{
   if (!(PB1))
  {
	  return 1;
  }
   else if (!(PB2))
  {
	  return 2;
  }
   else if (!(PB3))
  {
	  return 4;
  }
   else if (!(PB4))
  {
	  return 8;
  }
   else
   {
	   return 0;
   }

}




// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}
void idle2()
{
  while(true)
  {
    RED_LED = 1;
    waitMicrosecond(1000);
    RED_LED = 0;
    yield();
  }
}

void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);

  }

}

void oneshot()
{
 while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;

  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash4hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
	}

    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}


void important()
{
	while(true)
	{
	wait(resource);
	BLUE_LED = 1;
	sleep(1000);
	BLUE_LED = 0;
	post(resource);
	}
}




//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
 {
  bool ok;

  // Initialize hardware
  initHw();
  rtosInit();

  // Power-up flash
  RED_LED = 1;
  waitMicrosecond(250000);
  RED_LED = 0;
  waitMicrosecond(250000);

  // Initialize semaphores
  keyPressed = createSemaphore(1,"keyPressed");
  keyReleased = createSemaphore(0,"keyReleased");
  flashReq = createSemaphore(5,"flashReq");
  resource = createSemaphore(1,"resource");

  // Add required idle process
  ok =  createThread(idle, "Idle", 15);
 // ok =  createThread(idle2, "Idle2", 0);

  // Add other processes
 ok &= createThread(lengthyFn, "LengthyFn", 12);
  ok &= createThread(flash4Hz, "Flash4hz", 4);
  ok &= createThread(oneshot, "OneShot", 4);
  ok &= createThread(readKeys, "ReadKeys", 8);
  ok &= createThread(debounce, "Debounce", 8);
  ok &= createThread(important, "Important", 0);
  ok &= createThread(uncooperative, "Uncoop", 10);
  ok &= createThread(shell, "Shell", 8);

  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;

  // don't delete this unreachable code
  // if a function is only called once in your code, it will be
  // accessed with two goto instructions instead of call-return,
  // so any stack-based code will not function correctly
  yield(); sleep(0); wait(0); post(0);
}

