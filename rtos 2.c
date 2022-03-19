// Basic RTOS Framework - Spring 2022
// No memory protection, no privilege enforcement
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED PF2
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED PA2
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board green LED PE0
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED PA4
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board orange LED PA3

#define PB0   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) // off-board PC4 PORTC
#define PB1   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) // off-board PC5 PORTC
#define PB2   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) // off-board PC6 PORTC
#define PB3   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) // off-board PC7 PORTC
#define PB4   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4))) // off-board PD6 PORTD
#define PB5   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4))) // off-board PD7 PORTD

#define PB0_MASK 16                            //128 64 32 16 8 4 2 1
#define PB1_MASK 32
#define PB2_MASK 64
#define PB3_MASK 128
#define PB4_MASK 64
#define PB5_MASK 128

#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define GREEN_LED_MASK 1
#define YELLOW_LED_MASK 16
#define ORANGE_LED_MASK 8

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
// function pointer
typedef void (*fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 2
#define MAX_FIELDS 20
#define MAX_CHARS 80
#define PRIORITY_MAX_LEVEL 8
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    char name[16];
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t pidCounter = 0;   // incremented on each thread created
bool scheduler=1;  //1 is priority and 0 is round robin
bool preemption=0;
bool flag=1;
USER_DATA data;
uint32_t Time[2][MAX_TASKS];
bool ping=0;
bool pong=1;
bool temp=0;
uint64_t intCpu[MAX_TASKS];
uint64_t floatCpu[MAX_TASKS];
uint8_t TASKS_CREATED=0;


uint32_t heap[MAX_TASKS*2][256]   __attribute__((location(0x20002000)))     __attribute__((aligned (1024)));
char str[50]; //for sprintf
// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly


int power(int,int);
extern void setPSP(uint32_t);
extern void setMSP(uint32_t);
extern uint32_t getR0fromPSP();
void activateASP();
void pushRegs();
void popRegs();
extern uint32_t getPSP();
extern uint32_t getMSP();
extern uint8_t getSvcnumber();
void setxPRSR();
void stringCopy(char*, const char*);
void parseFields(USER_DATA* data);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
uint16_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);
uint8_t strCompare(char*,char*);
char* i_to_a(uint64_t);
void getsUart0(USER_DATA*);

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    uint32_t pid;                  // PID
    fn pFn;                        // function pointer
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 7=lowest
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    uint8_t s;                     // index of semaphore that is blocking the thread
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pFn = 0;
    }
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
//    bool ok;
//    static uint8_t task = 0xFF;
//    ok = false;
//    while (!ok)
//    {
//        task++;
//        if (task >= MAX_TASKS)
//            task = 0;
//        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
//    }
//    return task;
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    uint8_t curTask = 0,p_level=0,i;
    if(!scheduler) //round robin scheduler
    {
        while (!ok)
        {
            task++;
            if (task >= TASKS_CREATED)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        return task;
    }
    else  //priority
    {
        curTask=taskCurrent;
        while(p_level<PRIORITY_MAX_LEVEL && (!ok))
        {

            for(i=0;i<TASKS_CREATED;i++)
            {
                curTask+=1;
                if(curTask>=TASKS_CREATED)
                    curTask=0;
                if(tcb[curTask].priority==p_level)
                {
                    ok=(tcb[curTask].state == STATE_READY || tcb[curTask].state == STATE_UNRUN);
                }
                if(ok==1)
                {
                    return curTask;
                }

            }
            p_level++;
            if(p_level>PRIORITY_MAX_LEVEL)
                p_level=0;
        }

    }
    return 0;
}

bool createThread(fn task, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0,j=0;
    bool found = false;
    // REQUIRED:
    // store the thread name
    // allocate stack space and store top of stack in sp and spInit
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure task not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pFn == task);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            for(j=0;name[j]!='\0';j++)
            {
                tcb[i].name[j]=name[j];
            }
            tcb[i].name[j]='\0';
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = pidCounter++;
            tcb[i].pFn = task;
            if(stackBytes==1024)
                tcb[i].spInit = &heap[i][256];
            else
                tcb[i].spInit = &heap[i+3][256]; //for shell
            tcb[i].sp = tcb[i].spInit;
            tcb[i].priority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(fn task)
{
    __asm volatile(" SVC #0x19 ");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(fn task)
{
    __asm volatile(" SVC #0x20 ");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(fn task, uint8_t priority)
{
    uint8_t k=0;
    while(k<TASKS_CREATED)
    {
        if(tcb[k].pFn==task)
        {
            tcb[k].priority=priority;
            break;
        }
        k++;
    }
}

bool createSemaphore(uint8_t semaphore, uint8_t count, const char name[])
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
        stringCopy(semaphores[semaphore].name,name);
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, and PC
void startRtos()
{
    taskCurrent=rtosScheduler();
    tcb[taskCurrent].state = STATE_READY;
    TASKS_CREATED=pidCounter;
    setPSP((uint32_t)tcb[taskCurrent].sp);
    activateASP();
    //start sytick timer
    NVIC_ST_CTRL_R=0;
    NVIC_ST_CURRENT_R=0;
    NVIC_ST_RELOAD_R=0x9c3F;           //(40Mhz/1000)-1 or 39999
    NVIC_ST_CTRL_R  = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE ;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    fn task=(fn)tcb[taskCurrent].pFn;
    (*task)();
}

void yield()
{
    __asm volatile(" SVC #0x10 ");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm volatile(" SVC #0x11 ");
}

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t s)
{
    __asm volatile(" SVC #0x12 ");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t s)
{
    __asm volatile(" SVC #0x13 ");
}
void callpidoff(char* a)
{
    __asm volatile(" SVC #0x15 ");
}
void callrun(char* a)
{
    __asm volatile(" SVC #0x16 ");
}

//void callscheduler(char* a)
//{
//    __asm volatile(" SVC #0x17 ");
//}
//void callpreemption(char* a)
//{
//    __asm volatile(" SVC #0x18 ");
//}
//void callipcs()
//{
//    __asm volatile(" SVC #0x21 ");
//}


// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    static uint16_t counter=1000;
    for(i=0;i<MAX_TASKS;i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            if(tcb[i].ticks>0)
                tcb[i].ticks--;
            else if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
        }
    }
    counter--;
    if(counter==0)
    {
        counter=1000;
        if (ping==0)
        {
            temp=ping;
            ping=1;
            pong=1;
        }
        else if(pong==1)
        {
            temp=pong;
            pong=0;
            ping=0;
        }
        for(i=0;i<MAX_TASKS;i++)
        {
            Time[temp][i]=0;
        }

    }
    if(preemption == 1)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; //every time the systick ISR is called and if preemption is ON, the control goes to pendsv and hence the the tasks are switched every time
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    pushRegs();
    tcb[taskCurrent].sp=(uint32_t*)getPSP();
    Time[temp][taskCurrent]+=TIMER1_TAV_R;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    taskCurrent=rtosScheduler();
    setPSP((uint32_t)tcb[taskCurrent].sp);

    if((tcb[taskCurrent].state)==STATE_READY)
    {
        popRegs();  //pop out R11-r4
        TIMER1_TAV_R=0;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
    }
    else //the task is unrun
    {
        uint8_t count=0;
        uint32_t *temp=(uint32_t*)tcb[taskCurrent].sp;
        tcb[taskCurrent].state=STATE_READY;
        --temp;
        *temp=0x01000000; //xpsr value the first 4 bits which are flags are dont cares and hence the 24th bit will be set which denotes thumb or ARM state
        --temp;
        *temp=((uint32_t)tcb[taskCurrent].pFn) | 1; //PC- should point to the start of the function and the LSB should be 1 to indicate thumb instrucrtion set
        --temp;
        *temp=0xFFFFFFFD; //LR this is EXEC-RETURN which states which mode the process should return after pendsv and which SP should be in use
        while((count++)!=5) //r12-r3-r2-r1-r0
            *(--temp)=1;
        tcb[taskCurrent].sp=(uint32_t*)temp;
        setPSP((uint32_t)tcb[taskCurrent].sp);
        TIMER1_TAV_R=0;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint8_t svc=getSvcnumber();
    uint32_t r_0=getR0fromPSP();
    uint8_t i=0,j=0,k=0,l=0;
    fn kill;
    fn restartTask;
    char *runTask=0;
    char *a=0;
    uint8_t killsemaphore;

    switch(svc)
    {
    case 0x10: //yield
        NVIC_INT_CTRL_R |=NVIC_INT_CTRL_PEND_SV;
        break;
    case 0x11: //sleep
        tcb[taskCurrent].ticks=r_0;
        tcb[taskCurrent].state=STATE_DELAYED;
        NVIC_INT_CTRL_R |=NVIC_INT_CTRL_PEND_SV;
        break;
    case 0x12: //wait
        //just decrement the count and execute the critical section if count>0
        if(semaphores[r_0].count > 0)
        {
            semaphores[r_0].count--;
            //tcb[taskCurrent].s = r_0; //index of the semaphore that is blocking the thread
        }
        else
        {
            tcb[taskCurrent].state = STATE_BLOCKED;
            semaphores[r_0].processQueue[semaphores[r_0].queueSize] = taskCurrent; //add the task to the queue
            tcb[taskCurrent].s = r_0;     //index of the semaphore that is blocking the thread
            semaphores[r_0].queueSize++;
            NVIC_INT_CTRL_R |=NVIC_INT_CTRL_PEND_SV;
        }



        break;
    case 0x13: //post
        semaphores[r_0].count++;
        if(semaphores[r_0].queueSize > 0)
        {
            tcb[semaphores[r_0].processQueue[0]].state = STATE_READY;
            semaphores[r_0].count--;
            for(i = 0;i < semaphores[r_0].queueSize;i++)
            {
                semaphores[r_0].processQueue[i] = semaphores[r_0].processQueue[i+1];
            }
            semaphores[r_0].queueSize--;
           // NVIC_INT_CTRL_R |=NVIC_INT_CTRL_PEND_SV;
        }

        break;
    case 0x14: //reboot
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ ;
        break;
    case 0x15: //pidoff
        a=(char*)r_0;
        while(k<TASKS_CREATED)
        {
            if(strCompare(a,tcb[k].name))
            {
                putsUart0("\n\rPID of ");
                putsUart0(tcb[k].name);
                putsUart0(" is: ");
                putsUart0(i_to_a(tcb[k].pid));
                putsUart0("\n\r");
                break;
            }
            k++;
        }
        if(k==TASKS_CREATED)
        {
            putsUart0("invalid task name\n\r");
        }
        break;
    case 0x16: //run
        runTask = (char *)r_0;
        for(i = 0;i < TASKS_CREATED;i++)
        {
            if(strCompare(runTask, tcb[i].name))
            {
                tcb[i].state = STATE_UNRUN;
                tcb[i].sp = tcb[i].spInit;
//                tcb[i].currentPriority = 8;
//                tcb[i].priority = 8;
                break;
            }
        }
        if(i==TASKS_CREATED)
        {
            putsUart0("invalid task name\n\r");
        }
        break;


    case 0x19: //restart
        restartTask=(fn)r_0;
        while(k<MAX_TASKS)
        {
            if(tcb[k].pFn==restartTask)
            {
                tcb[k].sp=tcb[k].spInit;
                tcb[k].state=STATE_UNRUN;
                break;
            }
            k++;
        }
        break;
    case 0x20: //kill the thread
        kill=(fn)r_0;
        l=1; //can't kill idle
        while(l<MAX_TASKS)
        {
            if(tcb[l].pFn == kill)
            {
                //                if(strCompare(tcb[l].name,"Idle"))
                //                {
                //                    putsUart0("\n\rcan't kill idle process\n\r");
                //                    break;
                //                }
                //                else
                //                {
                //                    if(tcb[l].state==STATE_BLOCKED)
                //                    {
                //                        killsemaphore=tcb[l].s;
                //                        for(j = 0;j<semaphores[killsemaphore].queueSize; j++)
                //                        {
                //                            if(semaphores[killsemaphore].processQueue[j] == l)
                //                            {
                //                                semaphores[killsemaphore].processQueue[j] = 0;
                //                                for(k = 0;k < semaphores[killsemaphore].queueSize;k++)
                //                                {
                //                                    semaphores[killsemaphore].processQueue[k] = semaphores[killsemaphore].processQueue[k+1];
                //                                }
                //                                semaphores[killsemaphore].queueSize--;
                //                            }
                //
                //                        }
                //
                //                        tcb[l].state =STATE_INVALID ;
                //                        break;
                //                    }
                //                }
                tcb[l].state = STATE_INVALID;
                //taskCount--;
                if(tcb[l].s != 0)
                {
                    killsemaphore = tcb[l].s;
                    for(j = 0;j < semaphores[killsemaphore].queueSize; j++)
                    {
                        if(semaphores[killsemaphore].processQueue[j] == i)
                        {
                            semaphores[killsemaphore].processQueue[j] = 0;
                            for(k = j;k < semaphores[killsemaphore].queueSize;k++)
                            {
                                semaphores[killsemaphore].processQueue[k] = semaphores[killsemaphore].processQueue[k+1];
                            }
                            semaphores[killsemaphore].queueSize--;
                        }
                        break;
                    }
                }
                break;
            }
            l++;
        }
        break;
    }
}



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    // Initialize system clock to 40 MHz
       //initSystemClockTo40Mhz();

       // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
       SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
       // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
       SYSCTL_GPIOHBCTL_R = 0;
       _delay_cycles(3);
       SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3|SYSCTL_RCGCGPIO_R4| SYSCTL_RCGCGPIO_R5;

       // Configure LED pin
       GPIO_PORTA_DIR_R |= RED_LED_MASK|ORANGE_LED_MASK|YELLOW_LED_MASK;  // make LED as outputs
       GPIO_PORTA_DR2R_R |= RED_LED_MASK|ORANGE_LED_MASK|YELLOW_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
       GPIO_PORTA_DEN_R |= RED_LED_MASK|ORANGE_LED_MASK|YELLOW_LED_MASK;  // enable LED
       GPIO_PORTE_DIR_R |= GREEN_LED_MASK;
       GPIO_PORTE_DR2R_R |= GREEN_LED_MASK;
       GPIO_PORTE_DEN_R |= GREEN_LED_MASK;
       GPIO_PORTF_DIR_R |=BLUE_LED_MASK;
       GPIO_PORTF_DR2R_R |=BLUE_LED_MASK;
       GPIO_PORTF_DEN_R |=BLUE_LED_MASK;

       GPIO_PORTC_DIR_R=0;
       GPIO_PORTC_DEN_R |= PB0_MASK|PB1_MASK|PB2_MASK|PB3_MASK;  // enable LEDs and pushbuttons
       GPIO_PORTC_PUR_R |= PB0_MASK|PB1_MASK|PB2_MASK|PB3_MASK; // enable internal pull-up for push button

       GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
       GPIO_PORTD_CR_R =128; //PB5_mask
       GPIO_PORTD_DIR_R=0;
       GPIO_PORTD_DEN_R |= PB4_MASK|PB5_MASK;  // enable LEDs and pushbuttons
       GPIO_PORTD_PUR_R |= PB4_MASK|PB5_MASK;

       SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;                                      // Enable Systick timer
       TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                                // Disable timer before configuring (safe programming)
       TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                                          // Configure as 32-bit timer (A+B)
       TIMER1_TAILR_R = 0xFFFFFFFF;                                                    //max value
       TIMER1_TAV_R = 0;                                                               //initail value set to 0
       TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;                     // Configure for periodic mode and Count Up timer}
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t press=0;
    if(!PB0)
        press=press+power(2,0);
    if(!PB1)
        press=press+power(2,1);
    if(!PB2)
        press=press+power(2,2);
    if(!PB3)
        press=press+power(2,3);
    if(!PB4)
        press=press+power(2,4);
    if(!PB5)
        press=press+power(2,5);
    return press;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
int power(int base,int exp)
{
    int i;
    int result=1;
    if(exp==0)
        return 1;
    else
    {
        for(i=1;i<=exp;i++)
            result=result*base;
        return result;
    }
}
void activateASP()
{
    __asm volatile(" MOV R0, #0x02 ");
    __asm volatile(" MSR CONTROL,R0 ");
    __asm volatile(" BX LR ");
}
void popRegs()
{
    __asm volatile(" MRS R0,PSP ");
    __asm volatile(" LDMIA R0!,{R4-R11}");
    __asm volatile(" MSR PSP,R0 ");
    __asm volatile(" BX LR ");

}
void pushRegs()
{
    __asm volatile(" MRS R0,PSP ");
    __asm volatile(" STMDB R0!,{R4-R11} "); //push command is not used bcoz push is used on MSP's
    __asm volatile(" MSR PSP,R0 ");
    __asm volatile(" BX LR ");
}

void setxPRSR()
{
    __asm volatile(" MOV R2,R0 ");
    __asm volatile(" MRS R0,PSP ");
    __asm volatile(" SUB R0,#4" );
    __asm volatile(" STR R2,[R0] ");
}

//referred from the internet
void stringCopy(char *to, const char *from)
{

    while(*from){
        *to++ = *from++;
    }
    *to = '\0';
}
int number_of_digits(int n)
{
    int dc = 0;

    while(n > 0)
    {
        dc++;
        n /= 10;
    }

    return dc;
}


char* i_to_a(uint64_t n)
{
    uint16_t dc = 0;

//    if(n < 0)
//    {
//        n = -1*n;
//        dc++;
//    }

    if(n==0)
    {
        str[dc++]='0';
        str[dc]='\0';
        return str;
    }
    dc += number_of_digits(n);

    str[dc] = '\0';

    while(n > 0)
    {
        str[dc-1] = n%10 + 48;
        n = n/10;
        dc--;
    }

    if(dc == 1)
        str[0] = '-';

    return str;
}

void parseFields(USER_DATA* data)
{
    int i=0,j=0;  //i used for normal string array and j used for fieldtype which can be a max of 5
    data->fieldPosition[j]=i;
    data->fieldCount=0;
    (data->fieldCount)++;
    while(data->buffer[i]!='\0')
    {
        if( (data->buffer[i]>=65 && data->buffer[i]<=90) || (data->buffer[i]>=97 && data->buffer[i]<=122))
            data->fieldType[j]='a';
        else if((data->buffer[i]>=48 && data->buffer[i]<=57) || data->buffer[i]==46 || data->buffer[i]==45)
            data->fieldType[j]='n';
        else
        {
            data->buffer[i]='\0';
            data->fieldPosition[j+1]=i+1;
            (data->fieldCount)++;
            j++;
        }

        i++;
        if(j>=MAX_FIELDS)
        {

            flag=0;
            return;
        }
    }
}
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber<MAX_FIELDS)
    {
        if(data->fieldType[fieldNumber]=='a')
            return &(data->buffer[data->fieldPosition[fieldNumber]]);
        else
        {
            putsUart0("\nthe word in the alphabet is not a alphabet");
            return '\0';
        }
    }
    putsUart0("\ninvalid number of arguments");

    return '\0';
}

uint16_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    int i=data->fieldPosition[fieldNumber];
    int result=0,temp=0,num=i,j,inc=0; //result is to store the final integer value
    //temp is used to store the intermediate value
    //num is like to counter to go till the end of the number and then to convert it
    //j is used for for lopp
    if(fieldNumber<MAX_FIELDS)
    {
        if(data->fieldType[fieldNumber]=='n')
        {
            while(data->buffer[i]!='\0')
            {
             i++;
            }

         for(j=i-1;j>=num;j--)
         {
             temp=(data->buffer[j])-48;
             result=result+temp*power(10,inc++);
         }
         return result;
        }
        else
        {
            putsUart0("not a integer string, cannot convert it");
            return 0;
        }
    }
    putsUart0("\ninvalid number of arguments");
    return 0;
}

uint8_t strCompare(char* str1,char* str2)
{
    int i=0;
    while(str1[i]!='\0' || str2[i]!='\0')
    {
        if(str1[i]==str2[i])
            i++;
        else
            return 0;
    }
    return 1;
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    int i=data->fieldPosition[0];
    char* str=&(data->buffer[data->fieldPosition[0]]);
    int f=1;

        while(str[i]!='\0' && f!=0)
        {
            if(str[i]==strCommand[i])
            {
                i++;
            }
            else
            {
                f=0;
                return 0;
            }
        }
        if(((data->fieldCount)-1)!=minArguments)
            return 0;
        return 1;
}

void getsUart0(USER_DATA* data)
{
    unsigned int ct=0;  //count
    char c;
//    goto entry;
//    entry:
//    c[ct++]=getcUart0();
// if(ct>0 && (c[ct-1]==8 || c[ct-1]==128]))
//    {
//      ct--;
//      goto entry;
//    }
    while(ct!=MAX_CHARS)
    {
        c=getcUart0();
        if(c>=32 && c<127)
            data->buffer[ct++]=c;
        else if(c==8 || c==127)
        {
            if(ct>0)
                ct--;
            else
                continue;
        }
        else if(ct==(MAX_CHARS) || (c==10 || c==13))
        {
            data->buffer[ct]='\0';
            break;
        }

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

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot() // the wait semaphore will block this function as there is no post
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
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
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
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
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
        while (readPbs() == 32)
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

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    while (true)
    {
        putsUart0("welcome\n\r");
        bool valid=false;
        while (true)
        {
            if (kbhitUart0())
            {
                getsUart0(&data);
                putsUart0("\n\r");
                parseFields(&data);
                putsUart0(data.buffer);
                putsUart0("\n\r");
                //yield();
                if(isCommand(&data,"reboot",0))
                {
                    valid=true;
                    __asm volatile(" SVC #0x14 ");
                }
                else if(isCommand(&data,"kill",1))
                {
                    valid=true;
                    fn toBeKilledPid =(fn)getFieldInteger(&data, 1);
                    destroyThread(toBeKilledPid);
                }
                else if(isCommand(&data,"preemption",1))        //Is command argument PREEMPT
                {
                    if(strCompare(getFieldString(&data,1),"on") == true)
                    {
                        valid=true;
                        preemption=true;
                    }
                    else if(strCompare(getFieldString(&data,1),"off") == true)
                    {
                        preemption=false;
                        valid=true;
                    }
                    else
                        putsUart0("Please enter valid Input Argument\r\n");
                }
                else if(isCommand(&data,"scheduler",1))        //Is command argument PREEMPT
                {
                    if(strCompare(getFieldString(&data,1),"RR") == true)
                    {
                        valid=true;
                        scheduler=false;
                    }
                    else if(strCompare(getFieldString(&data,1),"PRIO") == true)
                    {
                        scheduler=true;
                        valid=true;
                    }
                    else
                        putsUart0("Please enter valid Input Argument\r\n");
                }
                else if(isCommand(&data,"run",1) || isCommand(&data,"restart",1))
                {
                    valid=true;
                    callrun(getFieldString(&data,1));
                }
                else if(isCommand(&data,"pidof",1))  //returns the PID of the task
                {
                    valid=true;
                    callpidoff(getFieldString(&data,1));
                }
                else if(isCommand(&data,"ps",0))
                {
                    valid=true;
                    uint8_t i=0;
                    uint64_t total=0,totalCpu=0;
                    for(i=0;i<pidCounter;i++)
                    {
                        total+=Time[!temp][i];
                    }
                    for(i=0;i<pidCounter;i++)
                    {
                        totalCpu=((Time[!temp][i])*10000)/total;
                        intCpu[i]=totalCpu/100;
                        floatCpu[i]=totalCpu%100;
                    }

                    putsUart0("Pid\t\tName\t\tState\t\tPriority\t\tCpu Usage\r\n");
                    putsUart0("-------------------------------------------------------------\n\r");
                    for(i=0;i<pidCounter;i++)
                    {
                        putsUart0(i_to_a(tcb[i].pid));
                        putcUart0('\t');
                        putsUart0(tcb[i].name);
                        putcUart0('\t');
                        if(tcb[i].state == STATE_READY)
                            putsUart0("STATE_READY  \t");
                        else if(tcb[i].state == STATE_UNRUN)
                            putsUart0("STATE_UNRUN  \t");
                        else if(tcb[i].state == STATE_BLOCKED)
                            putsUart0("STATE_BLOCKED\t");
                        else if(tcb[i].state == STATE_DELAYED)
                            putsUart0("STATE_DELAYED\t");
                        else if(tcb[i].state == STATE_INVALID)
                            putsUart0("STATE_INVALID\t");
                        putsUart0("\t");
                        putsUart0(i_to_a(tcb[i].priority));
                        putsUart0("\t");
                        putsUart0(i_to_a(intCpu[i]));
                        putcUart0('.');
                        putsUart0(i_to_a(floatCpu[i]));
                        putsUart0("\n\r");
                    }
                }
                else if(isCommand(&data,"ipcs",0))
                {
                    valid=true;
                    putsUart0("Semaphore index\tName\tSemaphore Count\tQueue Size\twaiting task\tRunning Task\r\n");
                    uint8_t i,temp1=0,temp2=0;
                    for(i=1;i<MAX_SEMAPHORES;i++)
                    {
                        putsUart0(i_to_a(i));
                        putsUart0("\t\t");
                        putsUart0(semaphores[i].name);
                        putsUart0("\t\t");

                        putsUart0(i_to_a(semaphores[i].count));
                        putsUart0("\t\t");
                        putsUart0(i_to_a(semaphores[i].queueSize));
                        putsUart0("\t\t");

                        if(semaphores[i].queueSize==0)
                            putsUart0("None\t\t");
                        else
                        {
                            for(temp1=0;temp1<semaphores[i].queueSize;temp1++)
                            {
                                putsUart0(tcb[semaphores[i].processQueue[temp1]].name);
                                putsUart0(",");
                            }
                        }
                        putsUart0("\t\t");
                        for(temp2=0;temp2<pidCounter;temp2++)
                        {
                            if(tcb[temp2].state==STATE_READY||tcb[temp2].state==STATE_DELAYED)
                            {
                                if(tcb[temp2].s==i)
                                {
                                    putsUart0(tcb[temp2].name);
                                    putcUart0(',');
                                }
                            }
                        }
                        putsUart0("\n\r");
                    }
                }

                if(valid==false)
                {
                    putsUart0("invalid command\n\r");
                }
            }
            yield();
        }
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
    initUart0();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    // Initialize semaphores
      createSemaphore(keyPressed, 1,"keyPressed");
      createSemaphore(keyReleased, 0,"keyReleased");
      createSemaphore(flashReq, 5,"flashReq");
      createSemaphore(resource, 1,"resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);  //0

//    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024); //1
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);//2
    ok &= createThread(oneshot, "OneShot", 2, 1024);//3
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);//4
    ok &= createThread(debounce, "Debounce", 6, 1024);//5
    ok &= createThread(important, "Important", 0, 1024);//6
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);//7
    ok &= createThread(shell, "Shell", 6, 4096);//8

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
