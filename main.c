// RTOS Framework - Fall 2018
// J Losh

// Student Name:SUSHMITHA KRISHNA
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// 06_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) // off-board red LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board orange LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board yellow LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) // off-board green LED
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED

#define PB1            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PB2            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB3            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB4            (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))

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
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t n;
uint32_t sysstack;
uint8_t prior = 1;
uint8_t prior_inheritance = 1;
uint16_t count = 0;
uint8_t valid1 = 0;
struct semaphore *pSemaphore;
uint32_t start_time;
uint32_t end_time;
uint32_t totaltime;
uint8_t val = 0;
uint8_t preemption = 1;
uint8_t j;

#define input_length 50
char String_Entered[input_length];
#define MAX_CHARS 80
#define MAX_FIELDS 10
uint8_t field = 0;
uint8_t type[MAX_FIELDS];
uint8_t pos[MAX_FIELDS];
char str[MAX_CHARS + 1];
char* che1[10] = { "idle", "lengthyfn", "flash4hz", "oneshot",
                           "readkeys", "debounce", "important", "uncoop",
                           "shell"};

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    uint8_t priority;              // 0=highest, 15=lowest
    uint8_t currentPriority;       // used for priority inheritance
    uint64_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;     // pointer to the semaphore that is blocking the thread
    uint16_t skipcount;
    uint64_t b;
    uint64_t processTime;
    uint64_t taskTime;
    uint64_t cpuPercentage;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

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
    // REQUIRED: initialize systick for 1ms system timer
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN
            | NVIC_ST_CTRL_ENABLE;
}

void setSP(uint32_t value)
{
    __asm("              ADD  SP, #0x08");
    __asm("              MOV   SP, R0 ");
    __asm("              SUB  SP, #0x08");
}

uint32_t getSP()
{

    __asm("              MOV  R0, SP" );
    __asm("              BX    LR     ");
}

void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    _fn fn;
    taskCurrent = rtosScheduler();
    sysstack = getSP();
    setSP((uint32_t) tcb[taskCurrent].sp);
    fn = (_fn) tcb[taskCurrent].pid;
    (*fn)();
    // Add code to initialize the SP with tcb[task_current].sp;
}

bool createThread(_fn fn, char name[], int priority)
{
    __asm("    SVC #05");

}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm("    SVC #06");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i;
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (tcb[i].pid == fn)
        {
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
        }
    }
}

struct semaphore* createSemaphore(uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
    }
    return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    __asm("    SVC #01");
    // push registers, call scheduler, pop registers, return to new function

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm("    SVC #02");
    // push registers, set state to delayed, store timeout, call scheduler, pop registers,
    // return to new function (separate unrun or ready processing)
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm("    SVC #03");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm("     SVC #04");
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        if (prior == 0)
        {
            ok = (tcb[task].state == STATE_READY
                    || tcb[task].state == STATE_UNRUN);
        }
        if (prior == 1)
        {
            if (tcb[task].skipcount <= tcb[task].currentPriority)
            {
                tcb[task].skipcount++;
            }
            else
            {
                ok = (tcb[task].state == STATE_READY
                        || tcb[task].state == STATE_UNRUN);
                tcb[task].skipcount = 0;

            }
        }
    }
    return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint32_t i, alpha = 0.9;
    for (i = 0; i < MAX_TASKS; i++)
    {
        if ((tcb[i].ticks > 0) && (tcb[i].state == STATE_DELAYED))
        {
            tcb[i].ticks--;
            if (tcb[i].ticks == 0)
                tcb[i].state = STATE_READY;
        }
    }

    if (count == 1000)
    {
        count = 0;
        start_time = 0;
        end_time = 0;
        for (i = 0; i < MAX_TASKS; i++)
        {

            tcb[i].b = alpha * tcb[i].b + (1 - alpha) * tcb[i].taskTime;
            tcb[i].cpuPercentage = (((float) tcb[i].b * 100) / totaltime) * 100;
            tcb[i].taskTime=0;
            tcb[i].processTime=0;

        }
        totaltime=0;
    }
    count++;

    if (preemption == 1)
    {
        tcb[taskCurrent].state = STATE_READY;
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    __asm("         PUSH {R4-R11}" );
    tcb[taskCurrent].sp = (void*) getSP();
    setSP(sysstack);

    end_time = TIMER1_TAV_R;
    tcb[taskCurrent].processTime = end_time - start_time;
    tcb[taskCurrent].taskTime += tcb[taskCurrent].processTime;
    totaltime += tcb[taskCurrent].processTime;
    taskCurrent = rtosScheduler();
    TIMER1_TAV_R = 0;
    start_time = TIMER1_TAV_R;

    if (tcb[taskCurrent].state == STATE_UNRUN)
    {
        setSP((uint32_t) tcb[taskCurrent].sp);
        stack[taskCurrent][255] = 0x01000000;
        stack[taskCurrent][254] = (uint32_t) tcb[taskCurrent].pid;
        stack[taskCurrent][253] = 11;
        stack[taskCurrent][252] = 10;
        stack[taskCurrent][251] = 9;
        stack[taskCurrent][250] = 8;
        stack[taskCurrent][249] = 7;
        stack[taskCurrent][248] = 6;
        stack[taskCurrent][247] = 0XFFFFFFF9;
        stack[taskCurrent][246] = 4;
        stack[taskCurrent][245] = 4;
        stack[taskCurrent][244] = 4;
        stack[taskCurrent][243] = 4;
        stack[taskCurrent][242] = 4;
        stack[taskCurrent][241] = 4;
        stack[taskCurrent][240] = 4;
        stack[taskCurrent][239] = 4;
        stack[taskCurrent][238] = 4;
        stack[taskCurrent][237] = 4;
        stack[taskCurrent][236] = 4;
        tcb[taskCurrent].sp = &stack[taskCurrent][236];
        tcb[taskCurrent].state = STATE_READY;
    }
    if (tcb[taskCurrent].state == STATE_READY)
    {
        setSP((uint32_t) tcb[taskCurrent].sp);

        __asm("           POP {R11,R10,R9,R8,R7,R6,R5,R4}" );
    }
}
uint8_t getSVVALUE()
{

}
uint32_t getR0()
{
}
char* getR1()
{
    __asm("     MOV R0,R1");
}
uint32_t getR2()
{
    __asm("     MOV R0,R2");
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    //
    uint32_t reg0 = getR0();
    char* reg1 = getR1();
    uint32_t reg2 = getR2();

    uint8_t x, k, i;
    __asm("    MOV R0, SP");
    __asm("    LDR R0, [R0, #64]");
    __asm("    LDRB R0, [R0, #-2]");
    n = getSVVALUE();

    switch (n)
    {
    case 1:
    { // flash4Hz();
        tcb[taskCurrent].state = STATE_READY;
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 2:
    {
        tcb[taskCurrent].ticks = reg0;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 3:
    {
        struct semaphore *pSemaphore;
        pSemaphore = reg0;
        uint8_t k, c = 0;
        uint8_t inherit;
        if (pSemaphore->count == 0)
        {
            tcb[taskCurrent].state = STATE_BLOCKED;
            pSemaphore->processQueue[pSemaphore->queueSize] =
                    (uint32_t) tcb[taskCurrent].pid;
            pSemaphore->queueSize++;

            tcb[taskCurrent].semaphore = pSemaphore;
            if (prior_inheritance == 1)
            {
                while (c < MAX_TASKS)
                {
                    if ((tcb[c].semaphore == tcb[taskCurrent].semaphore)
                            && (tcb[c].currentPriority
                                    > tcb[taskCurrent].currentPriority))
                    {
                        inherit = tcb[taskCurrent].currentPriority;
                        tcb[c].currentPriority = inherit;
                    }
                    c++;
                }
            }
            NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        }
        else
        {
            pSemaphore->count--;
        }
        break;
    }
    case 4:
    {
        struct semaphore *pSemaphore;
        pSemaphore = (uint32_t) reg0;
        tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
        pSemaphore->count++;
        if ((pSemaphore->count > 0) && (pSemaphore->queueSize > 0))
        {
            for (x = 0; x < MAX_TASKS; x++)
            {
                for (k = 0; k < pSemaphore->queueSize; k++)
                {
                    if (pSemaphore->processQueue[k] == (uint32_t) tcb[x].pid)
                    {
                        pSemaphore->processQueue[k] = 0;
                        tcb[x].state = STATE_READY;
                        pSemaphore->count--;
                        pSemaphore->queueSize--;


                    }
                }

            }
        }
        tcb[taskCurrent].state = STATE_READY;
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;

    }
    case 5:
    {
        bool ok = false;
        uint8_t i = 0;
        bool found = false;
        // REQUIRED: store the thread name
        // add task if room in task list
        if (taskCount < MAX_TASKS)
        {
            // make sure fn not already in list (prevent reentrancy)
            while (!found && (i < MAX_TASKS))
            {
                found = (tcb[i++].pid == (_fn) reg0);
            }
            if (!found)
            {
                // find first available tcb record
                i = 0;
                while (tcb[i].state != STATE_INVALID)
                {
                    i++;
                }
                tcb[i].state = STATE_UNRUN;
                tcb[i].pid = (_fn) reg0;
                tcb[i].sp = &stack[i][255];
                tcb[i].priority = reg2;
                tcb[i].currentPriority = reg2;
                strcpy(tcb[i].name, reg1);
                // increment task count
                taskCount++;
                ok = true;
            }
        }
        // REQUIRED: allow tasks switches again
        return ok;
    }

    case 6:
    {
        for (x = 0; x < MAX_TASKS; x++)
        {
            if (tcb[x].pid == (_fn) reg0)
            {
                pSemaphore = tcb[x].semaphore;
                for (j = 0; j < MAX_QUEUE_SIZE; j++)
                {
                    for (i = 0; i < taskCount; i++)
                    {
                        if (pSemaphore->processQueue[j] == tcb[i].pid)
                        {
                            pSemaphore->processQueue[j] = 0;
                            tcb[i].state = STATE_READY;

                            pSemaphore->queueSize--;
                        }
                    }
                }

                tcb[x].state = STATE_INVALID;
                                tcb[x].pid = 0;
                                tcb[x].cpuPercentage=0;
                                taskCount--;
            }

        }

        break;
    }
    }
}
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
    //           5 pushbuttons, and uart
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,F and C peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB
            | SYSCTL_RCGC2_GPIOF;

    // Configure LED and pushbutton pins
    GPIO_PORTA_DIR_R |= 0xE0; // bits 7,6,5 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0xFC;  // enable LEDs and pushbuttons
    GPIO_PORTA_PUR_R |= 0x1C;

    GPIO_PORTB_DIR_R |= 0x10;  // bits 4 is output, other pins are inputs
    GPIO_PORTB_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= 0xD0;  // enable LEDs and pushbuttons
    GPIO_PORTB_PUR_R |= 0xC0;

    GPIO_PORTF_DEN_R |= 0x04; // enable LEDs and pushbuttons(red, blue, green)
    //GPIO_PORTF_PUR_R |= 0x10;  // enable internal pull-up for push button

    GPIO_PORTF_DIR_R |= 0x04; // bits 1,2 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)

    // Configure GPIO pins for UART0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                     // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                   // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;             // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;             // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                              // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; // configure for periodic mode (count down) TIMER_TAMR_TACDIR
    TIMER1_TAV_R = 0x00;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

}

char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
    {
        yield();
    }
    return UART0_DR_R & 0xFF;
}

void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = c;
}
void putnUart0(uint8_t n)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = n;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");
    // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
    // 6
    __asm("             CBZ  R1, WMS_DONE1");
    // 5+1*3
    __asm("             NOP");
    // 5
    __asm("             B    WMS_LOOP1");
    // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");
    // 1
    __asm("             CBZ  R0, WMS_DONE0");
    // 1
    __asm("             B    WMS_LOOP0");
    // 1*3
    __asm("WMS_DONE0:");
    // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t led = 0;
    if (!PB1)
    {
        led = 1;
    }
    if (!PB2)
    {
        led = led + 2;
    }
    if (!PB3)
    {
        led = led + 4;
    }
    if (!PB4)
    {
        led = led + 8;
    }
    if (!PUSH_BUTTON)
    {
        led = led + 16;
    }
    return led;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle1()
{
    while (true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void idle2()
{
    while (true)
    {
        GREEN_LED = 1;
        waitMicrosecond(1000);
        GREEN_LED = 0;
        yield();
    }
}
void flash4Hz()
{
    while (true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while (true)
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
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while (true)
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
    while (true)
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
            createThread(flash4Hz, "Flash4Hz", 0);
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
    while (true)
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
    while (true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while (true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void i2s(uint32_t n)
{
    char c3, c2, c1, c0;
    uint16_t a;
    a = n % 10;
    n = n / 10;
    c3 = a + 48;
    a = n % 10;
    c2 = a + 48;
    n = n / 10;
    a = n % 10;
    c1 = a + 48;
    n = n / 10;
    a = n % 10;
    c0 = a + 48;

    putcUart0(c0);
    putcUart0(c1);
    putcUart0(c2);
    putcUart0(c3);
}

void f2s(uint32_t n)
{
    char c3, c2, c1, c0;
    uint16_t a;
    a = n % 10;
    n = n / 10;
    c3 = a + 48;
    a = n % 10;
    c2 = a + 48;
    n = n / 10;
    a = n % 10;
    c1 = a + 48;
    n = n / 10;
    a = n % 10;
    c0 = a + 48;

    putcUart0(c0);
    putcUart0(c1);
    putcUart0(46);
    putcUart0(c2);
    putcUart0(c3);
}

void shell()
{
    char *final[MAX_CHARS] = " \0";
    char *input_str = " \0";
    while (1)
    {
        memset(String_Entered, NULL, input_length);

       putsUart0("\n\r");
        uint8_t len = 0;
        char name[81];
        char ch;
        uint8_t i = 0;
        char *input_str = " ";
        char *in_str;
        input_str = String_Entered;
        uint8_t ctr = 0;
        putsUart0("*********Fall 2018********\n\r");
                putsUart0("Design of a Real Time Operating System\n\r");
                putsUart0("Commands:\n\r");
                putsUart0("1.Set Pi ON/OFF\n\r");
                putsUart0("2.Set Priority ON/OFF\n\r");
                putsUart0("3.Set Preemption ON/OFF\n\r");
                putsUart0("4.IPCS:Inter Process Communication Table\n\r");
                putsUart0("5.PS:Process State\n\r");
                putsUart0("6.Kill:To Destroy a Thread\n\r");
                putsUart0("7.Create Thread\n\r");
                putsUart0("8.Pidof:\n\r");
                putsUart0("9.Reboot\n\r");
        putsUart0("Enter your String:\n\r");
        putsUart0("\n\r");

        while (1)
        {
            char c = getcUart0();
            c= tolower(c);
            if (c == '\r')
            {
                if (ctr != 0)
                    break;
            }

            else if (c == 8)
            {
                if (ctr > 0)
                {
                    ctr--;
                    input_str[ctr] = NULL;

                }
                else if (ctr == 0)
                {
                    input_str[ctr] = NULL;
                }

            }
             else if (c == '\b')
             {
             if (ctr > 0)
             ctr--;
             }
            else if (c >= 32)
            {
                input_str[ctr] = c;
                ctr++;
            }
        }

        putsUart0(" the Enterted STring is \n\r");
        putsUart0("\n\r");
        putsUart0(input_str);
        const char del[2] = " ";
        char *token = strtok(input_str, del);
        while (token != NULL)
        {

            final[len] = token;
            len++;

            token = strtok(NULL, del);
        }

        char* com = final[0];
        int len1 = strlen(com);
        char* che[4] = { "keyPressed", "keyRequired", "flashReq", "resource" };
        int a;
        if (strcmp(final[0], "pidof") == 0)
        {
            uint8_t j, i;
            for (j = 0; j < MAX_TASKS; j++)
            {
                if (strcmp(final[1], che1[j]) == 0)
                {
                    putsUart0("\n\rThe PID of the Task is..");
                    i2s(tcb[j].pid);
                    putcUart0('\n');
                    putcUart0('\r');
                    break;
                }
            }
        }
        else if (strcmp(final[0], "set") == 0 && strcmp(final[1], "priority") == 0)
        {
            if (strcmp(final[2], "on") == 0)
            {
                putsUart0("\n\rThe Priority is on");
                putcUart0('\n');
                putcUart0('\r');
                prior = 1;
            }
            if (strcmp(final[2], "off") == 0)
            {
                putsUart0("\n\rThe Priority is off");
                putcUart0('\n');
                putcUart0('\r');
                prior = 0;
            }

        }
        else if (strcmp(final[0], "set") == 0 && strcmp(final[1], "preemption") == 0)
        {
            if (strcmp(final[2], "on") == 0)
            {
                putsUart0("\n\rThe Preemption is on");
                putcUart0('\n');
                putcUart0('\r');
                preemption = 1;
            }
            if (strcmp(final[2], "off") == 0)
            {
                putsUart0("\n\rThe Preemption is off");
                putcUart0('\n');
                putcUart0('\r');
                preemption = 0;
            }

        }
        else if (strcmp(final[0], "set") == 0 && strcmp(final[1], "pi") == 0)
               {
                   putcUart0('\n');
                   putcUart0('\r');
                   if (strcmp(final[2], "on") == 0)
                   {
                       putsUart0("\n\rThe Priority Inheritance is on");
                       putcUart0('\n');
                       putcUart0('\r');
                       prior_inheritance = 1;
                   }
                   if (strcmp(final[2], "off") == 0)
                   {
                       putsUart0("\n\rThe Priority Inheritance is off");
                       putcUart0('\n');
                       putcUart0('\r');
                       prior_inheritance = 0;
                   }

               }
        else if (strcmp(final[0], "ps") == 0)
        {
            uint8_t p;
            char* che[4] = { "keyPressed", "keyRequired", "flashReq", "resource" };
            putsUart0("\n\r");
            putsUart0(
                    " PID\t|PROCESS NAME\t\t|CPU PERCENTAGE\t    |PROCESS STATE\n\r");
            putsUart0("------------------------------------------------------------------");
            putsUart0("\n\r");
            for (p = 0; p < 9; p++)
            {
                putsUart0(" ");
                i2s(tcb[p].pid);
                putsUart0("\t|");
                putsUart0(tcb[p].name);
                for (i = 0; i < 16 - strlen(tcb[p].name); i++)
                    putsUart0(" ");
                putsUart0("\t|");

                f2s((tcb[p].cpuPercentage));
                for (i = 0; i < 14; i++)
                    putsUart0(" ");
                putcUart0('|');
                if (tcb[p].state == 0)
                    putsUart0("invalid");
                else if (tcb[p].state == 1)
                    putsUart0("unrun  ");
                else if (tcb[p].state == 2)
                    putsUart0("ready  ");
                else if (tcb[p].state == 3)
                    putsUart0("blocked");
                else if (tcb[p].state == 4)
                    putsUart0("delayed");
                else if (tcb[p].state == 5)
                    putsUart0("run    ");
                putcUart0('\n');

                putcUart0('\r');
            }
            putcUart0('\n');
        }

        else if (strcmp(final[0], "ipcs") == 0)
        {
            putsUart0("\n\rSemaphore name\t|\tCount\t|\tQueue\t|\tQueueSize\r");
            putsUart0("\n----------------------------------------------------------------\r");
            for (a = 0; a < 4; a++)
            {
                putsUart0("\n");
                putsUart0(che[a]);
                putsUart0("\t|\t");
                i2s(semaphores[a].count);
                putsUart0("\t|\t");
                i2s(semaphores[a].queueSize);
                putsUart0("\t|\t");
                int z, flag = 0;
                i2s(semaphores[a].processQueue[0]);
                flag = 0;
                putsUart0("\n");
                putsUart0("\r");
            }
        }

        else if (strcmp(final[0], "reboot") == 0)
        {
            putcUart0('\n');
            putcUart0('\r');

            putsUart0(" reboot");
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ ;

        }
        else if (strcmp(final[0], "kill") == 0)
        {
            putcUart0('\n');
            putcUart0('\r');
            uint8_t x;
            uint8_t flag1 = 0;
            for (x = 0; x < taskCount; x++)
            {
                if (atoi(final[1]) == tcb[x].pid){
                    destroyThread((_fn) tcb[x].pid);
                    flag1++;
                    putsUart0("\n\rThe thread is killed successfully");
                }
            }
            if( flag1 == 0)
                putsUart0("\n\rThread cannot be found. Please enter a valid task");
        }

        else if (com[len1-1] == '&')
        {
            uint8_t k,c,d;
            char* che1[10] = { "idle", "lengthyfn", "uncoop", "oneshot", "flash4hz", "readkeys", "debounce","shell", "important"};
            char* check;
            for( c = 0; c<= len1-1 ; c++){
                if(com[c] == '&')
                    com[c]= '\0';
            }
            for( c = 0; c< 10 ; c++){
                if(strcmp(che1[c],com) == 0){
                    free(check);
                    check = che1[c];
                    break;
                }
                else
                    free(check);
            }
            for (k = 0; k < MAX_TASKS; k++)
            {
                if (strcmp(check, che1[k]) == 0)
                {
                    if (strcmp(che1[k], "idle") == 0)
                    {
                        createThread(idle1, "Idle", tcb[k].priority);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "lengthyfn") == 0)
                    {
                        createThread(lengthyFn, "LengthyFn", tcb[k].priority);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "oneshot") == 0)
                    {
                        createThread(oneshot, "OneShot", tcb[k].priority);
                        post(flashReq);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "readkeys") == 0)
                    {
                        createThread(readKeys, "ReadKeys", tcb[k].priority);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "debounce") == 0)
                    {
                        createThread(debounce, "Debounce", tcb[k].priority);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "important") == 0)
                    {
                        createThread(important, "Important", tcb[k].priority);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "uncoop") == 0)
                    {
                        createThread(uncooperative, "Uncoop",
                                     tcb[k].priority);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "shell") == 0)
                    {
                        createThread(shell, "Shell", tcb[k].priority);
                        taskCount--;
                    }
                    if (strcmp(che1[k], "flash4hz") == 0)
                    {
                        createThread(flash4Hz, "Flash4Hz", tcb[k].priority);
                        taskCount--;
                    }
                    putcUart0('\n');
                    putcUart0('\r');
                    putsUart0("The thread is created\n\r ");
                    putcUart0('\n');
                    putcUart0('\r');
                }
            }
        }

// REQUIRED: add processing for the shell commands through the UART here
    }
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

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
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

// Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

// Add required idle process
    ok = createThread(idle1, "Idle", 15);
//ok =  createThread(idle2, "Idle", 15);

// Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12);
    ok &= createThread(flash4Hz, "Flash4Hz", 4);
    ok &= createThread(oneshot, "OneShot", 4);
    ok &= createThread(readKeys, "ReadKeys", 12);
    ok &= createThread(debounce, "Debounce", 12);
    ok &= createThread(important, "Important", 0);
    ok &= createThread(uncooperative, "Uncoop", 10);
    ok &= createThread(shell, "Shell", 8);

    NVIC_ST_CTRL_R = 7;               //Disable SysTick during configuration
    NVIC_ST_RELOAD_R = 0x00009C40;      //Reload value configured for 1ms
    NVIC_ST_CURRENT_R = 0;              //Current value is reset to 0

// Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
