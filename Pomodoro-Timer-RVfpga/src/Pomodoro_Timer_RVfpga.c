#include "psp_api.h"
#include "bsp_external_interrupts.h"
#include "psp_ext_interrupts_eh1.h"
#include "bsp_timer.h"
#include "bsp_printf.h"
#include <stdbool.h>

// 7 segment display registers
#define SegEn_ADDR      0x80001038
#define SegDig_ADDR     0x8000103C
#define GPIO_LEDs 0x80001404 // LED base address

// GPIO registers
#define GPIO_SWs        0x80001400
#define GPIO_LEDs       0x80001404
#define GPIO_INOUT      0x80001408
#define RGPIO_INTE      0x8000140C
#define RGPIO_PTRIG     0x80001410
#define RGPIO_CTRL      0x80001418
#define RGPIO_INTS      0x8000141C

// Buttons
#define GPIO_BTN 0x80001800 // Button base address
#define PB_BTNC 0x0001 // BTN Center
#define PB_BTNU 0x0002 // BTN Up
#define PB_BTNL 0x0004 // BTN Left
#define PB_BTNR 0x0008 // BTN Right
#define PB_BTND 0x000F // BTN Down

// Timer Registers
#define RPTC_CNTR       0x80001200
#define RPTC_HRC        0x80001204
#define RPTC_LRC        0x80001208
#define RPTC_CTRL       0x8000120c

#define Select_INT      0x80001018

// Pomodoro Macros
#define Min_focus_time 25
#define Max_focus_time 60 
#define Min_break_time 5
#define Max_break_time 15

#define READ_GPIO(dir) (*(volatile unsigned *)dir)
#define WRITE_GPIO(dir, value) { (*(volatile unsigned *)dir) = (value); }

// Declaring prototype functions
void update_display_mmss(int minutes, int seconds);
void set_demo_mode(bool enable_demo);
void start_Pomodoro_timer(void);
void stop_Pomodoro_timer(void);


// Global variables
bool demo_mode = false; // Set true for demo mode (shorter intervals of time)


// Focus session: initial time: 25 minutes 
int focus_minutes = 25;  // 25 minutes
int focus_seconds =  60; // 60 seconds

// Break session: initial time: 5 minutes
int break_minutes = 5;   //  5 minutes
int break_seconds =  60; // 60 seconds


void set_demo_mode(bool enable_demo)
{
    if(enable_demo == true) {
        focus_minutes = 2;  // 2 minutes
        focus_seconds = 60; // 60 seconds

        break_minutes = 1;  // 1 minute
        break_seconds = 5;  // 5 seconds
    }
    return;
}; 




extern D_PSP_DATA_SECTION D_PSP_ALIGNED(1024) pspInterruptHandler_t G_Ext_Interrupt_Handlers[8];

void GPIO_ISR(void)
{
  unsigned int i;


  i = M_PSP_READ_REGISTER_32(GPIO_LEDs);             /* RGPIO_OUT */
  i = !i;                                            /* Invert the LEDs */
  i = i & 0x1;                                       /* Keep only the right-most LED */
  M_PSP_WRITE_REGISTER_32(GPIO_LEDs, i);             /* RGPIO_OUT */

  /* Clear GPIO interrupt */
  M_PSP_WRITE_REGISTER_32(RGPIO_INTS, 0x0);           /* RGPIO_INTS */

  /* Stop the generation of the specific external interrupt */
  bspClearExtInterrupt(4);
}

void PTC_ISR(void)  // Interrupcion del timer
{
  // Setting up the timer 
  M_PSP_WRITE_REGISTER_32(RPTC_CNTR, 0x0);
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40); // Clean interrupt
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x31); // re-enable timer

  /* Incrementa el contador de los displays 7 segmentos */
  SegDisplCount++;
  


  // Clear PTC interrupt
  bspClearExtInterrupt(3);
}

void update_display_mmss(int minutes, int seconds)
{
  M_PSP_WRITE_REGISTER_32(SegDig_ADDR, real_time_timer);      // Shows in display
}




void DefaultInitialization(void)
{
  u32_t uiSourceId;

  /* Register interrupt vector */
  pspInterruptsSetVectorTableAddress(&M_PSP_VECT_TABLE);

  /* Set external-interrupts vector-table address in MEIVT CSR */
  pspExternalInterruptSetVectorTableAddress(G_Ext_Interrupt_Handlers);

  /* Put the Generation-Register in its initial state (no external interrupts are generated) */
  bspInitializeGenerationRegister(D_PSP_EXT_INT_ACTIVE_HIGH);

  for (uiSourceId = D_BSP_FIRST_IRQ_NUM; uiSourceId <= D_BSP_LAST_IRQ_NUM; uiSourceId++)
  {
    /* Make sure the external-interrupt triggers are cleared */
    bspClearExtInterrupt(uiSourceId);
  }

  /* Set Standard priority order */
  pspExtInterruptSetPriorityOrder(D_PSP_EXT_INT_STANDARD_PRIORITY);

  /* Set interrupts threshold to minimal (== all interrupts should be served) */
  pspExtInterruptsSetThreshold(M_PSP_EXT_INT_THRESHOLD_UNMASK_ALL_VALUE);

  /* Set the nesting priority threshold to minimal (== all interrupts should be served) */
  pspExtInterruptsSetNestingPriorityThreshold(M_PSP_EXT_INT_THRESHOLD_UNMASK_ALL_VALUE);
}


void ExternalIntLine_Initialization(u32_t uiSourceId, u32_t priority, pspInterruptHandler_t pTestIsr)
{
  /* Set Gateway Interrupt type (Level) */
  pspExtInterruptSetType(uiSourceId, D_PSP_EXT_INT_LEVEL_TRIG_TYPE);

  /* Set gateway Polarity (Active high) */
  pspExtInterruptSetPolarity(uiSourceId, D_PSP_EXT_INT_ACTIVE_HIGH);

  /* Clear the gateway */
  pspExtInterruptClearPendingInt(uiSourceId);

  /* Set IRQ4 priority */
  pspExtInterruptSetPriority(uiSourceId, priority);
    
  /* Enable IRQ4 interrupts in the PIC */
  pspExternalInterruptEnableNumber(uiSourceId);

  /* Register ISR */
  G_Ext_Interrupt_Handlers[uiSourceId] = pTestIsr;
}


void GPIO_Initialization(void)
{
  /* Configure LEDs and Switches */
  M_PSP_WRITE_REGISTER_32(GPIO_INOUT, 0xFFFF);        /* GPIO_INOUT */
  M_PSP_WRITE_REGISTER_32(GPIO_LEDs, 0x0);            /* GPIO_LEDs */

  /* Configure GPIO interrupts */
  M_PSP_WRITE_REGISTER_32(RGPIO_INTE, 0x10000);       /* RGPIO_INTE */
  M_PSP_WRITE_REGISTER_32(RGPIO_PTRIG, 0x10000);      /* RGPIO_PTRIG */
  M_PSP_WRITE_REGISTER_32(RGPIO_INTS, 0x0);           /* RGPIO_INTS */
  M_PSP_WRITE_REGISTER_32(RGPIO_CTRL, 0x1);           /* RGPIO_CTRL */
}


void PTC_Initialization(void)
{
// 50 MHz clock (20 ns period). For 1 second period, RPTC_LRC = 1/20ns = 50'000'000
M_PSP_WRITE_REGISTER_32(RPTC_LRC, 50000000);
M_PSP_WRITE_REGISTER_32(RPTC_CNTR, 0x0);
M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40);
M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x31);
}


int main(void)
{
  /* INITIALIZE THE INTERRUPT SYSTEM */
  DefaultInitialization();                            /* Default initialization */
  pspExtInterruptsSetThreshold(5);                    /* Set interrupts threshold to 5 */
  set_demo_mode(demo_mode); // Set demo mode if enabled


  /* INITIALIZE INTERRUPT LINES IRQ3 AND IRQ4 */
  ExternalIntLine_Initialization(4, 6, GPIO_ISR);     /* Initialize line IRQ4 with a priority of 6. Set GPIO_ISR as the Interrupt Service Routine */
  ExternalIntLine_Initialization(3, 6, PTC_ISR);      /* Initialize line IRQ3 with a priority of 6. Set PTC_ISR as the Interrupt Service Routine */
  M_PSP_WRITE_REGISTER_32(Select_INT, 0x3);           /* Connect the GPIO interrupt to the IRQ4 interrupt line and the PTC interrupt to the IRQ3 line*/

  /* INITIALIZE THE PERIPHERALS */
  GPIO_Initialization();                              /* Initialize the GPIO */
  PTC_Initialization();                               /* Initialize the Timer */
  M_PSP_WRITE_REGISTER_32(SegEn_ADDR, 0x0);           /* Initialize the 7-Seg Displays */

  /* ENABLE INTERRUPTS */
  pspInterruptsEnable();                              /* Enable all interrupts in mstatus CSR */
  M_PSP_SET_CSR(D_PSP_MIE_NUM, D_PSP_MIE_MEIE_MASK);  /* Enable external interrupts in mie CSR */

  while (1) {
    
    int button_state = READ_GPIO(GPIO_BTN); // Read buttons state
    bool start_timer = false; // Indicates if timer has started
    
    if (button_state & PB_BTNC) {
      // Center button pressed: Reset the display count
      // PTC_Initialization(); // Starts counting 

      M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x31);
    }
    

  }
}
