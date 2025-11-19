#include "psp_api.h"
#include "bsp_external_interrupts.h"
#include "psp_ext_interrupts_eh1.h"
#include "bsp_timer.h"
#include "bsp_printf.h"
#include <stdbool.h>
#include <stdint.h>

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
#define GPIO2_INOUT 0x80001808 // GPIO2 corresponds to buttons
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
#define DEFAULT_WORK_TIME 25 // in minutes
#define DEFAULT_BREAK_TIME 5 // in minutes

#define MIN_WORK_TIME 5
#define MAX_WORK_TIME 60 
#define MIN_BREAK_TIME 5
#define MAX_BREAK_TIME 15

// Declaring states
#define STATE_CONFIG 0
#define STATE_WORK   1
#define STATE_BREAK  2

#define READ_GPIO(dir) (*(volatile unsigned *)dir)
#define WRITE_GPIO(dir, value) { (*(volatile unsigned *)dir) = (value); }

// Global variables
bool demo_mode = false; // Set true for demo mode (shorter intervals of time)
volatile int work_minutes = DEFAULT_WORK_TIME;
volatile int break_minutes = DEFAULT_BREAK_TIME;
volatile int remaining_seconds = 0;
volatile int mode_state = STATE_CONFIG;

extern D_PSP_DATA_SECTION D_PSP_ALIGNED(1024) pspInterruptHandler_t G_Ext_Interrupt_Handlers[8];

// Declaring prototype functions
void update_display_mmss(int minutes, int seconds, int state);
//void set_demo_mode(bool enable_demo);
//void start_Pomodoro_timer(void);
void stop_Pomodoro_timer(void);
static void delay_cycles(volatile uint32_t cycles);


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

  if (mode_state == STATE_WORK || mode_state == STATE_BREAK){
    if (remaining_seconds > 0){
      remaining_seconds--;
    }

    int mm = remaining_seconds / 60; // Caluculate minutes
    int ss = remaining_seconds % 60; // Calculate seconds

    update_display_mmss(mm, ss, mode_state);

    if(remaining_seconds == 0){
      if (mode_state == STATE_WORK){
        // Switch to break mode
        mode_state = STATE_BREAK;
        remaining_seconds = break_minutes * 60;
        update_display_mmss(break_minutes, 0, mode_state);
      } else if (mode_state == STATE_BREAK){
        // Switch to config mode
        mode_state = STATE_CONFIG;
        stop_Pomodoro_timer();
      }
    }
  }
  
  // Clear PTC interrupt
  bspClearExtInterrupt(3);
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


void PTC_Initialization(void){
  // 50 MHz clock (20 ns period). For 1 second period, RPTC_LRC = 1/20ns = 50'000'000
  M_PSP_WRITE_REGISTER_32(RPTC_LRC, 50000000);
  M_PSP_WRITE_REGISTER_32(RPTC_CNTR, 0x0);
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40);
  //M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x31);
}

void start_Pomodoro_timer(void){
  M_PSP_WRITE_REGISTER_32(RPTC_CNTR, 0x0);
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40);
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x31);
}

void stop_Pomodoro_timer(void){
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40); // Stop timer
}

void update_display_mmss(int minutes, int seconds, int state){
  if (minutes < 0) minutes = 0;
  if (minutes > 99) minutes = 99;
  if (seconds < 0) seconds = 0;
  if (seconds > 59) seconds = 59;

  int state_bits; 
  switch(state){
    case STATE_CONFIG:
      state_bits = 0xC;
      break;
    case STATE_WORK:
      state_bits = 0xF;
      break;
    case STATE_BREAK:
      state_bits = 0xB;
      break;
    default:
      state_bits = 0xC;
      break;
  }

  // Separting digits
  int d_minutes = (minutes / 10) % 10; // tens place  (minutes)
  int u_minutes = (minutes % 10);      // units place (minutes)
  int d_seconds = (seconds / 10) % 10; // tens place  (seconds)
  int u_seconds = (seconds % 10);      // units place (seconds)

  // Putting values together for display (State|MM|SS)
  uint32_t time_packet = (
    (state_bits << 16) |
    (d_minutes << 12) |
    (u_minutes << 8 ) |   
    (d_seconds << 4 ) |
    (u_seconds << 0 )
  );


  M_PSP_WRITE_REGISTER_32(SegDig_ADDR, time_packet);      // Shows in display
}


int main(void)
{
  /* INITIALIZE THE INTERRUPT SYSTEM */
  DefaultInitialization();          
  WRITE_GPIO(GPIO2_INOUT, 0x0000); // Botones como entradas                  /* Default initialization */
  pspExtInterruptsSetThreshold(5);                    /* Set interrupts threshold to 5 */
  //set_demo_mode(demo_mode); // Set demo mode if enabled


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

  update_display_mmss(work_minutes, 0x0, 0x0);

  unsigned int last_buttons = 0;

  while (1) {
    
    int button_state = READ_GPIO(GPIO_BTN); // Read buttons state

    unsigned int pressed = (button_state) & ~(last_buttons);

    bool start_timer = false; // Indicates if timer has started
    
    if (mode_state == STATE_CONFIG) {
      /* UP: increase work_minutes +5 */
      if (pressed & PB_BTNU) {
        if (work_minutes + 5 <= MAX_WORK_TIME) {
          work_minutes += 5;
        }
        update_display_mmss(work_minutes, 0, mode_state);
        delay_cycles(500000); /* pequeño antirrebote ~200ms */
      }

      /* DOWN: decrease work_minutes -5 */
      if (pressed & PB_BTND) {
        if (work_minutes - 5 >= MIN_WORK_TIME) {
          work_minutes -= 5;
        }
        update_display_mmss(work_minutes, 0, mode_state);
        delay_cycles(500000);
      }

      /* RIGHT: increase break_minutes +5 */
      if (pressed & PB_BTNR) {
        if (break_minutes + 5 <= MAX_BREAK_TIME) {
          break_minutes += 5;
        }
        update_display_mmss(break_minutes, 0, mode_state); /* muestra temporalmente break en pantalla; puedes cambiar */
         delay_cycles(500000);
      }

      /* LEFT: decrease break_minutes -5 */
      if (pressed & PB_BTNL) {
        if (break_minutes - 5 >= MIN_BREAK_TIME) {
          break_minutes -= 5;
        }
        update_display_mmss(break_minutes, 0, mode_state);
        delay_cycles(500000);
      }

      /* CENTER: iniciar sesión de trabajo */
      if (pressed & PB_BTNC) {
        mode_state = STATE_WORK;
        remaining_seconds = work_minutes * 60; /* cargar tiempo en segundos */
        update_display_mmss(work_minutes, 0, mode_state);
        start_Pomodoro_timer();
        delay_cycles(500000);
      }

      } else {
        /* Executing Work/Break mode*/
      }

      /* Save last button state */
      last_buttons = button_state;

      /* Wait */
      delay_cycles(150000); /* 50 ms */
    

  }
}

static void delay_cycles(volatile uint32_t cycles) {
    while(cycles--) {
        __asm__("nop");
    }
}
