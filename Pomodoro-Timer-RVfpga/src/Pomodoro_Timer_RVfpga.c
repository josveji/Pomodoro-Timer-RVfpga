/*
Estudiantes: 
- Josué María Jiménez Ramírez, C13987
- Gabriel Vega Chavez, C08344
*/ 

#include <stdint.h>
#include <stdbool.h>

#include <psp_api.h>                        // <-- mover arriba (define pspInterruptHandler_t)
#include "psp_ext_interrupts_eh1.h"
#include "bsp_external_interrupts.h"
#include <bsp_printf.h>
#include <bsp_mem_map.h>
#include <bsp_version.h>
#include "bsp_timer.h"

// Memory-mapped I/O addresses
#define GPIO_SWs    0x80001400
#define GPIO_LEDs   0x80001404
#define GPIO_INOUT  0x80001408
#define GPIO_BTN    0x80001800 // Buttons input address
#define GPIO2_INOUT 0x80001808 // GPIO that corresponds to buttons
#define RGPIO_INTS  0x8000141C

// Push buttons masks
#define PB_BTNC 0x0001 // Center button
#define PB_BTNU 0x0002 // Up button
#define PB_BTNL 0x0004 // Left button
#define PB_BTNR 0x0008 // Right button
#define PB_BTND 0x0010 // Down button

// 7 segment display registers
#define SegEn_ADDR      0x80001038 // Segment enable address
#define SegDig_ADDR     0x8000103C // Segment digit address
#define GPIO_LEDs       0x80001404 // LED base address

// Pomodoro Macros (time in minutes)
#define DEFAULT_WORK_MINUTES 25
#define MAX_WORK_MINUTES     60
#define MIN_WORK_MINUTES     1

#define DEFAULT_BREAK_MINUTES 5
#define MAX_BREAK_MINUTES     20
#define MIN_BREAK_MINUTES     1

// GPIO Registers
#define RGPIO_INTE      0x8000140C
#define RGPIO_PTRIG     0x80001410
#define RGPIO_CTRL      0x80001418
#define RGPIO_INTS      0x8000141C

// Timer Registers
#define RPTC_CNTR       0x80001200
#define RPTC_HRC        0x80001204
#define RPTC_LRC        0x80001208
#define RPTC_CTRL       0x8000120c

#define Select_INT      0x80001018

// State Macros
#define ST_CONFIG 0xC
#define ST_WORK   0xF
#define ST_BREAK  0xB

// LED combiantions 
#define LED_CONFIG_WRK 0b1111010000000000
#define LED_CONFIG_BRK 0b1111011000000000
#define LED_WORK       0b1110000000000000
#define LED_BREAK      0b1100000000000000

// Define read and write macros
#define READ_GPIO(dir) (*(volatile unsigned *)dir)
#define WRITE_GPIO(dir, value) { (*(volatile unsigned *)dir) = (value); }
extern D_PSP_DATA_SECTION D_PSP_ALIGNED(1024) pspInterruptHandler_t G_Ext_Interrupt_Handlers[8];

// Instancing functions
void update_display_mmss(int minutes, int seconds, int state);
uint32_t create_time_packet(int minutes, int seconds, int state);
void delay_cyc(volatile unsigned ciclos);
void stop_Pomodoro_Timer(void);
void start_Pomodoro_Timer(void);
void reset_Pomodoro_Timer(void);
void update_led(int led_combination);
void blink_leds(int times, int delay_cycles);


// Global Variables
volatile int work_minutes = DEFAULT_WORK_MINUTES;
volatile int break_minutes = DEFAULT_BREAK_MINUTES;
volatile int remaining_seconds = 0;
volatile int current_state = ST_CONFIG;

// ===================== Functions =====================
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

void PTC_ISR(void){  // Interrupcion del timer
  // Setting up the timer 
  M_PSP_WRITE_REGISTER_32(RPTC_CNTR, 0x0);
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40); // Clean interrupt
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x31); // re-enable timer

  if (current_state == ST_BREAK || current_state == ST_WORK){
    if (remaining_seconds > 0){
      remaining_seconds--;
    }

    int mm = remaining_seconds / 60; // Caluculate minutes
    int ss = remaining_seconds % 60; // Calculate seconds

    update_display_mmss(mm, ss, current_state);

    // When the timer reaches 0, switch states
    if(remaining_seconds == 0){
      if (current_state == ST_WORK){
        current_state = ST_BREAK;  // Switch to break mode
        remaining_seconds = break_minutes * 60;
        update_display_mmss(break_minutes, 0, current_state);
      } 
      else if (current_state == ST_BREAK){
        blink_leds(10, 1000000); // Blink LEDs to signal end of break
        current_state = ST_CONFIG; // Switch to config mode
        work_minutes = DEFAULT_WORK_MINUTES;   // Set default times
        break_minutes = DEFAULT_BREAK_MINUTES; // Set default times
        stop_Pomodoro_Timer();
        update_display_mmss(work_minutes, 0, current_state);
        update_led(LED_CONFIG_WRK);
      }
    }
  }
  
  bspClearExtInterrupt(3); // Clear PTC interrupt
}

void DefaultInitialization(void){
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

// Start the Pomodoro Timer
void start_Pomodoro_Timer(void){
  M_PSP_WRITE_REGISTER_32(RPTC_CNTR, 0x0);
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40);
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x31);
}

// Stop the Pomodoro Timer
void stop_Pomodoro_Timer(void){
  M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40); // Stop timer
}

// Reset the Pomodoro Timer
void reset_Pomodoro_Timer(void){
    M_PSP_WRITE_REGISTER_32(RPTC_CTRL, 0x40); // Stop timer
    remaining_seconds = 0;
    current_state = ST_CONFIG;
    work_minutes = DEFAULT_WORK_MINUTES;
    break_minutes = DEFAULT_BREAK_MINUTES;
    update_display_mmss(work_minutes, 0, current_state);
}


// ===================== Main Function =====================

int main(void)
{
    // Initializations
    DefaultInitialization(); 
    WRITE_GPIO(GPIO_INOUT, 0xFFFF);  // LEDs as outputs
    WRITE_GPIO(GPIO2_INOUT, 0x0000); // Buttons as inputs
    WRITE_GPIO(SegEn_ADDR, 0xE0);    // Enables the least significant 5 digits (0b11100000) (7-seg display)

    pspExtInterruptsSetThreshold(5);   

    /* INITIALIZE INTERRUPT LINES IRQ3 AND IRQ4 */
    ExternalIntLine_Initialization(4, 6, GPIO_ISR);     /* Initialize line IRQ4 with a priority of 6. Set GPIO_ISR as the Interrupt Service Routine */
    ExternalIntLine_Initialization(3, 6, PTC_ISR);      /* Initialize line IRQ3 with a priority of 6. Set PTC_ISR as the Interrupt Service Routine */
    M_PSP_WRITE_REGISTER_32(Select_INT, 0x3);           /* Connect the GPIO interrupt to the IRQ4 interrupt line and the PTC interrupt to the IRQ3 line*/

    /* INITIALIZE THE PERIPHERALS */
    GPIO_Initialization();                              /* Initialize the GPIO */
    PTC_Initialization();                               /* Initialize the Timer */

    // Default display and LED state
    update_display_mmss(work_minutes, 0, ST_CONFIG); // 25:00 show default
    update_led(LED_CONFIG_WRK);

    /* ENABLE INTERRUPTS */
    pspInterruptsEnable();                              /* Enable all interrupts in mstatus CSR */
    M_PSP_SET_CSR(D_PSP_MIE_NUM, D_PSP_MIE_MEIE_MASK);  /* Enable external interrupts in mie CSR */

    unsigned int last_buttons = 0; // Variable to store the last button state


    while (1) {
        
        int button_state = READ_GPIO(GPIO_BTN); // Reads the current button state
        unsigned int pressed = (button_state) & ~(last_buttons); // Detects newly pressed buttons

        if (current_state == ST_CONFIG) {
            // In configuration state, adjust work and break minutes
            if (pressed & PB_BTNU) { // UP button increases work time
                update_display_mmss(work_minutes, 0, current_state);
                update_led(LED_CONFIG_WRK);

                if (work_minutes < MAX_WORK_MINUTES) {
                    work_minutes++;
                    update_display_mmss(work_minutes, 0, current_state);
                }
                delay_cyc(500000); // Debounce delay
            }
            if (pressed & PB_BTND) { // DOWN button decreases work time
                update_display_mmss(work_minutes, 0, current_state);
                update_led(LED_CONFIG_WRK);

                if (work_minutes > MIN_WORK_MINUTES) {
                    work_minutes--;
                    update_display_mmss(work_minutes, 0, current_state);
                }
                delay_cyc(500000); // Debounce delay
            }
            if (pressed & PB_BTNR) { // RIGHT button increases break time
                update_display_mmss(break_minutes, 0, current_state);
                update_led(LED_CONFIG_BRK);

                if (break_minutes < MAX_BREAK_MINUTES) {
                    break_minutes++;
                    update_display_mmss(break_minutes, 0, current_state);
                }
                delay_cyc(500000); // Debounce delay
            }


            if (pressed & PB_BTNL) { // LEFT button decreases break time
                update_display_mmss(break_minutes, 0, current_state);
                update_led(LED_CONFIG_BRK);

                if (break_minutes > MIN_BREAK_MINUTES) {
                    break_minutes--;
                    update_display_mmss(break_minutes, 0, current_state);
                }
                delay_cyc(500000); // Debounce delay
            }
            
            if (pressed & PB_BTNC) { // CENTER button starts the timer
                current_state = ST_WORK;
                remaining_seconds = work_minutes * 60;
                start_Pomodoro_Timer();
                delay_cyc(500000); // Debounce delay
            }
        }

        else if (current_state == ST_WORK) {
            update_led(LED_WORK);
        } 
        
        else if (current_state == ST_BREAK) {
            update_led(LED_BREAK);
        }

        last_buttons = button_state; // Update last button state
        delay_cyc(150000); /* 50 ms */
    }
}




// ===================== Pomodoro Functions =====================

void delay_cyc(volatile unsigned ciclos){
    while (ciclos--) {
        __asm__("nop");
    }
}

// This function updates the 7-segment display with the given minutes, seconds, and state
void update_display_mmss(int minutes, int seconds, int state){
    // Clamping values to valid ranges
    if (minutes < 0) minutes = 0;
    if (minutes > 99) minutes = 99;
    if (seconds < 0) seconds = 0;
    if (seconds > 59) seconds = 59;

    uint32_t time_packet = create_time_packet(minutes, seconds, state);
    WRITE_GPIO(SegDig_ADDR, time_packet);   // Enviar el paquete de tiempo
}

// This function creates a time packet for the 7-segment display
uint32_t create_time_packet(int minutes, int seconds, int state){
    // Extracting digits
    int d_minutes = (minutes / 10) % 10; // tens place  (minutes)
    int u_minutes = (minutes % 10);      // units place (minutes)
    int d_seconds = (seconds / 10) % 10; // tens place  (seconds)
    int u_seconds = (seconds % 10);      // units place (seconds)

    // Creating the time packet
    uint32_t time_packet = (
        (state     << 16) |
        (d_minutes << 12) |
        (u_minutes << 8 ) |   
        (d_seconds << 4 ) |
        (u_seconds << 0 )
    );

  return time_packet;
}

void update_led(int led_combination){
    WRITE_GPIO(GPIO_LEDs, led_combination);
}

void blink_leds(int times, int delay_cycles){
    for (int i = 0; i < times; i++){
        WRITE_GPIO(GPIO_LEDs, 0x0000); // Turn off LEDs
        delay_cyc(delay_cycles);
        WRITE_GPIO(GPIO_LEDs, 0xFFFF); // Turn on LEDs
        delay_cyc(delay_cycles);
    }
}