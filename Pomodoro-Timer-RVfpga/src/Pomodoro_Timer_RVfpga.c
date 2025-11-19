/*
Estudiantes: 
- Josué María Jiménez Ramírez, C13987
- Gabriel Vega Chavez, C08344
*/ 

#include <bsp_printf.h>
#include <bsp_mem_map.h>
#include <bsp_version.h>
#include <stdint.h>
#include <psp_api.h>

// Memory-mapped I/O addresses
#define GPIO_SWs 0x80001400
#define GPIO_LEDs 0x80001404
#define GPIO_INOUT 0x80001408
#define GPIO_BTN 0x80001800
#define GPIO2_INOUT 0x80001808

// --Push buttons mask
#define PB_BTNC 0x0001
#define PB_BTNU 0x0002
#define PB_BTNL 0x0004
#define PB_BTNR 0x0008
#define PB_BTND 0x0010

#define READ_GPIO(dir) (*(volatile unsigned *)dir)
#define WRITE_GPIO(dir, value) { (*(volatile unsigned *)dir) = (value); }

void delay_ciclos(volatile unsigned ciclos);

int main(void)
{
    // Configurar dirección de GPIO
    WRITE_GPIO(GPIO_INOUT, 0xFFFF);  // LEDs como salidas
    WRITE_GPIO(GPIO2_INOUT, 0x0000); // Botones como entradas

    while (1) {
        int valor_inicial = 0b0000000000000001;
        int led_totales = 0;
        int count_speed = 5000000; // Velocidad normal
        int reiniciar = 0;

        for (int i = 0; i <= 15; i++) {
            // Leer el estado de los botones
            int botones = READ_GPIO(GPIO_BTN);
            
            // Verificar botones específicos
            if (botones & PB_BTND) { // Botón CENTER presionado
                count_speed = 500000; // Velocidad rápida
            } else {
                count_speed = 5000000; // Velocidad normal
            }

            // Actualizar LEDs
            int led_actual_a_encender = valor_inicial << i;
            led_totales = led_actual_a_encender | led_totales;
            WRITE_GPIO(GPIO_LEDs, led_totales);

            // Delay con la velocidad actual
            delay_ciclos(count_speed);

            // Verificar si se debe reiniciar (después del delay)
            botones = READ_GPIO(GPIO_BTN); // Leer nuevamente
            if (botones & PB_BTNU) { // Botón UP para reiniciar
                reiniciar = 1;
                break; // Salir del for loop
            }
        }

        // Si se solicitó reinicio, volver al inicio del while
        if (reiniciar) {
            WRITE_GPIO(GPIO_LEDs, 0x0000);
            delay_ciclos(1000000);
            continue; // Volver al inicio del while(1)
        }

        // Si llegó aquí, completó el ciclo normal
        delay_ciclos(1000000);
        WRITE_GPIO(GPIO_LEDs, 0x0000);
        delay_ciclos(1000000);
    }
}

void delay_ciclos(volatile unsigned ciclos) {
    while (ciclos--) {
        __asm__("nop");
    }
}