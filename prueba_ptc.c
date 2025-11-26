#include <stdint.h>

#define GPIO_LEDs  (*((volatile uint32_t*)0x80001404))
#define GPIO_INOUT (*((volatile uint32_t*)0x80001408))
#define AUDIO_CTRL (*((volatile uint32_t*)0x80001600))
#define AUDIO_DATA (*((volatile uint32_t*)0x80001604))

int main(void) {
    // LEDs como salida
    GPIO_INOUT = 0xFFFF;
    GPIO_LEDs = 0x0001;

    // Habilitar módulo de audio (asumimos bit 0 = enable)
    AUDIO_CTRL = 1;

    // Generar onda cuadrada simple
    int16_t sample_pos = 20000;   // amplitud positiva
    int16_t sample_neg = -20000;  // amplitud negativa
    int toggle = 0;

    while (1) {
        if (toggle) {
            AUDIO_DATA = sample_pos;
        } else {
            AUDIO_DATA = sample_neg;
        }
        toggle ^= 1;

        // Delay ajusta la frecuencia del tono (prueba distintos valores)
        for (volatile int i = 0; i < 5000; i++);

        // Parpadear LEDs para ver que el loop está vivo
        static int cnt = 0;
        if (++cnt > 1000) {
            cnt = 0;
            GPIO_LEDs ^= 0x0003;
        }
    }
}
