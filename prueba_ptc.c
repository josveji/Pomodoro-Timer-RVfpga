#include <stdint.h>

#define WRITE(addr) (*(volatile uint32_t*)(addr))

// GPIOs
#define GPIO_SWS       0x80001400
#define GPIO_LEDs      0x80001404
#define GPIO_INOUTREG  0x80001410

// wbpwmaudio - Wishbone PWM Audio Controller
#define AUDIO_DATA     0x800012C0    // Addr[0] - Sample data
#define AUDIO_RELOAD   0x8000000    // Addr[1] - Sample rate control

int main(void){
    WRITE(GPIO_INOUTREG) = 0xFFFF;

    while (1) {
        uint32_t sws = WRITE(GPIO_SWS);
        sws >>= 16;
        WRITE(GPIO_LEDs) = sws;

        // Generar audio basado en switches
        uint16_t audio_sample = (sws & 0x1F) << 11; // 5 bits → 16 bits
        WRITE(AUDIO_DATA) = audio_sample;
        
        // Controlar frecuencia/tempo con reload rate
        uint32_t sample_rate = (sws & 0x1F) + 15;
        WRITE(AUDIO_RELOAD) = sample_rate;
    }
}
