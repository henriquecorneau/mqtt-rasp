#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>  // Para usar uint8_t, uint16_t, etc.

// Definição dos pinos usados para o joystick e LEDs
#define VRX 26          // Pino de leitura do eixo X do joystick (conectado ao ADC)
#define VRY 27          // Pino de leitura do eixo Y do joystick (conectado ao ADC)
#define ADC_CHANNEL_0 0 // Canal ADC para o eixo X do joystick
#define ADC_CHANNEL_1 1 // Canal ADC para o eixo Y do joystick
#define SW 22          // Pino de leitura do botão do joystick

void setup_joystick();
void joystick_read_axis(uint16_t *vrx_value, uint16_t *vry_value, uint16_t *sw_value);


#endif