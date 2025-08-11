#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>                 // Para funções de string como strlen()
#include "pico/cyw43_arch.h"        // Driver WiFi para Pico W
#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"

#include "control_joy.h"

#include "wifi_conn.h"      // Funções personalizadas de conexão WiFi
#include "mqtt_comm.h"      // Funções personalizadas para MQTT
#include "xor_cipher.h"     // Funções de cifra XOR

#define LED_PIN_G 11
#define TEMPERATURE_SENSOR 4

void setup();
void vread_temp(void *pvParameters);
void vjoystick_direction(void *pvParameters);

void main()
{
    setup();
    setup_joystick();
    // Conecta à rede WiFi
    // Parâmetros: Nome da rede (SSID) e senha
    connect_to_wifi("JHC", "jhc302010");

    // Configura o cliente MQTT
    // Parâmetros: ID do cliente, IP do broker, usuário, senha
    mqtt_setup("henrique", "200.137.1.176", "desafio04", "desafio04.laica");

    xTaskCreate(vread_temp, "Read Temp Task", 512, NULL, 1, NULL);
    xTaskCreate(vjoystick_direction, "Joystick Direction Task", 256, NULL, 1, NULL);

    vTaskStartScheduler();
}

void setup() {
    stdio_init_all();

    gpio_init(LED_PIN_G);
    gpio_set_dir(LED_PIN_G, GPIO_OUT);

}

void vread_temp(void *pvParameters) {
    const float conversion_factor = 3.3f / 4095;
    float adc = 0, tempC = 0;
    float voltage = 0;
    
    adc_set_temp_sensor_enabled(true);

    for (;;) {
        adc_select_input(TEMPERATURE_SENSOR);
        
        adc = adc_read();
        voltage = adc * conversion_factor;
        tempC = 27 - (voltage - 0.706f) / 0.001721f;

        printf("Temperatura: %d C\n", (int)tempC);

        char buffer[50];
        snprintf(buffer, sizeof(buffer), "%d", (int)tempC);
        mqtt_comm_publish("ha/desafio04/henrique.corneau/temp", buffer, strlen(buffer), 1);

        gpio_put(LED_PIN_G, 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        gpio_put(LED_PIN_G, 0);
        vTaskDelay(pdMS_TO_TICKS(950));

        printf("Piscando\n");
        
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void vjoystick_direction(void *pvParameters)
{
    uint16_t x = 0, y = 0, sw = 0;
    uint8_t direction[4] = {0}; // Cima, Baixo, Esquerda, Direita

    uint8_t last_direction = 5; // Variável para armazenar a última direção

    for (;;)
    {
        joystick_read_axis(&x, &y, &sw);
        
        if (x > 3000) {
            direction[0] = 1; direction[1] = 0; direction[2] = 0; direction[3] = 0;
        } else if (x < 1000) {
            direction[0] = 0; direction[1] = 1; direction[2] = 0; direction[3] = 0;
        } else if (y > 3000) {
            direction[0] = 0; direction[1] = 0; direction[2] = 0; direction[3] = 1;
        } else if (y < 1000) {
            direction[0] = 0; direction[1] = 0; direction[2] = 1; direction[3] = 0;
        }
        else {
            direction[0] = 0; direction[1] = 0; direction[2] = 0; direction[3] = 0;
        }

        for (int i=0; i < 4; i++) {
            if (direction[i]) {
                switch (i)
                {
                case 0:
                    if (last_direction != 0) {
                        printf("Cima\n");
                        mqtt_comm_publish("ha/desafio04/henrique.corneau/joy", "cima", 4, 1);
                        last_direction = 0; // Atualiza a última direção
                    }
                    break;
                case 1:
                    if (last_direction != 1) {
                        printf("Baixo\n");
                        mqtt_comm_publish("ha/desafio04/henrique.corneau/joy", "baixo", 5, 1);
                        last_direction = 1; // Atualiza a última direção
                    }
                    break;
                case 2:
                    if (last_direction != 2) {
                        printf("Esquerda\n");
                        mqtt_comm_publish("ha/desafio04/henrique.corneau/joy", "esquerda", 8, 1);
                        last_direction = 2; // Atualiza a última direção
                    }
                    break;
                case 3:
                    if (last_direction != 3) {
                        printf("Direita\n");
                        mqtt_comm_publish("ha/desafio04/henrique.corneau/joy", "direita", 7, 1);
                        last_direction = 3; // Atualiza a última direção
                    }
                    break;
                default:
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
