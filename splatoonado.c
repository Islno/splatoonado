#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "ws2818b.pio.h"

// Definições de pinos e constantes
#define LED_COUNT 25          // Número de LEDs na matriz
#define LED_PIN 7             // Pino de controle dos LEDs WS2812B
#define BUTTON_A_PIN 5        // Botão A no GPIO5 (direita)
#define BUTTON_B_PIN 6        // Botão B no GPIO6 (esquerda)
#define BUZZER_PIN 21         // Pino do buzzer
#define MOVE_DELAY 200        // Delay entre movimentos (em ms)
#define BUZZER_FREQUENCY 100  // Frequência do buzzer (em Hz)

// Protótipos das funções
void npInit(uint pin);        // Inicializa o controlador de LEDs
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);  // Define a cor de um LED
void npClear(void);           // Apaga todos os LEDs
void npWrite(void);           // Envia os dados de cor para os LEDs
void pwm_init_buzzer(uint pin);  // Inicializa o PWM para o buzzer
void beep(uint pin, uint duration_ms);  // Emite um som no buzzer

// Estrutura para armazenar as cores dos LEDs
struct pixel_t {
    uint8_t G, R, B;  // Cores no formato GRB (Green, Red, Blue)
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;

// Variáveis globais
npLED_t leds[LED_COUNT];  // Array para armazenar as cores dos LEDs
PIO np_pio;               // Controlador PIO usado para os LEDs
uint sm;                  // State Machine do PIO

// Ciclo de cores (cores fracas)
uint8_t color_cycle[][3] = {
    {20, 0, 20},   // Roxo fraco
    {0, 0, 20},    // Azul fraco
    {0, 20, 0},    // Verde fraco
    {20, 0, 0},    // Vermelho fraco
    {20, 20, 0},   // Amarelo fraco
    {0, 20, 20},   // Ciano fraco
    {20, 0, 20},   // Magenta fraco
    {20, 10, 0}    // Laranja fraco
};
int color_index[LED_COUNT] = {0};  // Índice da cor atual de cada LED
bool visited[LED_COUNT] = {false}; // Rastreia quais LEDs foram visitados

// Função para inicializar o controlador de LEDs
void npInit(uint pin) {
    // Adiciona o programa PIO para controlar os LEDs WS2812B
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
    }
    // Inicializa o programa PIO no pino especificado
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
    npClear();  // Apaga todos os LEDs
    npWrite();  // Envia os dados para os LEDs
}

// Função para definir a cor de um LED específico
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

// Função para apagar todos os LEDs
void npClear(void) {
    for (uint i = 0; i < LED_COUNT; ++i)
        npSetLED(i, 0, 0, 0);  // Define todas as cores como 0 (apagado)
}

// Função para enviar os dados de cor para os LEDs
void npWrite(void) {
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);  // Envia o componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].R);  // Envia o componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].B);  // Envia o componente azul
    }
    sleep_us(100);  // Pequeno delay para garantir a atualização dos LEDs
}

// Função para inicializar o PWM no pino do buzzer
void pwm_init_buzzer(uint pin) {
    // Configura o pino como saída de PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Obtém o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configura o PWM com a frequência desejada
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096)); // Divisor de clock
    pwm_init(slice_num, &config, true);

    // Inicia o PWM no nível baixo
    pwm_set_gpio_level(pin, 0);
}

// Função para emitir um som no buzzer
void beep(uint pin, uint duration_ms) {
    // Obtém o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configura o duty cycle para 50% (ativo)
    pwm_set_gpio_level(pin, 2048);

    // Temporização
    sleep_ms(duration_ms);

    // Desativa o sinal PWM (duty cycle 0)
    pwm_set_gpio_level(pin, 0);

    // Pausa entre os beeps
    sleep_ms(100); // Pausa de 100ms
}

// Função principal
int main() {
    stdio_init_all();  // Inicializa o sistema de saída padrão
    npInit(LED_PIN);   // Inicializa o controlador de LEDs

    // Configuração dos botões
    gpio_init(BUTTON_A_PIN);
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);  // Resistor de pull-up interno
    gpio_pull_up(BUTTON_B_PIN);  // Resistor de pull-up interno

    // Inicializa o PWM no pino do buzzer
    pwm_init_buzzer(BUZZER_PIN);

    int current_led = 12;  // LED inicial no centro da matriz 5x5
    int previous_led = -1;
    visited[current_led] = true;

    uint64_t last_move_time = 0;

    // Define a cor inicial personalizada (roxo fraco)
    npSetLED(current_led, 20, 0, 20);  // Roxo fraco (R: 20, G: 0, B: 20)
    npWrite();

    // Loop principal
    while (true) {
        // Verifica botões (LOW quando pressionado)
        bool button_a = !gpio_get(BUTTON_A_PIN);  // Botão A (direita)
        bool button_b = !gpio_get(BUTTON_B_PIN);  // Botão B (esquerda)

        int new_led = current_led;

        // Verifica se o tempo desde o último movimento foi maior que o delay
        if (to_ms_since_boot(get_absolute_time()) - last_move_time > MOVE_DELAY) {
            // Movimento para direita (Botão A)
            if (button_a) {
                if (current_led == LED_COUNT - 1) {
                    new_led = 0;  // Vai para o início da matriz (linha 1, coluna 1)
                } else {
                    new_led += 1;  // Move para a direita
                }
                beep(BUZZER_PIN, 100);  // Emitir som ao mover para a direita
            }
            // Movimento para esquerda (Botão B)
            else if (button_b) {
                if (current_led == 0) {
                    new_led = LED_COUNT - 1;  // Vai para o final da matriz (linha 5, coluna 5)
                } else {
                    new_led -= 1;  // Move para a esquerda
                }
                beep(BUZZER_PIN, 100);  // Emitir som ao mover para a esquerda
            }

            // Atualiza a posição do LED
            if (new_led != current_led) {
                previous_led = current_led;
                current_led = new_led;
                visited[current_led] = true;
                last_move_time = to_ms_since_boot(get_absolute_time());

                // Atualiza o LED anterior para a próxima cor no ciclo
                if (previous_led != -1) {
                    color_index[previous_led] = (color_index[previous_led] + 1) % 8;  // 8 cores no ciclo
                    npSetLED(previous_led, 
                            color_cycle[color_index[previous_led]][0],
                            color_cycle[color_index[previous_led]][1],
                            color_cycle[color_index[previous_led]][2]);
                }
            }
        }

        // Atualiza LED atual para a cor personalizada (roxo fraco)
        npSetLED(current_led, 20, 0, 20);  // Roxo fraco (R: 20, G: 0, B: 20)
        npWrite();

        // Pequeno delay para evitar leituras inconsistentes (debounce simples)
        sleep_ms(50);
    }
}